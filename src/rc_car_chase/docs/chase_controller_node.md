# `chase_controller_node.py` 상세 설명

RC카 추적의 **메인 컨트롤러**. **PHASE1(장거리)도 PHASE2(근거리)도 둘 다 타겟 월드좌표를
Nav2(`NavigateToPose` 액션)로 넘겨 자율주행에 맡기는 상태 머신 노드.** 차이는 딱 하나 — PHASE1은
타겟 좌표 그대로 goal을 보내고, PHASE2는 로봇→타겟 방향으로 `target_distance`만큼 못 미친 지점을
goal로 보내서 차와 일정 거리(기본 0.6m)를 유지하며 따라간다.

타겟 좌표의 **주 출처는 웹캠**(고정 외부 카메라 + 호모그래피)이고, 자체 OAK-D 카메라(YOLO+뎁스)는
기본적으로 "차가 실제로 가까이 왔는지" PHASE1→PHASE2 전환을 확정하는 용도로만 쓰인다. 다만 PHASE2에서
**웹캠 타겟이 stale해지면**(예: 차가 로봇 바로 앞이라 고정 웹캠 시야를 벗어난 경우) 자체 카메라의
뎁스를 카메라 내부파라미터(`CameraInfo`)와 TF로 월드좌표로 변환해 **자동으로 fallback**한다
(`_update_own_cam_world_fallback`).

---

## 1. 모듈 레벨 상수

| 이름 | 기능 |
|---|---|
| `PHASE1_APPROACH` | 상태 문자열 상수 — 웹캠 타겟 좌표 그대로 Nav2 목표로 보내 접근하는 단계 (타겟 없으면 자동 탐색 or 대기) |
| `PHASE2_FOLLOW` | 상태 문자열 상수 — 웹캠 타겟에서 `target_distance`만큼 못 미친 지점을 Nav2 목표로 보내 일정 거리 유지하며 따라가는 단계 |
| `SUB_MODE_IDLE` | PHASE1 하위 모드: 웹캠 타겟도 없고 자동 탐색도 꺼져있어 대기 중 |
| `SUB_MODE_EXPLORING` | PHASE1 하위 모드: 웹캠 타겟이 없어 `explore_lite`에 탐색을 맡긴 상태 |
| `SUB_MODE_NAVIGATING` | PHASE1 하위 모드: 웹캠 타겟이 있어 Nav2에 그 좌표로 목표를 보낸 상태 |
| `SUB_MODE_UNDOCKING` | PHASE1 하위 모드: 웹캠 타겟은 있는데 아직 도킹된 상태라 먼저 언도킹 중 (완료되어야 Nav2 목표 전송으로 넘어감) |

---

## 2. 파라미터 (`_declare_parameters` / `_read_parameters`)

### 2-1. 토픽/모델 경로

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `own_cam_rgb_topic` | `/robot5/oakd/rgb/image_raw/compressed` | 자체 RGB 카메라 압축 이미지 토픽 |
| `own_cam_depth_topic` | `/robot5/oakd/stereo/image_raw/compressedDepth` | 자체 뎁스 이미지 토픽 (압축뎁스) |
| `own_cam_info_topic` | `/robot5/oakd/rgb/camera_info` | 자체 카메라 내부파라미터(K 행렬) 토픽 — PHASE2 뎁스 fallback의 픽셀→카메라좌표 변환에 사용 |
| `webcam_target_topic` | `/rc_car_chase/webcam_target` | `webcam_locator_node`가 발행하는 타겟 월드좌표 토픽 |
| `own_cam_model_path` | `.../car_dum_yolo11n_seqsplit/weights/best.pt` | 자체 카메라용 YOLO 가중치 경로 (핸드오프 확인용) |
| `tracker` | `bytetrack.yaml` | Ultralytics 트래커 설정 |
| `debug_image_topic` | `/rc_car_chase/debug_image` | 디버그 오버레이 발행 토픽 |
| `odom_topic` | `/robot5/odom` | PHASE2 오프셋 계산에 쓰는 로봇 자기 위치 (`pose.pose.position`)를 읽는 오도메트리 토픽 |

### 2-2. 탐지/핸드오프 관련

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `own_cam_conf_threshold` | `0.5` | YOLO 탐지 최소 신뢰도 |
| `own_cam_confirm_conf` | `0.6` | "확실히 확인됨"으로 카운트하는 신뢰도 기준 |
| `own_cam_confirm_frames` | `4` | 이 신뢰도 이상이 연속 몇 프레임이면 PHASE2로 전환할지 |
| `own_cam_handoff_max_depth_m` | `0.7` | 전환 시점에 뎁스로 잰 거리가 이보다 멀면 전환 보류 — 자체 카메라 기준 0.7m 이내로 들어와야 PHASE2로 전환됨 (`0` 이하면 이 체크 비활성) |
| `target_class_id` | `0` | 추적 대상 클래스 ID |

### 2-3. 거리/뎁스

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `target_distance` | `0.6` (m) | **PHASE2에서 웹캠 타겟으로부터 유지할 거리(goal 오프셋)로 재사용됨.** 로봇→타겟 직선상에서 타겟으로부터 이 거리만큼 못 미친 지점이 goal이 됨 |
| `depth_scale` | `0.001` | 뎁스 raw 값(보통 mm) → 미터 변환 배율 |
| `depth_patch_size` | `5` | 뎁스 샘플링 시 중심점 주변 패치 크기(픽셀) |
| `depth_stale_timeout` | `0.5` (s) | 뎁스 데이터가 이보다 오래되면 무효 처리 (핸드오프 판단에서 제외) |

### 2-4. 타임아웃(신선도 체크)

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `webcam_stale_timeout` | `1.0` (s) | 웹캠 타겟이 오래되면 PHASE1/PHASE2 모두 Nav2 목표 전송 중단 |

### 2-5. QoS/동기화

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `depth_qos_best_effort` | `False` | 뎁스 구독 QoS를 BEST_EFFORT로 할지 (기본은 RELIABLE) |
| `rgb_depth_sync_slop` | `0.1` (s) | RGB-뎁스 근사 시간동기화 허용 오차 |

### 2-6. Nav2 / explore_lite 연동

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `nav2_action_name` | `/robot5/navigate_to_pose` | Nav2 `NavigateToPose` 액션 서버 이름 |
| `nav2_goal_frame_id` | `odom` | 목표 좌표(`PoseStamped.header.frame_id`)의 기준 프레임 (웹캠 타겟·odom 좌표계와 일치해야 함) |
| `nav2_goal_update_threshold_m` | `0.5` | 마지막으로 **수락된** goal에서 이 거리 이상 움직였을 때만 새 Nav2 목표 재전송 (너무 잦으면 이전 goal이 실행 중간에 계속 preempt당해 ABORTED가 반복됨) |
| `enable_autonomous_exploration` | `False` | 웹캠 타겟이 없을 때(PHASE1) `explore_lite`에 탐색을 맡길지 여부 (**SAFE DEFAULT**) |
| `explore_resume_topic` | `/robot5/explore/resume` | `explore_lite`에 탐색 재개/일시정지를 알리는 `std_msgs/Bool` 토픽 |
| `handoff_event_topic` | `/rc_car_chase/handoff_event` | PHASE1→PHASE2 전환 순간 RViz에 표시할 `visualization_msgs/Marker`를 발행하는 토픽 |

### 2-7. 자동 언도킹

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `auto_undock` | `True` | 웹캠이 타겟을 잡았는데 로봇이 아직 도킹 상태면 자동으로 `undock` 액션을 보낼지 |
| `dock_status_topic` | `/robot5/dock_status` | 도킹 상태(`irobot_create_msgs/msg/DockStatus`) 구독 토픽 |
| `undock_action_name` | `/robot5/undock` | `irobot_create_msgs/action/Undock` 액션 서버 이름 |

### 2-8. 실행 제어

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `control_rate_hz` | `10.0` | 제어 타이머 주기 |
| `publish_debug_image` | `True` | 디버그 오버레이 발행 여부 |
| `show_window` | `False` | 로컬 창 표시 여부 |
| `enable_cmd_vel` | `False` | **실제 발행 여부 (안전 기본값=dry-run)**. Nav2 목표 전송, undock, explore resume 신호 전부 이 값이 `False`면 실제 전송 대신 로그만 |
| `initial_state` | `PHASE1_APPROACH` | 시작 상태 |

---

## 3. 인스턴스 변수 (캐시된 센서 상태)

| 변수명 | 갱신 콜백 | 기능 |
|---|---|---|
| `self.state` | `tick_phase1` | 현재 상태 (`PHASE1_APPROACH` / `PHASE2_FOLLOW`) |
| `self.latest_webcam_target`, `self.latest_webcam_stamp` | `on_webcam_target` | 웹캠이 알려준 최신 타겟 좌표와 수신 시각 — PHASE1/PHASE2 모두 이걸로 주행 |
| `self.latest_odom_xy` | `on_odom` | 로봇 자신의 현재 위치 (`nav2_goal_frame_id`와 같은 프레임의 odom `pose.pose.position`) — PHASE2 오프셋 계산용 |
| `self.own_cam_K`, `self.own_cam_frame_id` | `on_own_cam_info` | 자체 카메라 내부파라미터(fx, fy, cx, cy)와 그 좌표계의 TF 프레임 이름 — 뎁스 fallback 변환에 사용 |
| `self.latest_own_cam_world_xy`, `self.latest_own_cam_world_stamp` | `_update_own_cam_world_fallback` | 자체 카메라 뎁스를 TF로 `nav2_goal_frame_id`에 투영한 월드좌표와 시각 — PHASE2에서 웹캠 타겟이 stale할 때 fallback 타겟으로 사용 |
| `self.tf_buffer`, `self.tf_listener` | (프레임워크가 자동 갱신) | `tf2_ros.Buffer`/`TransformListener` — 뎁스 fallback의 카메라→`nav2_goal_frame_id` 변환에 사용 |
| `self.latest_own_cam_bbox`, `self.latest_own_cam_conf`, `self.latest_own_cam_stamp` | `on_oakd_synced` | 자체 카메라 최신 타겟 바운딩박스/신뢰도/시각 (핸드오프 판단 전용) |
| `self.latest_depth_image`, `self.latest_depth_stamp` | `on_oakd_synced` | 최신 디코딩된 뎁스 이미지와 시각 (핸드오프 판단 전용) |
| `self.own_cam_confirm_count` | `on_oakd_synced` | 고신뢰도 연속 탐지 카운트 (핸드오프 판단용) |
| `self.locked_track_id` | `on_oakd_synced` | 현재 고정 추적 중인 ByteTrack ID |
| `self._depth_debug_printed` | `on_oakd_synced` | 뎁스 sanity-check 로그를 한 번만 찍기 위한 플래그 |
| `self.frame_count` | `on_oakd_synced` | 처리된 동기화 프레임 수 |
| `self.latest_is_docked`, `self.latest_dock_stamp` | `on_dock_status` | 최신 도킹 상태(`irobot_create_msgs/msg/DockStatus.is_docked`)와 수신 시각 |
| `self._nav2_goal_handle` | Nav2 콜백들 | 현재 진행 중인 Nav2 목표의 goal handle (취소용) |
| `self._nav2_goal_pending` | Nav2 콜백들 | accept/reject 응답을 기다리는 중인지 여부 (중복 전송 방지) |
| `self._nav2_last_goal_xy` | `_on_nav2_goal_response`/`_maybe_send_nav2_goal` | 마지막으로 **수락된** 목표 좌표 (재전송 threshold 판단용) |
| `self._phase1_sub_mode` | `_enter_phase1_sub_mode` | PHASE1의 현재 하위 모드, 바뀔 때만 로그 |
| `self._explore_resume_published` | `_set_explore_resume` | 마지막으로 발행한 explore resume 값 (중복 발행 방지) |
| `self._undock_goal_pending` | 언도킹 콜백들 | 언도킹 goal의 accept/reject·결과 응답을 기다리는 중인지 여부 (중복 전송 방지) |

---

## 4. 함수 상세

### 초기화

| 함수 | 기능 |
|---|---|
| `__init__` | 파라미터 로드 → YOLO 모델(핸드오프 확인용)/CvBridge 준비 → 상태 변수 초기화 → QoS 2종 설정(reliable/depth) → 콜백그룹 4개 생성(light/vision/nav2·undock/timer) → 구독 3개(webcam target, dock status, odom) + RGB/뎁스 동기화 구독 → 퍼블리셔(debug image, explore resume, handoff event) + Nav2 `NavigateToPose` / `Undock` 액션 클라이언트 → 제어 타이머 생성 |
| `_declare_parameters` | 위 표의 모든 파라미터를 기본값과 함께 선언 |
| `_read_parameters` | 선언된 파라미터 값을 읽어 `self.*` 멤버 변수로 캐싱 |

### 콜백 (센서 입력 → 상태 캐싱만 수행, 제어는 안 함)

| 함수 | 콜백 그룹 | 기능 |
|---|---|---|
| `on_webcam_target(msg)` | cb_light | 웹캠 타겟 좌표 캐싱 |
| `on_dock_status(msg)` | cb_light | 도킹 상태(`is_docked`)와 수신 시각 캐싱 |
| `on_odom(msg)` | cb_light | `msg.pose.pose.position`을 그대로 캐싱 (속도 적분 없음, 순수 현재 위치 참조용) |
| `on_own_cam_info(msg)` | cb_light | `msg.k`에서 fx, fy, cx, cy 추출 + `msg.header.frame_id` 캐싱 (뎁스 fallback용) |
| `on_oakd_synced(rgb_msg, depth_msg)` | cb_vision (message_filters 동기화 콜백) | 아래 "5. `on_oakd_synced` 상세" 참고 |

### 제어 루프

| 함수 | 기능 |
|---|---|
| `on_control_timer` | 매 `1/control_rate_hz` 초마다 실행. `PHASE1_APPROACH`면 `tick_phase1`, 아니면 `tick_phase2` 호출. **어느 쪽도 twist를 직접 발행하지 않음** — 실제 주행은 전부 Nav2에 위임 |
| `tick_phase1(now)` | **PHASE1 로직**: 핸드오프 조건(6번 참고) 확인 → 만족하면 `_publish_handoff_event()` 발행, 진행 중인 Nav2 목표 취소, PHASE2로 전환 후 리턴. 아니면 웹캠 타겟 신선도로 분기: 신선하고 `auto_undock=True`이며 아직 도킹 상태면 `UNDOCKING`(언도킹 goal 전송); 신선하면(언도킹 불필요) `NAVIGATING_TO_TARGET`(타겟 좌표 그대로 Nav2 목표 전송, `_maybe_send_nav2_goal`); 안 신선하고 `enable_autonomous_exploration=True`면 `EXPLORING`(Nav2 목표 취소 + explore resume 켬); 둘 다 아니면 `IDLE`(Nav2 목표 취소 + explore resume 끔) |
| `tick_phase2(now)` | **PHASE2 로직**: 타겟 좌표 출처를 결정 — 웹캠 타겟이 신선하면 그걸 씀; 안 신선한데 `latest_own_cam_world_xy`가 신선하면(뎁스 fallback) 그걸 대신 씀; 둘 다 없으면 Nav2 목표 취소하고 리턴. 자기 위치(`latest_odom_xy`)도 모르면 마찬가지로 취소 후 리턴. 그 외엔 로봇→타겟 벡터 `(dx, dy)`와 거리 `dist`를 계산 → `dist <= target_distance`면 이미 충분히 가까우니 아무것도 안 함(붙지 않도록) → 아니면 그 직선상에서 타겟으로부터 `target_distance`만큼 못 미친 지점 `(goal_x, goal_y)`와 타겟을 바라보는 방향 `yaw = atan2(dy, dx)`를 계산해 `_maybe_send_nav2_goal(goal_x, goal_y, yaw=yaw)` 호출 |
| `_sample_depth_at_bbox_center(now)` | 현재 바운딩박스 중심 픽셀에서 뎁스 패치 샘플링 (`vision_utils.sample_depth_patch` 호출). 뎁스가 stale하거나 없으면 `None` — 핸드오프 판단과 뎁스 fallback 둘 다에 쓰임 |
| `_update_own_cam_world_fallback(now)` | `own_cam_K`/`own_cam_frame_id`가 아직 없으면 스킵. bbox 중심 픽셀 `(u,v)`와 뎁스 `z`로 카메라 좌표계 3D 점 `((u-cx)*z/fx, (v-cy)*z/fy, z)`을 만들어 `PointStamped`(stamp는 0, "최신 변환 사용" 의미)로 감싼 뒤 `tf_buffer.transform(...)`으로 `nav2_goal_frame_id`로 변환. 변환 실패 시 경고 로그만 남기고 스킵, 성공하면 `latest_own_cam_world_xy`/`_stamp` 갱신 |
| `_publish_handoff_event()` | PHASE1→PHASE2 전환 순간 `base_link` 프레임 기준으로 노란 구(SPHERE) + `"HANDOFF: PHASE2_FOLLOW"` 텍스트(TEXT_VIEW_FACING) 마커를 `handoff_event_topic`에 발행 (수명 2초짜리 일회성 플래시) |

### Nav2 / explore_lite 연동

| 함수 | 기능 |
|---|---|
| `_enter_phase1_sub_mode(name)` | PHASE1 하위 모드가 바뀔 때만 `self._phase1_sub_mode` 갱신 + 로그 (매 틱마다 로그 스팸 방지) |
| `_set_explore_resume(want_resume)` | `explore_resume_topic`에 `Bool` 발행. `enable_cmd_vel=False`(dry-run)면 항상 `False`로 강제. 이전과 값이 같으면 재발행 안 함 |
| `_maybe_send_nav2_goal(x, y, yaw=None)` | 아직 이전 목표의 accept/reject 응답 대기 중이면 스킵. 마지막으로 **수락된** 목표에서 `nav2_goal_update_threshold_m`만큼 안 움직였으면 스킵(리플랜 최소화). `enable_cmd_vel=False`면 실제 전송 없이 `[DRY RUN]` 로그만. 그 외엔 `_send_nav2_goal` 호출. `yaw`는 PHASE1에선 안 넘겨서(`None`) 정면(`w=1.0`) 고정, PHASE2에선 타겟을 바라보는 방향을 넘김 |
| `_send_nav2_goal(x, y, yaw=None)` | 액션 서버 준비 안 됐으면 경고 후 스킵. 진행 중인 이전 goal이 있으면 새로 보내기 전에 먼저 취소(Nav2의 암묵적 preempt에 기대지 않고 우리 쪽 goal handle 상태를 명확하게 유지). `nav2_goal_frame_id` 기준 `PoseStamped`(`yaw`가 있으면 그 방향의 쿼터니언, 없으면 `w=1.0`)를 만들어 `NavigateToPose.Goal`로 비동기 전송, 응답 콜백(`_on_nav2_goal_response`) 등록. **`header.stamp`는 일부러 0(`Time()`)으로 둠** — "지금 이 순간"으로 못박으면, 메시지가 DDS 큐에서 지연되다 도착했을 때 그 시각이 이미 TF 버퍼 보관기간을 벗어나 즉시 extrapolation 에러로 ABORT될 수 있음. 0으로 두면 tf2가 "최신 변환을 써라"로 해석해서 이 문제를 피함 |
| `_on_nav2_goal_response(future, x, y)` | 전송 실패/거부 시 로그만 남기고 종료 — **이때 `_nav2_last_goal_xy`를 갱신하지 않음** (거부된 좌표가 "마지막으로 보낸 목표"로 남아 이후 재전송이 막히는 걸 방지). 수락되면 그제서야 `_nav2_last_goal_xy = (x, y)` 기록, `goal_handle` 저장하고 결과 콜백(`_on_nav2_result`) 등록 |
| `_on_nav2_result(future)` | 목표 완료 시 상태(`SUCCEEDED`/`CANCELED`/`ABORTED`/기타)를 로그로 출력 |
| `_cancel_nav2_goal_if_active()` | 진행 중인 Nav2 목표가 있으면 비동기 취소 요청 후 goal handle/pending/last-goal 상태를 모두 초기화 |
| `_maybe_send_undock_goal()` | 이전 언도킹 goal 응답/결과를 기다리는 중이면 스킵. `enable_cmd_vel=False`면 `[DRY RUN]` 로그만. 액션 서버 준비 안 됐으면 경고. 그 외엔 `Undock.Goal()`을 비동기 전송(`_on_undock_goal_response` 등록) — day3_pkg의 `TurtleBot4Navigator.undock()`과 같은 액션을 쓰지만, 그건 내부적으로 `rclpy.spin_until_future_complete`로 **블로킹**하는 방식이라 이 노드의 비동기 타이머/콜백 구조에 안 맞아 raw `ActionClient` + 콜백으로 직접 구현함 |
| `_on_undock_goal_response(future)` | 전송 실패/거부 시 로그 남기고 `_undock_goal_pending` 해제. 수락되면 결과 콜백(`_on_undock_result`) 등록 |
| `_on_undock_result(future)` | `_undock_goal_pending` 해제 + 완료 로그. 언도킹이 실제로 끝났는지는 다음 틱의 `on_dock_status`가 갱신한 `latest_is_docked`로 반영됨 |

### 종료/진입점

| 함수 | 기능 |
|---|---|
| `destroy_node` | 진행 중인 Nav2 목표가 있으면 취소(show_window면 창 정리) 후 노드 정리 — 종료 시 로봇이 계속 주행하는 것 방지 |
| `main` | rclpy 초기화 → 노드 생성 → **`MultiThreadedExecutor`**로 spin → 종료 처리 |

---

## 5. `on_oakd_synced` 상세 (핸드오프 확인 + 뎁스 fallback 계산)

`message_filters.ApproximateTimeSynchronizer`가 RGB와 뎁스를 시간 맞춰 동시에 넘겨줄 때 호출됨.
**여기서 계산되는 값들은 PHASE1→PHASE2 핸드오프 판단(6번)과 PHASE2 뎁스 fallback(`latest_own_cam_world_xy`)에
쓰이고, 웹캠 타겟이 살아있는 한 실제 주행 방향/속도 계산엔 관여하지 않는다.**

1. RGB 압축 이미지를 `bgr8` OpenCV 이미지로 디코딩
2. 뎁스는 `decode_compressed_depth`로 디코딩. 실패하면 경고만 찍고 계속 진행
3. 뎁스 디코딩 성공 시, **최초 1회만** dtype/min/max/nonzero_min을 로그로 출력해서 `depth_scale` 파라미터가 실제 데이터 단위와 맞는지 검증할 수 있게 함
4. `own_cam_model.track()`으로 YOLO+ByteTrack 추론
5. `select_target_box`로 최종 타겟 박스 선택
   - 있으면: bbox/신뢰도/시각 캐싱, 신뢰도가 `own_cam_confirm_conf` 이상이면 `own_cam_confirm_count` 증가(최대 `own_cam_confirm_frames`), 아니면 리셋. 이어서 `_update_own_cam_world_fallback(now)` 호출해 뎁스 fallback 좌표 갱신
   - 없으면: 락 해제, 카운트 리셋 (fallback 좌표는 갱신 안 됨 — `depth_stale_timeout` 지나면 `tick_phase2`가 알아서 stale로 판단)
6. `publish_debug_image=True`면 오버레이 이미지를 만들어 발행(+옵션으로 로컬 창 표시)

---

## 6. 상태 전환(핸드오프) 조건 요약

```
PHASE1_APPROACH  (goal = 웹캠 타겟 좌표 그대로)
  ├─ 하위 모드 (매 틱 재평가):
  │    ├─ 웹캠 타겟 fresh + 아직 도킹 상태(auto_undock=True) → UNDOCKING: 언도킹 goal 전송, explore resume=False
  │    ├─ 웹캠 타겟 fresh (언도킹 불필요)  → NAVIGATING_TO_TARGET: Nav2에 (x,y) 목표 전송, explore resume=False
  │    ├─ 타겟 stale + explore=on  → EXPLORING: Nav2 목표 취소, explore resume=True
  │    └─ 타겟 stale + explore=off → IDLE: Nav2 목표 취소, explore resume=False
  │
  └─ own_cam_confirm_count >= own_cam_confirm_frames  (자체 카메라 연속 고신뢰도 탐지)
       AND (own_cam_handoff_max_depth_m <= 0  OR  뎁스로 잰 거리 <= own_cam_handoff_max_depth_m)
       → 진행 중인 Nav2 목표 취소
       ─────────────────────────────────────────────► PHASE2_FOLLOW

PHASE2_FOLLOW  (goal = 타겟에서 로봇 방향으로 target_distance만큼 못 미친 지점)
  ├─ 웹캠 타겟 fresh                          → 웹캠 좌표로 오프셋 goal 계산
  ├─ 웹캠 타겟 stale + 뎁스 fallback 좌표 fresh → 자체카메라 뎁스→월드 좌표로 오프셋 goal 계산
  └─ 둘 다 없음 (또는 자기 위치를 모름)         → 진행 중인 Nav2 목표 취소하고 대기
```

※ PHASE2 → PHASE1로 되돌아가는 로직은 없음 (한 번 핸드오프되면 계속 PHASE2로 남음).
※ 실제 주행(경로 계획, 장애물 회피)은 이 노드가 아니라 **PHASE1/PHASE2 둘 다 Nav2**(+로컬/글로벌
  costmap)가 담당한다 — 이 노드는 목표 좌표를 계산해서 넘기고 취소하는 역할만 한다. 자체 카메라와
  라이다는 이제 이 노드에서 직접 주행 제어에 쓰이지 않는다 (라이다 안전도 Nav2 costmap이 담당).

---

## 7. 이 노드가 사용하는 `vision_utils.py` 함수

| 함수 | 용도 (이 노드에서) |
|---|---|
| `decode_compressed_depth(data)` | `compressedDepth` 토픽의 raw payload(12바이트 헤더 + PNG)를 16UC1 뎁스 이미지로 디코딩 |
| `sample_depth_patch(depth_img, u, v, patch_size, depth_scale)` | 바운딩박스 중심 주변 패치의 중앙값 뎁스를 미터로 변환 (핸드오프 판단용) |
| `select_target_box(boxes, target_cls_id, locked_track_id)` | 타겟 클래스 박스들 중 고정 트랙ID 우선, 없으면 최고 신뢰도 박스 선택 |

자세한 내용은 [`vision_utils.md`](vision_utils.md) 참고.

---

## 8. 안전장치 요약

- **`enable_cmd_vel=False`가 기본값** → 실수로 로봇이 움직이는 것 방지 (dry-run 로그만, Nav2 목표/undock/explore resume 전부 실제 전송 대신 로그만)
- **`enable_autonomous_exploration=False`가 기본값** → 웹캠 타겟이 없어도 로봇이 알아서 돌아다니지 않음
- 웹캠/뎁스에 **stale timeout** 체크 → 오래된 데이터로 잘못된 목표 전송 방지
- 장애물 회피는 PHASE1/PHASE2 모두 **Nav2 costmap**이 담당 (이 노드는 별도 라이다 안전 레이어를 두지 않음)
- 종료 시 진행 중인 Nav2 목표 취소
- 뎁스 핸드오프 거리 체크(`own_cam_handoff_max_depth_m`)로 너무 먼 오탐지에 의한 조기 전환 방지
- PHASE2에서 이미 `target_distance` 이내로 가까우면 goal을 안 보내서 차를 밀고 들어가지 않음
