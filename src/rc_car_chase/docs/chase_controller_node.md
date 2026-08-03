# `chase_controller_node.py` 상세 설명

RC카 추적의 **메인 컨트롤러**. PHASE1에서는 웹캠이 알려준 타겟 좌표를 **Nav2(`NavigateToPose` 액션)**
로 넘겨 장거리 접근을 자율주행에 맡기고(타겟이 안 보이면 `explore_lite`로 프론티어 자동 탐색),
자체 OAK-D 카메라(YOLO+뎁스)로 확인되면 근거리 추적(PHASE2)으로 전환, 라이다로 충돌 방지하며
`cmd_vel`(Twist)을 발행하는 상태 머신 노드.

---

## 1. 모듈 레벨 상수/헬퍼 함수

| 이름 | 기능 |
|---|---|
| `PHASE1_APPROACH` | 상태 문자열 상수 `'PHASE1_APPROACH'` — 웹캠 타겟을 Nav2 목표로 보내 접근하는 단계 (타겟 없으면 자동 탐색 or 대기) |
| `PHASE2_FOLLOW` | 상태 문자열 상수 `'PHASE2_FOLLOW'` — 자체 카메라로 정밀 추적하는 단계 |
| `SUB_MODE_IDLE` | PHASE1 하위 모드: 웹캠 타겟도 없고 자동 탐색도 꺼져있어 대기 중 |
| `SUB_MODE_EXPLORING` | PHASE1 하위 모드: 웹캠 타겟이 없어 `explore_lite`에 탐색을 맡긴 상태 |
| `SUB_MODE_NAVIGATING` | PHASE1 하위 모드: 웹캠 타겟이 있어 Nav2에 그 좌표로 목표를 보낸 상태 |
| `clamp(value, lo, hi)` | 값을 `[lo, hi]` 범위로 제한 (속도/각속도 제한에 반복 사용) |
| `normalize_angle(angle)` | 각도를 `(-π, π]` 범위로 정규화 (bearing 계산 시 사용) |

---

## 2. 파라미터 (`_declare_parameters` / `_read_parameters`)

### 2-1. 토픽/모델 경로

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `own_cam_rgb_topic` | `/robot5/oakd/rgb/image_raw/compressed` | 자체 RGB 카메라 압축 이미지 토픽 |
| `own_cam_depth_topic` | `/robot5/oakd/stereo/image_raw/compressedDepth` | 자체 뎁스 이미지 토픽 (압축뎁스) |
| `cmd_vel_topic` | `/robot5/cmd_vel` | 로봇 속도 명령 발행 토픽 |
| `webcam_target_topic` | `/rc_car_chase/webcam_target` | `webcam_locator_node`가 발행하는 타겟 월드좌표 토픽 |
| `own_cam_model_path` | `.../car_dum_yolo11n_seqsplit/weights/best.pt` | 자체 카메라용 YOLO 가중치 경로 |
| `tracker` | `bytetrack.yaml` | Ultralytics 트래커 설정 |
| `debug_image_topic` | `/rc_car_chase/debug_image` | 디버그 오버레이 발행 토픽 |
| `lidar_topic` | `/robot5/scan` | 라이다 스캔 토픽 |

### 2-2. 탐지/핸드오프 관련

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `own_cam_conf_threshold` | `0.5` | YOLO 탐지 최소 신뢰도 |
| `own_cam_confirm_conf` | `0.6` | "확실히 확인됨"으로 카운트하는 신뢰도 기준 |
| `own_cam_confirm_frames` | `4` | 이 신뢰도 이상이 연속 몇 프레임이면 PHASE2로 전환할지 |
| `own_cam_handoff_max_depth_m` | `0.7` | 전환 시점에 뎁스로 잰 거리가 이보다 멀면 전환 보류 — 즉 PHASE1(Nav2 접근)이 자체 카메라 기준 0.7m 이내로 들어와야 PHASE2로 전환됨 (너무 멀리서 오탐 방지, `0` 이하면 이 체크 비활성) |
| `target_class_id` | `0` | 추적 대상 클래스 ID |

### 2-3. 거리/뎁스 제어

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `target_distance` | `0.5` (m) | PHASE2에서 유지하려는 목표 거리 |
| `distance_deadband` | `0.03` (m) | 이 오차 범위 내에서는 전진/후진 안 함 |
| `depth_scale` | `0.001` | 뎁스 raw 값(보통 mm) → 미터 변환 배율 |
| `depth_patch_size` | `5` | 뎁스 샘플링 시 중심점 주변 패치 크기(픽셀) |
| `depth_stale_timeout` | `0.5` (s) | 뎁스 데이터가 이보다 오래되면 무효 처리 |

### 2-4. 타임아웃(신선도 체크)

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `detection_loss_timeout` | `1.5` (s) | 자체 카메라 탐지가 이보다 오래 없으면 "놓침"으로 간주 → 탐색모드 |
| `webcam_stale_timeout` | `1.0` (s) | 웹캠 타겟이 오래되면 PHASE1에서 Nav2 목표 전송 중단 (idle/exploring로 전환) |
| `lidar_stale_timeout` | `1.0` (s) | 라이다 데이터가 오래되면 안전레이어 스킵 |

### 2-5. QoS/동기화

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `depth_qos_best_effort` | `False` | 뎁스 구독 QoS를 BEST_EFFORT로 할지 (기본은 RELIABLE) |
| `rgb_depth_sync_slop` | `0.1` (s) | RGB-뎁스 근사 시간동기화 허용 오차 |

### 2-6. 카메라 내부 파라미터 (bearing 각도 계산용)

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `fx` | `565.658...` | 카메라 초점거리(픽셀 단위) |
| `cx` | `355.673...` | 이미지 중심 x좌표(픽셀) — `bearing = atan2(u_center - cx, fx)` 계산에 사용 |

### 2-7. PHASE1 Nav2 / explore_lite 연동

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `nav2_action_name` | `/robot5/navigate_to_pose` | Nav2 `NavigateToPose` 액션 서버 이름 |
| `nav2_goal_frame_id` | `odom` | 목표 좌표(`PoseStamped.header.frame_id`)의 기준 프레임 (웹캠 타겟 좌표계와 일치해야 함) |
| `nav2_goal_update_threshold_m` | `0.3` | 새 웹캠 타겟이 기존 목표에서 이 거리 이상 움직였을 때만 새 Nav2 목표 재전송 (너무 잦은 리플랜 방지) |
| `enable_autonomous_exploration` | `False` | 웹캠 타겟이 없을 때 `explore_lite`에 탐색을 맡길지 여부 (**SAFE DEFAULT**) |
| `explore_resume_topic` | `/robot5/explore/resume` | `explore_lite`에 탐색 재개/일시정지를 알리는 `std_msgs/Bool` 토픽 |
| `handoff_event_topic` | `/rc_car_chase/handoff_event` | PHASE1→PHASE2 전환 순간 RViz에 표시할 `visualization_msgs/Marker`를 발행하는 토픽 |

### 2-8. PHASE2 제어 게인

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `phase2_kp_ang` | `1.0` | bearing angle → 각속도 비례 게인 |
| `phase2_max_ang` | `0.6` | 최대 각속도 |
| `phase2_kp_lin` | `0.5` | 거리 오차 → 선속도 비례 게인 |
| `phase2_max_lin` | `0.15` | 최대 전진 속도 |
| `phase2_max_lin_reverse` | `0.08` | 최대 후진 속도 |
| `allow_reverse` | `True` | 너무 가까우면 후진 허용 여부 |

### 2-9. 탐색(search) / 실행 제어

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `search_ang_speed` | `0.3` | (PHASE2) 타겟 놓쳤을 때 회전 탐색 각속도 |
| `search_timeout_sec` | `15.0` | (PHASE2) 탐색 포기 시간 |
| `control_rate_hz` | `10.0` | 제어 타이머 주기 |
| `publish_debug_image` | `True` | 디버그 오버레이 발행 여부 |
| `show_window` | `False` | 로컬 창 표시 여부 |
| `enable_cmd_vel` | `False` | **실제 발행 여부 (안전 기본값=dry-run)**. PHASE2 twist 발행뿐 아니라 `explore_lite` resume 신호도 이 값이 `False`면 항상 억제됨 |
| `initial_state` | `PHASE1_APPROACH` | 시작 상태 |

### 2-10. 라이다 안전

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `lidar_front_half_angle` | `0.44` (rad) | 정면으로 간주할 각도 범위(±) |
| `lidar_safety_stop_distance` | `0.3` (m) | 이보다 가까운 장애물이 있으면 전진 금지 |
| `lidar_forward_offset_rad` | `0.0` | 라이다 정면 기준 보정 오프셋 (라이다 장착 각도 보정용) |

---

## 3. 인스턴스 변수 (캐시된 센서 상태)

| 변수명 | 갱신 콜백 | 기능 |
|---|---|---|
| `self.state` | `tick_phase1` | 현재 상태 (`PHASE1_APPROACH` / `PHASE2_FOLLOW`) |
| `self.latest_webcam_target`, `self.latest_webcam_stamp` | `on_webcam_target` | 웹캠이 알려준 최신 타겟 좌표와 수신 시각 |
| `self.latest_own_cam_bbox`, `self.latest_own_cam_conf`, `self.latest_own_cam_stamp` | `on_oakd_synced` | 자체 카메라 최신 타겟 바운딩박스/신뢰도/시각 |
| `self.latest_depth_image`, `self.latest_depth_stamp` | `on_oakd_synced` | 최신 디코딩된 뎁스 이미지와 시각 |
| `self.latest_lidar_min_front`, `self.latest_lidar_stamp` | `on_lidar` | 전방 최소 거리와 시각 |
| `self.own_cam_confirm_count` | `on_oakd_synced` | 고신뢰도 연속 탐지 카운트 (핸드오프 판단용) |
| `self.locked_track_id` | `on_oakd_synced` | 현재 고정 추적 중인 ByteTrack ID |
| `self.last_known_bearing_sign` | `on_oakd_synced` | 마지막으로 타겟이 화면 좌/우 어디 있었는지 (+1/-1) — 놓쳤을 때 탐색 회전 방향 결정 |
| `self.search_start_time` | `tick_phase2`/`_search_twist` | (PHASE2) 탐색 모드 시작 시각 (타임아웃 판단용) |
| `self._depth_debug_printed` | `on_oakd_synced` | 뎁스 sanity-check 로그를 한 번만 찍기 위한 플래그 |
| `self.frame_count` | `on_oakd_synced` | 처리된 동기화 프레임 수 |
| `self._nav2_goal_handle` | Nav2 콜백들 | 현재 진행 중인 Nav2 목표의 goal handle (취소용) |
| `self._nav2_goal_pending` | Nav2 콜백들 | accept/reject 응답을 기다리는 중인지 여부 (중복 전송 방지) |
| `self._nav2_last_goal_xy` | `_maybe_send_nav2_goal`/`_send_nav2_goal` | 마지막으로 보낸 목표 좌표 (재전송 threshold 판단용) |
| `self._phase1_sub_mode` | `_enter_phase1_sub_mode` | PHASE1의 현재 하위 모드 (IDLE/EXPLORING/NAVIGATING_TO_TARGET), 바뀔 때만 로그 |
| `self._explore_resume_published` | `_set_explore_resume` | 마지막으로 발행한 explore resume 값 (중복 발행 방지) |

---

## 4. 함수 상세

### 초기화

| 함수 | 기능 |
|---|---|
| `__init__` | 파라미터 로드 → YOLO 모델/CvBridge 준비 → 상태 변수 초기화 → QoS 2종 설정(reliable/depth) → 콜백그룹 5개 생성(webcam/lidar/vision/nav2/timer, 병렬 처리 분리) → 구독 2개(webcam target, lidar) + RGB/뎁스 동기화 구독 → 퍼블리셔(cmd_vel, debug image, explore resume) + Nav2 `NavigateToPose` 액션 클라이언트 → 제어 타이머 생성 |
| `_declare_parameters` | 위 표의 모든 파라미터를 기본값과 함께 선언 |
| `_read_parameters` | 선언된 파라미터 값을 읽어 `self.*` 멤버 변수로 캐싱 (매 루프마다 파라미터 서버 조회하지 않도록) |

### 콜백 (센서 입력 → 상태 캐싱만 수행, 제어는 안 함)

| 함수 | 콜백 그룹 | 기능 |
|---|---|---|
| `on_webcam_target(msg)` | cb_light | 웹캠 타겟 좌표 캐싱 |
| `on_lidar(msg)` | cb_lidar | 스캔 전체 중 **정면 반각(`lidar_front_half_angle`) 범위 + 유효 거리(finite, >0.01m)**만 걸러서 최소값 저장. `lidar_forward_offset_rad`로 정면 기준 보정 |
| `on_oakd_synced(rgb_msg, depth_msg)` | cb_vision (message_filters 동기화 콜백) | 아래 "5. `on_oakd_synced` 상세" 참고 |

### 제어 루프

| 함수 | 기능 |
|---|---|
| `on_control_timer` | 매 `1/control_rate_hz` 초마다 실행. `PHASE1_APPROACH`면 `tick_phase1`만 호출하고 리턴(Nav2가 주행을 대신하므로 여기서 twist 발행 안 함); 그 외(PHASE2)는 `tick_phase2`로 twist를 만들고 `_apply_lidar_safety` 적용 → `enable_cmd_vel=True`면 실제 발행, 아니면 `[DRY RUN]` 로그만 출력 |
| `tick_phase1(now)` | **PHASE1 로직**: 핸드오프 조건(아래 참고) 확인 → 만족하면 `_publish_handoff_event()`로 RViz 이벤트 마커 발행 후 진행 중인 Nav2 목표 취소하고 PHASE2로 전환 후 리턴. 아니면 웹캠 타겟 신선도로 분기: 신선하면 `NAVIGATING_TO_TARGET` 하위모드로 들어가 explore resume을 끄고 그 좌표로 Nav2 목표 전송(`_maybe_send_nav2_goal`); 신선하지 않고 `enable_autonomous_exploration=True`면 `EXPLORING`로 들어가 Nav2 목표 취소 + explore resume 켬; 둘 다 아니면 `IDLE`로 들어가 Nav2 목표 취소 + explore resume 끔 |
| `tick_phase2(now)` | **PHASE2 로직**: 최근 탐지가 없으면 `_search_twist` 호출 → 있으면 바운딩박스 중심의 bearing angle로 각속도 계산, 뎁스로 거리 오차 계산해 전진/후진(데드밴드, 후진 허용 여부 반영) |
| `_search_twist(now)` | 타겟을 놓쳤을 때: 처음 놓친 시각 기록 → `search_timeout_sec` 초과 시 정지+에러로그 → 아니면 `last_known_bearing_sign` 방향으로 제자리 회전 |
| `_sample_depth_at_bbox_center(now)` | 현재 바운딩박스 중심 픽셀에서 뎁스 패치 샘플링 (`vision_utils.sample_depth_patch` 호출). 뎁스가 stale하거나 없으면 `None` |
| `_apply_lidar_safety(twist, now)` | 라이다 데이터가 신선하고 전방 최소거리가 안전거리 미만이면 전진 속도를 0 이하로 clamp (후진은 허용). 라이다 데이터가 없거나 오래되면 **안전layer를 건너뜀** (막지 않음). PHASE1은 Nav2 자체 costmap이 장애물을 처리하므로 이 함수는 PHASE2 twist에만 적용됨 |
| `_publish_handoff_event()` | PHASE1→PHASE2 전환 순간 `base_link` 프레임 기준으로 노란 구(SPHERE) + `"HANDOFF: PHASE2_FOLLOW"` 텍스트(TEXT_VIEW_FACING) 마커를 `handoff_event_topic`에 발행 (수명 2초짜리 일회성 플래시, RViz의 Marker 디스플레이로 시각적으로 확인 가능) |

### Nav2 / explore_lite 연동

| 함수 | 기능 |
|---|---|
| `_enter_phase1_sub_mode(name)` | PHASE1 하위 모드가 바뀔 때만 `self._phase1_sub_mode` 갱신 + 로그 (매 틱마다 로그 스팸 방지) |
| `_set_explore_resume(want_resume)` | `explore_resume_topic`에 `Bool` 발행. `enable_cmd_vel=False`(dry-run)면 항상 `False`로 강제 (explore_lite가 실수로 실제 주행하는 것 방지). 이전과 값이 같으면 재발행 안 함 |
| `_maybe_send_nav2_goal(x, y)` | 아직 이전 목표의 accept/reject 응답 대기 중이면 스킵. 마지막으로 보낸 목표에서 `nav2_goal_update_threshold_m`만큼 안 움직였으면 스킵(리플랜 최소화). `enable_cmd_vel=False`면 실제 전송 없이 `[DRY RUN]` 로그만. 그 외엔 `_send_nav2_goal` 호출 |
| `_send_nav2_goal(x, y)` | 액션 서버 준비 안 됐으면 경고 후 스킵. `nav2_goal_frame_id` 기준 `PoseStamped`(orientation은 항상 정면 `w=1.0`)를 만들어 `NavigateToPose.Goal`로 비동기 전송, 응답 콜백(`_on_nav2_goal_response`) 등록 |
| `_on_nav2_goal_response(future)` | 전송 실패/거부 시 로그만 남기고 종료. 수락되면 `goal_handle` 저장하고 결과 콜백(`_on_nav2_result`) 등록 |
| `_on_nav2_result(future)` | 목표 완료 시 상태(`SUCCEEDED`/`CANCELED`/`ABORTED`/기타)를 로그로 출력 |
| `_cancel_nav2_goal_if_active()` | 진행 중인 Nav2 목표가 있으면 비동기 취소 요청 후 goal handle/pending/last-goal 상태를 모두 초기화 |

### 종료/진입점

| 함수 | 기능 |
|---|---|
| `destroy_node` | 정지 Twist를 3번 발행 후 (show_window면 창 정리) 노드 정리 — 종료 시 로봇이 계속 움직이는 것 방지 |
| `main` | rclpy 초기화 → 노드 생성 → **`MultiThreadedExecutor`**로 spin (콜백그룹이 여러 개라 병렬 실행 가능) → 종료 처리 |

---

## 5. `on_oakd_synced` 상세 (핵심 비전 처리)

`message_filters.ApproximateTimeSynchronizer`가 RGB와 뎁스를 시간 맞춰 동시에 넘겨줄 때 호출됨.

1. RGB 압축 이미지를 `bgr8` OpenCV 이미지로 디코딩
2. 뎁스는 `decode_compressed_depth`로 디코딩. 실패하면 경고만 찍고 계속 진행
3. 뎁스 디코딩 성공 시, **최초 1회만** dtype/min/max/nonzero_min을 로그로 출력해서 `depth_scale` 파라미터가 실제 데이터 단위와 맞는지 검증할 수 있게 함
4. `own_cam_model.track()`으로 YOLO+ByteTrack 추론
5. `select_target_box`로 최종 타겟 박스 선택
   - 있으면: bbox/신뢰도/시각 캐싱, bbox 중심 x좌표로 좌/우 부호(`last_known_bearing_sign`) 갱신, 신뢰도가 `own_cam_confirm_conf` 이상이면 `own_cam_confirm_count` 증가(최대 `own_cam_confirm_frames`), 아니면 리셋
   - 없으면: 락 해제, 카운트 리셋
6. `publish_debug_image=True`면 오버레이 이미지를 만들어 발행(+옵션으로 로컬 창 표시)

---

## 6. 상태 전환(핸드오프) 조건 요약

```
PHASE1_APPROACH
  ├─ 하위 모드 (매 틱 재평가):
  │    ├─ 웹캠 타겟 fresh          → NAVIGATING_TO_TARGET: Nav2에 (x,y) 목표 전송, explore resume=False
  │    ├─ 타겟 stale + explore=on  → EXPLORING: Nav2 목표 취소, explore resume=True
  │    └─ 타겟 stale + explore=off → IDLE: Nav2 목표 취소, explore resume=False
  │
  └─ own_cam_confirm_count >= own_cam_confirm_frames  (연속 고신뢰도 탐지)
       AND (own_cam_handoff_max_depth_m <= 0  OR  뎁스로 잰 거리 <= own_cam_handoff_max_depth_m)
       → 진행 중인 Nav2 목표 취소
       ─────────────────────────────────────────────► PHASE2_FOLLOW

PHASE2_FOLLOW
  └─ detection_loss_timeout 동안 자체 카메라 탐지 없음
       ─────────────────────────────────────────────► 탐색(제자리 회전) 모드
       (search_timeout_sec 초과 시 완전 정지 후 재탐지 대기, 상태는 PHASE2 유지)
```

※ PHASE2 → PHASE1로 되돌아가는 로직은 없음 (한 번 핸드오프되면 계속 자체 카메라로만 추적).
※ PHASE1의 실제 주행(경로 계획, 장애물 회피)은 이 노드가 아니라 Nav2(+로컬/글로벌 costmap)가 담당한다 —
  이 노드는 목표 좌표를 넘기고 취소하는 역할만 한다. `nav2_stack.launch.py`가 SLAM+Nav2+`explore_lite`를
  함께 띄운다.

---

## 7. 이 노드가 사용하는 `vision_utils.py` 함수

| 함수 | 용도 (이 노드에서) |
|---|---|
| `decode_compressed_depth(data)` | `compressedDepth` 토픽의 raw payload(12바이트 헤더 + PNG)를 16UC1 뎁스 이미지로 디코딩 |
| `sample_depth_patch(depth_img, u, v, patch_size, depth_scale)` | 바운딩박스 중심 주변 패치의 중앙값 뎁스를 미터로 변환 (노이즈에 강하도록 중앙값 사용) |
| `select_target_box(boxes, target_cls_id, locked_track_id)` | 타겟 클래스 박스들 중 고정 트랙ID 우선, 없으면 최고 신뢰도 박스 선택 |

자세한 내용은 [`vision_utils.md`](vision_utils.md) 참고.

---

## 8. 안전장치 요약

- **`enable_cmd_vel=False`가 기본값** → 실수로 로봇이 움직이는 것 방지 (dry-run 로그만, Nav2 목표도 실제 전송 대신 로그만, explore resume도 항상 False로 강제)
- **`enable_autonomous_exploration=False`가 기본값** → 웹캠 타겟이 없어도 로봇이 알아서 돌아다니지 않음
- 모든 센서(웹캠/뎁스/라이다)에 **stale timeout** 체크 → 오래된 데이터로 잘못된 제어 방지
- PHASE2에서 라이다 기반 전방 장애물 감지 시 **전진만 차단** (후진/회전은 허용); PHASE1은 Nav2 costmap이 장애물 회피 담당
- 종료 시 정지 명령 3회 발행
- 뎁스 핸드오프 거리 체크(`own_cam_handoff_max_depth_m`)로 너무 먼 오탐지에 의한 조기 전환 방지
- PHASE2로 핸드오프되는 순간 진행 중이던 Nav2 목표는 즉시 취소됨
