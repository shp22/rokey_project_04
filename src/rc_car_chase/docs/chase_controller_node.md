# `chase_controller_node.py` 상세 설명

RC카 추적의 **메인 컨트롤러**. 웹캠 타겟 좌표 + 오도메트리로 장거리 접근(PHASE1)하고,
자체 OAK-D 카메라(YOLO+뎁스)로 확인되면 근거리 추적(PHASE2)으로 전환, 라이다로 충돌 방지하며
`cmd_vel`(Twist)을 발행하는 상태 머신 노드.

---

## 1. 모듈 레벨 상수/헬퍼 함수

| 이름 | 기능 |
|---|---|
| `PHASE1_APPROACH` | 상태 문자열 상수 `'PHASE1_APPROACH'` — 웹캠+오도메트리로 대략 접근하는 단계 |
| `PHASE2_FOLLOW` | 상태 문자열 상수 `'PHASE2_FOLLOW'` — 자체 카메라로 정밀 추적하는 단계 |
| `clamp(value, lo, hi)` | 값을 `[lo, hi]` 범위로 제한 (속도/각속도 제한에 반복 사용) |
| `normalize_angle(angle)` | 각도를 `(-π, π]` 범위로 정규화 (heading error 계산 시 필수) |
| `quaternion_to_yaw(q)` | 쿼터니언에서 yaw(z축 회전각)만 추출 (오도메트리의 orientation → 로봇 방향) |

---

## 2. 파라미터 (`_declare_parameters` / `_read_parameters`)

### 2-1. 토픽/모델 경로

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `own_cam_rgb_topic` | `/robot5/oakd/rgb/image_raw/compressed` | 자체 RGB 카메라 압축 이미지 토픽 |
| `own_cam_depth_topic` | `/robot5/oakd/stereo/image_raw/compressedDepth` | 자체 뎁스 이미지 토픽 (압축뎁스) |
| `cmd_vel_topic` | `/robot5/cmd_vel` | 로봇 속도 명령 발행 토픽 |
| `webcam_target_topic` | `/rc_car_chase/webcam_target` | `webcam_locator_node`가 발행하는 타겟 월드좌표 토픽 |
| `odom_topic` | `/robot5/odom` | 로봇 오도메트리 토픽 |
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
| `own_cam_handoff_max_depth_m` | `3.0` | 전환 시점에 뎁스로 잰 거리가 이보다 멀면 전환 보류 (너무 멀리서 오탐 방지, `0` 이하면 이 체크 비활성) |
| `target_class_id` | `0` | 추적 대상 클래스 ID |

### 2-3. 거리/뎁스 제어

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `target_distance` | `0.35` (m) | PHASE2에서 유지하려는 목표 거리 |
| `distance_deadband` | `0.03` (m) | 이 오차 범위 내에서는 전진/후진 안 함 |
| `depth_scale` | `0.001` | 뎁스 raw 값(보통 mm) → 미터 변환 배율 |
| `depth_patch_size` | `5` | 뎁스 샘플링 시 중심점 주변 패치 크기(픽셀) |
| `depth_stale_timeout` | `0.5` (s) | 뎁스 데이터가 이보다 오래되면 무효 처리 |

### 2-4. 타임아웃(신선도 체크)

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `detection_loss_timeout` | `1.5` (s) | 자체 카메라 탐지가 이보다 오래 없으면 "놓침"으로 간주 → 탐색모드 |
| `webcam_stale_timeout` | `1.0` (s) | 웹캠 타겟이 오래되면 PHASE1에서 정지 |
| `odom_stale_timeout` | `1.0` (s) | 오도메트리가 오래되면 PHASE1에서 정지 |
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

### 2-7. PHASE1 제어 게인

| 파라미터명 | 기본값 | 설명 |
|---|---|---|
| `phase1_kp_ang` | `0.8` | heading error → 각속도 비례 게인 |
| `phase1_max_ang` | `0.6` | 최대 각속도 |
| `phase1_forward_speed` | `0.12` | 기본 전진 속도 (heading이 맞을수록 유지) |
| `phase1_max_lin` | `0.15` | 최대 전진 속도 |

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
| `search_ang_speed` | `0.3` | 타겟 놓쳤을 때 회전 탐색 각속도 |
| `search_timeout_sec` | `15.0` | 탐색 포기 시간 |
| `control_rate_hz` | `10.0` | 제어 타이머 주기 |
| `publish_debug_image` | `True` | 디버그 오버레이 발행 여부 |
| `show_window` | `False` | 로컬 창 표시 여부 |
| `enable_cmd_vel` | `False` | **실제 발행 여부 (안전 기본값=dry-run)** |
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
| `self.latest_odom_xy`, `self.latest_odom_yaw`, `self.latest_odom_stamp` | `on_odom` | 최신 로봇 위치/방향/시각 |
| `self.latest_own_cam_bbox`, `self.latest_own_cam_conf`, `self.latest_own_cam_stamp` | `on_oakd_synced` | 자체 카메라 최신 타겟 바운딩박스/신뢰도/시각 |
| `self.latest_depth_image`, `self.latest_depth_stamp` | `on_oakd_synced` | 최신 디코딩된 뎁스 이미지와 시각 |
| `self.latest_lidar_min_front`, `self.latest_lidar_stamp` | `on_lidar` | 전방 최소 거리와 시각 |
| `self.own_cam_confirm_count` | `on_oakd_synced` | 고신뢰도 연속 탐지 카운트 (핸드오프 판단용) |
| `self.locked_track_id` | `on_oakd_synced` | 현재 고정 추적 중인 ByteTrack ID |
| `self.last_known_bearing_sign` | `on_oakd_synced` | 마지막으로 타겟이 화면 좌/우 어디 있었는지 (+1/-1) — 놓쳤을 때 탐색 회전 방향 결정 |
| `self.search_start_time` | `tick_phase2`/`_search_twist` | 탐색 모드 시작 시각 (타임아웃 판단용) |
| `self._depth_debug_printed` | `on_oakd_synced` | 뎁스 sanity-check 로그를 한 번만 찍기 위한 플래그 |
| `self.frame_count` | `on_oakd_synced` | 처리된 동기화 프레임 수 |

---

## 4. 함수 상세

### 초기화

| 함수 | 기능 |
|---|---|
| `__init__` | 파라미터 로드 → YOLO 모델/CvBridge 준비 → 상태 변수 초기화 → QoS 2종 설정(reliable/depth) → 콜백그룹 4개 생성(병렬 처리 분리) → 구독 3개(webcam target, odom, lidar) + RGB/뎁스 동기화 구독 → 퍼블리셔(cmd_vel, debug image) → 제어 타이머 생성 |
| `_declare_parameters` | 위 표의 모든 파라미터를 기본값과 함께 선언 |
| `_read_parameters` | 선언된 파라미터 값을 읽어 `self.*` 멤버 변수로 캐싱 (매 루프마다 파라미터 서버 조회하지 않도록) |

### 콜백 (센서 입력 → 상태 캐싱만 수행, 제어는 안 함)

| 함수 | 콜백 그룹 | 기능 |
|---|---|---|
| `on_webcam_target(msg)` | cb_light | 웹캠 타겟 좌표 캐싱 |
| `on_odom(msg)` | cb_light | 오도메트리 위치 + yaw(쿼터니언 변환) 캐싱 |
| `on_lidar(msg)` | cb_lidar | 스캔 전체 중 **정면 반각(`lidar_front_half_angle`) 범위 + 유효 거리(finite, >0.01m)**만 걸러서 최소값 저장. `lidar_forward_offset_rad`로 정면 기준 보정 |
| `on_oakd_synced(rgb_msg, depth_msg)` | (message_filters 동기화 콜백) | 아래 "5. `on_oakd_synced` 상세" 참고 |

### 제어 루프

| 함수 | 기능 |
|---|---|
| `on_control_timer` | 매 `1/control_rate_hz` 초마다 실행. 현재 state에 따라 `tick_phase1`/`tick_phase2` 호출 → `_apply_lidar_safety` 적용 → `enable_cmd_vel=True`면 실제 발행, 아니면 `[DRY RUN]` 로그만 출력 |
| `tick_phase1(now)` | **PHASE1 로직**: 핸드오프 조건 확인(아래 참고) → 웹캠/오도메트리 신선도 확인(오래됐으면 정지) → 타겟 방향(`atan2(dy,dx)`)과 로봇 yaw의 차이(heading_error)로 각속도 비례제어, `cos(heading_error)`로 전진속도 스케일(방향이 틀어질수록 감속) |
| `tick_phase2(now)` | **PHASE2 로직**: 최근 탐지가 없으면 `_search_twist` 호출 → 있으면 바운딩박스 중심의 bearing angle로 각속도 계산, 뎁스로 거리 오차 계산해 전진/후진(데드밴드, 후진 허용 여부 반영) |
| `_search_twist(now)` | 타겟을 놓쳤을 때: 처음 놓친 시각 기록 → `search_timeout_sec` 초과 시 정지+에러로그 → 아니면 `last_known_bearing_sign` 방향으로 제자리 회전 |
| `_sample_depth_at_bbox_center(now)` | 현재 바운딩박스 중심 픽셀에서 뎁스 패치 샘플링 (`vision_utils.sample_depth_patch` 호출). 뎁스가 stale하거나 없으면 `None` |
| `_apply_lidar_safety(twist, now)` | 라이다 데이터가 신선하고 전방 최소거리가 안전거리 미만이면 전진 속도를 0 이하로 clamp (후진은 허용). 라이다 데이터가 없거나 오래되면 **안전layer를 건너뜀** (막지 않음) |

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
  └─ own_cam_confirm_count >= own_cam_confirm_frames  (연속 고신뢰도 탐지)
       AND (own_cam_handoff_max_depth_m <= 0  OR  뎁스로 잰 거리 <= own_cam_handoff_max_depth_m)
       ─────────────────────────────────────────────► PHASE2_FOLLOW

PHASE2_FOLLOW
  └─ detection_loss_timeout 동안 자체 카메라 탐지 없음
       ─────────────────────────────────────────────► 탐색(제자리 회전) 모드
       (search_timeout_sec 초과 시 완전 정지 후 재탐지 대기, 상태는 PHASE2 유지)
```

※ PHASE2 → PHASE1로 되돌아가는 로직은 없음 (한 번 핸드오프되면 계속 자체 카메라로만 추적).

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

- **`enable_cmd_vel=False`가 기본값** → 실수로 로봇이 움직이는 것 방지 (dry-run 로그만)
- 모든 센서(웹캠/오도메트리/뎁스/라이다)에 **stale timeout** 체크 → 오래된 데이터로 잘못된 제어 방지
- 라이다 기반 전방 장애물 감지 시 **전진만 차단** (후진/회전은 허용)
- 종료 시 정지 명령 3회 발행
- 뎁스 핸드오프 거리 체크(`own_cam_handoff_max_depth_m`)로 너무 먼 오탐지에 의한 조기 전환 방지
