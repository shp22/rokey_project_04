# `launch/rc_car_chase.launch.py` 상세 설명

`ros2 launch rc_car_chase rc_car_chase.launch.py`로 실행하는 **진입점 launch 파일**.
`webcam_locator_node`와 `chase_controller_node` 두 노드를 한 번에 띄우고, 주요 파라미터를
launch 인자(`DeclareLaunchArgument`)로 노출해서 커맨드라인에서 바로 오버라이드할 수 있게 해준다.

---

## 1. Launch 인자 (`DeclareLaunchArgument`)

| 인자명 | 기본값 | 전달 대상 | 설명 |
|---|---|---|---|
| `webcam_device` | `/dev/video2` | webcam_locator_node → `device` | 웹캠 장치 경로 |
| `webcam_model_path` | `/home/rokey/rokey_ws/best_v11.pt` | webcam_locator_node → `model_path` | 웹캠용 YOLO 가중치 |
| `homography_yaml_path` | `.../config/webcam_homography.yaml` | webcam_locator_node → `homography_yaml_path` | 캘리브레이션 결과 파일 |
| `own_cam_model_path` | `.../car_dum_yolo11n_seqsplit/weights/best.pt` | chase_controller_node → `own_cam_model_path` | 자체 카메라용 YOLO 가중치 |
| `target_distance` | `0.35` | chase_controller_node | PHASE2 목표 거리 |
| `own_cam_confirm_frames` | `4` | chase_controller_node | 핸드오프에 필요한 연속 확인 프레임 수 |
| `detection_loss_timeout` | `1.5` | chase_controller_node | 탐지 유실 판정 시간 |
| `phase1_max_lin` / `phase1_max_ang` | `0.15` / `0.6` | chase_controller_node | PHASE1 최대 속도 |
| `phase2_max_lin` / `phase2_max_ang` | `0.15` / `0.6` | chase_controller_node | PHASE2 최대 속도 |
| `lidar_safety_stop_distance` | `0.3` | chase_controller_node | 라이다 안전 정지 거리 |
| `enable_cmd_vel` | `false` | chase_controller_node | **실제 발행 여부 (SAFE DEFAULT 주석대로 기본은 항상 false)** |
| `initial_state` | `PHASE1_APPROACH` | chase_controller_node | 시작 상태 |
| `webcam_show_window` | `true` | webcam_locator_node → `show_window` | 웹캠 디버그 창 표시 |
| `controller_show_window` | `false` | chase_controller_node → `show_window` | 컨트롤러 디버그 창 표시 |

> 위 표에 없는 나머지 파라미터(예: `fx`, `cx`, `phase1_kp_ang`, `depth_scale` 등)는 launch 인자로
> 노출되어 있지 않음 → 필요하면 `chase_controller_node.py`의 `_declare_parameters` 기본값을
> 직접 고치거나, 별도 파라미터 YAML 파일을 만들어 launch에 추가해야 함.

---

## 2. 띄우는 노드 2개

### `webcam_node`
```python
Node(package='rc_car_chase', executable='webcam_locator_node', name='webcam_locator_node', ...)
```
- 파라미터: `device`, `model_path`, `homography_yaml_path`, `show_window`만 launch 인자로 매핑
- (나머지 `webcam_locator_node`의 파라미터는 스크립트 기본값 그대로 사용됨: `conf_threshold=0.5`, `target_topic=/rc_car_chase/webcam_target` 등)

### `controller_node`
```python
Node(package='rc_car_chase', executable='chase_controller_node', name='chase_controller_node', ...)
```
- 위 표의 파라미터들을 launch 인자로 매핑, 나머지는 노드 기본값 사용

두 노드 모두 `output='screen'` → 터미널에 로그 바로 출력.

---

## 3. 실행 예시

```bash
# 기본 (안전, cmd_vel 발행 안 함 = dry-run)
ros2 launch rc_car_chase rc_car_chase.launch.py

# 실제로 로봇을 움직이려면
ros2 launch rc_car_chase rc_car_chase.launch.py enable_cmd_vel:=true

# 웹캠 장치가 다른 경우
ros2 launch rc_car_chase rc_car_chase.launch.py webcam_device:=/dev/video0 enable_cmd_vel:=true
```

---

## 4. 전체 실행 파이프라인 (파일 간 관계)

```
[사전 준비]
calibrate_webcam_homography.py  ──► config/webcam_homography.yaml 생성
   (※ 현재 config/ 폴더가 비어 있음 — 아직 캘리브레이션 미실행 상태.
      이 파일 없으면 webcam_locator_node가 RuntimeError로 즉시 종료됨)

[실행: ros2 launch rc_car_chase rc_car_chase.launch.py]

 ┌─────────────────────────┐        ┌───────────────────────────────┐
 │   webcam_locator_node    │        │      chase_controller_node      │
 │ (이 컴퓨터에 연결된 웹캠) │        │                                 │
 │                          │        │  구독:                         │
 │ YOLO(웹캠 모델) 탐지/추적 │        │   - webcam_target (PHASE1)     │
 │ + homography 변환        │──────► │   - odom (PHASE1)               │
 │                          │ /rc_   │   - oakd rgb+depth (PHASE2)     │
 │ 발행:                    │ car_   │   - scan (라이다 안전)          │
 │  /rc_car_chase/          │ chase/ │                                 │
 │   webcam_target          │ webcam_│  발행:                         │
 │   webcam_debug_image     │ target │   - /robot5/cmd_vel             │
 └─────────────────────────┘        │   - /rc_car_chase/debug_image   │
                                     └───────────────────────────────┘
                                              ▲
                                              │ (ROS 토픽 구독, 로봇이 발행)
                                     로봇(TurtleBot4/OAK-D/라이다/오도메트리)
```

- `webcam_locator_node`와 `chase_controller_node`는 **`vision_utils.py`의 함수들을 공유**하며 (자세한 내용은 [`vision_utils.md`](vision_utils.md)), 서로는 `/rc_car_chase/webcam_target` 토픽 하나로만 연결됨
- 로봇 자체(OAK-D 카메라, 라이다, 오도메트리, `cmd_vel` 구독자)는 이 패키지 밖에서 이미 실행 중이어야 함 (TurtleBot4 기본 드라이버 등)
- `chase_controller_node`에 대한 상세 설명은 [`chase_controller_node.md`](chase_controller_node.md), `webcam_locator_node`는 [`webcam_locator_node.md`](webcam_locator_node.md) 참고

---

## 5. 체크리스트 (실행 전 확인)

- [ ] `config/webcam_homography.yaml` 존재하는가? (없으면 `calibrate_webcam_homography` 먼저 실행)
- [ ] `webcam_device`가 실제 연결된 웹캠 경로와 일치하는가 (`ls /dev/video*`)
- [ ] `own_cam_model_path`, `webcam_model_path`의 `.pt` 파일이 실제로 존재하는가
- [ ] 로봇 쪽에서 `/robot5/oakd/rgb/image_raw/compressed`, `/robot5/oakd/stereo/image_raw/compressedDepth`, `/robot5/odom`, `/robot5/scan`이 실제로 발행되고 있는가
- [ ] 처음 실행할 때는 `enable_cmd_vel:=false`(기본값)로 dry-run 로그만 확인 후, 안전 확인되면 `true`로 전환
