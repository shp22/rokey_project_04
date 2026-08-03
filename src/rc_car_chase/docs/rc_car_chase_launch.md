# `launch/rc_car_chase.launch.py` 상세 설명

`ros2 launch rc_car_chase rc_car_chase.launch.py`로 실행하는 **진입점 launch 파일**.
`webcam_locator_node`와 `chase_controller_node` 두 노드에 더해, `bringup_nav2_stack:=true`(기본값)면
`nav2_stack.launch.py`(SLAM + Nav2 + `explore_lite`)까지 함께 띄운다. 주요 파라미터를
launch 인자(`DeclareLaunchArgument`)로 노출해서 커맨드라인에서 바로 오버라이드할 수 있게 해준다.

---

## 1. Launch 인자 (`DeclareLaunchArgument`)

| 인자명 | 기본값 | 전달 대상 | 설명 |
|---|---|---|---|
| `webcam_device` | `/dev/video2` | webcam_locator_node → `device` | 웹캠 장치 경로 |
| `webcam_model_path` | `/home/rokey/rokey_ws/best_v11.pt` | webcam_locator_node → `model_path` | 웹캠용 YOLO 가중치 |
| `homography_yaml_path` | `.../config/webcam_homography.yaml` | webcam_locator_node → `homography_yaml_path` | 캘리브레이션 결과 파일 (웹캠 픽셀 → 지면좌표 변환용, PHASE1 자체는 Nav2가 주행하지만 타겟 위치 추정에는 여전히 이 호모그래피를 사용) |
| `own_cam_model_path` | `.../car_dum_yolo11n_seqsplit/weights/best.pt` | chase_controller_node → `own_cam_model_path` | 자체 카메라용 YOLO 가중치 |
| `target_distance` | `0.5` | chase_controller_node | PHASE2(자체 카메라) 목표 추종 거리 |
| `own_cam_handoff_max_depth_m` | `0.7` | chase_controller_node | PHASE1→PHASE2 전환 조건: 자체 카메라 기준 이 거리 이내로 들어와야 전환 (전환 시 RViz에 이벤트 마커도 발행됨) |
| `own_cam_confirm_frames` | `4` | chase_controller_node | 핸드오프에 필요한 연속 확인 프레임 수 |
| `detection_loss_timeout` | `1.5` | chase_controller_node | 탐지 유실 판정 시간 |
| `phase2_max_lin` / `phase2_max_ang` | `0.15` / `0.6` | chase_controller_node | PHASE2 최대 속도 |
| `lidar_safety_stop_distance` | `0.3` | chase_controller_node | 라이다 안전 정지 거리 (PHASE2에만 적용, PHASE1은 Nav2 costmap이 담당) |
| `enable_cmd_vel` | `false` | chase_controller_node | **실제 발행 여부 (SAFE DEFAULT 주석대로 기본은 항상 false)**. Nav2 목표 전송 및 explore resume도 함께 억제됨 |
| `initial_state` | `PHASE1_APPROACH` | chase_controller_node | 시작 상태 |
| `webcam_show_window` | `true` | webcam_locator_node → `show_window` | 웹캠 디버그 창 표시 |
| `controller_show_window` | `false` | chase_controller_node → `show_window` | 컨트롤러 디버그 창 표시 |
| `bringup_webcam` | `true` | `webcam_node` 생성 조건 | `webcam_locator_node`를 이 launch에서 함께 띄울지. **웹캠이 다른 컴퓨터에 연결되어 있고 그쪽에서 이미 `webcam_locator_node`를 띄운 상태라면 `false`** — `/rc_car_chase/webcam_target`은 어느 호스트가 발행했든 같은 `ROS_DOMAIN_ID`면 DDS로 그대로 보임 |
| `bringup_nav2_stack` | `true` | `nav2_stack.launch.py` include 조건 | SLAM+Nav2+explore_lite를 이 launch에서 함께 띄울지 (이미 다른 곳에서 떠 있으면 `false`) |
| `bringup_rviz` | `true` | `rviz2` 노드 생성 조건 | 시각화(RViz)를 이 launch에서 자동으로 함께 띄울지 |
| `rviz_config` | `.../config/rc_car_chase.rviz` | `rviz2` → `-d` | 맵/라이다/자체카메라 디버그 이미지/핸드오프 이벤트 마커가 미리 구성된 RViz 설정 파일 |
| `nav2_action_name` | `/robot5/navigate_to_pose` | chase_controller_node | Nav2 `NavigateToPose` 액션 서버 이름 |
| `explore_resume_topic` | `/robot5/explore/resume` | chase_controller_node | `explore_lite` 재개/일시정지 제어 토픽 |
| `nav2_goal_update_threshold_m` | `0.5` | chase_controller_node | 이 거리 이상 타겟이 움직여야 Nav2 목표 재전송 (너무 잦으면 이전 goal이 끝나기도 전에 계속 preempt되어 ABORTED가 반복됨) |
| `nav2_goal_frame_id` | `odom` | chase_controller_node | Nav2 목표 좌표 기준 프레임 |
| `enable_autonomous_exploration` | `false` | chase_controller_node | **SAFE DEFAULT.** 웹캠 타겟이 없을 때 `explore_lite`로 자동 탐색할지 |
| `auto_undock` | `true` | chase_controller_node | 웹캠이 타겟을 잡았는데 아직 도킹 상태면 자동으로 `undock` 액션부터 보낼지 |
| `dock_status_topic` | `/robot5/dock_status` | chase_controller_node | 도킹 상태 구독 토픽 |
| `undock_action_name` | `/robot5/undock` | chase_controller_node | `Undock` 액션 서버 이름 |

> 위 표에 없는 나머지 파라미터(예: `fx`, `cx`, `depth_scale` 등)는 launch 인자로
> 노출되어 있지 않음 → 필요하면 `chase_controller_node.py`의 `_declare_parameters` 기본값을
> 직접 고치거나, 별도 파라미터 YAML 파일을 만들어 launch에 추가해야 함.

---

## 2. 띄우는 것들

### `nav2_stack` (조건부, `bringup_nav2_stack:=true`일 때)
```python
IncludeLaunchDescription(.../launch/nav2_stack.launch.py, condition=IfCondition(...))
```
- `turtlebot4_navigation`의 `slam.launch.py` + `nav2.launch.py`, 그리고 `explore_lite`의 `explore` 노드를
  `robot5` 네임스페이스로 띄운다. 자세한 내용은 `nav2_stack.launch.py` 자체 docstring 참고.
- 이미 로봇 쪽에서 SLAM/Nav2가 별도로 실행 중이라면 `bringup_nav2_stack:=false`로 꺼야 중복 기동을 피할 수 있다.

### `webcam_node` (조건부, `bringup_webcam:=true`일 때)
```python
Node(package='rc_car_chase', executable='webcam_locator_node', name='webcam_locator_node', ..., condition=IfCondition(...))
```
- 파라미터: `device`, `model_path`, `homography_yaml_path`, `show_window`만 launch 인자로 매핑
- (나머지 `webcam_locator_node`의 파라미터는 스크립트 기본값 그대로 사용됨: `conf_threshold=0.5`, `target_topic=/rc_car_chase/webcam_target` 등)
- 웹캠이 이 컴퓨터가 아니라 다른 컴퓨터에 연결되어 있고 그쪽에서 이미 `webcam_locator_node`를 띄웠다면
  `bringup_webcam:=false`로 꺼야 한다. 이 경우 이 컴퓨터에서 캘리브레이션(`webcam_homography.yaml`)을
  다시 할 필요도 없다 — 그 파일은 `webcam_locator_node`를 실제로 실행하는 컴퓨터에만 필요하고,
  결과물인 `/rc_car_chase/webcam_target` 토픽만 DDS를 통해 이 컴퓨터에서도 그대로 구독된다
  (같은 `ROS_DOMAIN_ID`, 네트워크로 서로 도달 가능해야 함).

### `controller_node`
```python
Node(package='rc_car_chase', executable='chase_controller_node', name='chase_controller_node', ...)
```
- 위 표의 파라미터들을 launch 인자로 매핑, 나머지는 노드 기본값 사용

### `rviz_node` (조건부, `bringup_rviz:=true`일 때)
```python
Node(package='rviz2', executable='rviz2', name='rc_car_chase_rviz', arguments=['-d', rviz_config],
     remappings=[('/tf', '/robot5/tf'), ('/tf_static', '/robot5/tf_static')], condition=IfCondition(...))
```
- `/tf`, `/tf_static`을 `/robot5/tf`, `/robot5/tf_static`으로 리매핑 — robot5의 TF가 네임스페이스가 붙은 토픽으로 발행되기 때문에 (프레임 이름 자체는 `map`/`odom`/`base_link`로 그대로임)
- `rc_car_chase.rviz` 설정에 Map, LaserScan, `/rc_car_chase/debug_image`(자체 카메라 탐지 오버레이), `/rc_car_chase/handoff_event`(PHASE1→PHASE2 전환 마커)가 기본 포함되어 있음

네 항목 모두 `output='screen'` → 터미널에 로그 바로 출력.

---

## 3. 실행 예시

```bash
# 기본 (안전, cmd_vel/Nav2 목표/explore 모두 발행 안 함 = dry-run, Nav2 스택도 같이 기동)
ros2 launch rc_car_chase rc_car_chase.launch.py

# 실제로 로봇을 움직이려면
ros2 launch rc_car_chase rc_car_chase.launch.py enable_cmd_vel:=true

# 타겟을 놓쳤을 때 explore_lite로 자동 탐색까지 원하면
ros2 launch rc_car_chase rc_car_chase.launch.py enable_cmd_vel:=true enable_autonomous_exploration:=true

# Nav2/SLAM이 이미 다른 곳에서 떠 있다면 (중복 기동 방지)
ros2 launch rc_car_chase rc_car_chase.launch.py bringup_nav2_stack:=false enable_cmd_vel:=true

# 웹캠 장치가 다른 경우
ros2 launch rc_car_chase rc_car_chase.launch.py webcam_device:=/dev/video0 enable_cmd_vel:=true

# Nav2도, 웹캠도 다른 컴퓨터에서 이미 떠 있는 경우 - 이 컴퓨터는 chase_controller_node만 필요
ros2 launch rc_car_chase rc_car_chase.launch.py bringup_nav2_stack:=false bringup_webcam:=false enable_cmd_vel:=true

# RViz 자동 실행이 필요 없는 경우 (예: 원격 SSH라 GUI 창을 못 띄울 때)
ros2 launch rc_car_chase rc_car_chase.launch.py bringup_rviz:=false
```

---

## 4. 전체 실행 파이프라인 (파일 간 관계)

```
[사전 준비]
calibrate_webcam_homography.py  ──► config/webcam_homography.yaml 생성
   (이 파일 없으면 webcam_locator_node가 RuntimeError로 즉시 종료됨)

[실행: ros2 launch rc_car_chase rc_car_chase.launch.py]

 ┌──────────────────────────┐        ┌─────────────────────────────────┐
 │   webcam_locator_node    │        │      chase_controller_node       │
 │ (이 컴퓨터에 연결된 웹캠)  │        │                                   │
 │                           │        │  구독:                          │
 │ YOLO(웹캠 모델) 탐지/추적  │        │   - webcam_target (PHASE1)      │
 │ + homography 변환         │──────► │   - oakd rgb+depth (PHASE2)     │
 │                           │ /rc_   │   - scan (PHASE2 라이다 안전)   │
 │ 발행:                     │ car_   │                                 │
 │  /rc_car_chase/           │ chase/ │  발행/호출:                    │
 │   webcam_target           │ webcam_│   - /robot5/cmd_vel (PHASE2)   │
 │   webcam_debug_image      │ target │   - /robot5/navigate_to_pose   │
 └──────────────────────────┘        │     액션 (PHASE1, Nav2로)      │
                                      │   - explore/resume (탐색 on/off)│
                                      │   - /rc_car_chase/debug_image  │
                                      └─────────────────────────────────┘
                                               │              ▲
                                    Nav2 goal  │              │ (오도메트리/스캔/카메라, 로봇이 발행)
                                               ▼              │
                                   ┌─────────────────────────────────┐
                                   │  nav2_stack.launch.py           │
                                   │  (SLAM + Nav2 + explore_lite)   │
                                   └─────────────────────────────────┘
                                               ▲
                                               │
                                     로봇(TurtleBot4/OAK-D/라이다/오도메트리)
```

- `webcam_locator_node`와 `chase_controller_node`는 **`vision_utils.py`의 함수들을 공유**하며 (자세한 내용은 [`vision_utils.md`](vision_utils.md)), 서로는 `/rc_car_chase/webcam_target` 토픽 하나로만 연결됨
- PHASE1의 실제 주행/장애물 회피는 `chase_controller_node`가 아니라 **Nav2**(+`explore_lite`)가 담당한다 — 컨트롤러는 목표 좌표를 보내거나 탐색을 켜고 끄는 역할만 함
- 로봇 자체(OAK-D 카메라, 라이다, `cmd_vel` 구독자)는 이 패키지 밖에서 이미 실행 중이어야 함 (TurtleBot4 기본 드라이버 등)
- `chase_controller_node`에 대한 상세 설명은 [`chase_controller_node.md`](chase_controller_node.md), `webcam_locator_node`는 [`webcam_locator_node.md`](webcam_locator_node.md) 참고

---

## 5. 체크리스트 (실행 전 확인)

- [ ] `config/webcam_homography.yaml` 존재하는가? (없으면 `calibrate_webcam_homography` 먼저 실행)
- [ ] `webcam_device`가 실제 연결된 웹캠 경로와 일치하는가 (`ls /dev/video*`)
- [ ] `own_cam_model_path`, `webcam_model_path`의 `.pt` 파일이 실제로 존재하는가
- [ ] `turtlebot4_navigation`, `explore_lite` 패키지가 워크스페이스(또는 turtlebot4_ws)에 소싱되어 있는가 (`bringup_nav2_stack:=true`로 쓸 경우 필수)
- [ ] 로봇 쪽에서 `/robot5/oakd/rgb/image_raw/compressed`, `/robot5/oakd/stereo/image_raw/compressedDepth`, `/robot5/scan`이 실제로 발행되고 있는가
- [ ] 웹캠 타겟 좌표(`nav2_goal_frame_id`, 기본 `odom`)와 `webcam_locator_node`가 발행하는 `PointStamped.header.frame_id`가 일치하는가
- [ ] 처음 실행할 때는 `enable_cmd_vel:=false`(기본값)로 dry-run 로그만 확인 후, 안전 확인되면 `true`로 전환
- [ ] `enable_autonomous_exploration`은 필요할 때만 `true`로 (기본은 자동 탐색 없이 대기)
