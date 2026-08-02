# RC카를 지도 보고 장애물 피하면서 쫓아가기 — 아주 쉬운 설명서

이 문서는 어려운 말을 최대한 안 쓰고, 그림을 그리듯이 설명합니다.
전문 용어가 필요하면 옆에 (쉬운 말)을 붙였습니다.

---

## 1. 우리가 하려는 것 (비유로 설명)

숨바꼭질을 한다고 생각해봐요.

- **웹캠**: 천장에 매달린 CCTV처럼, 방 전체를 내려다보는 "눈"이에요. RC카가 어디 있는지 계속 지켜봐요.
- **지도(map)**: 우리 집을 위에서 내려다본 그림이에요. 어디가 벽이고 어디가 빈 바닥인지 그려져 있어요. (전에 SLAM으로 만든 `maps/turtle5_map.yaml`이 바로 이 그림이에요)
- **Nav2**: "길 찾기 선생님"이에요. "여기서 저기까지 가려면 어떻게 가야 벽에 안 부딪힐까?"를 계산해주는 똑똑한 프로그램이에요.
- **터틀봇(로봇)**: 심부름꾼이에요. 길 찾기 선생님이 알려준 길을 그대로 따라가요.
- **라이다(레이저 센서)**: 로봇의 지팡이예요. 앞에 뭔가 있으면 부딪히기 전에 알아채요.

**전체 흐름**: 웹캠이 "RC카 저기 있어!"라고 소리치면 → 그 위치를 지도 위의 좌표로 바꿔서 → 길 찾기 선생님(Nav2)한테 "저기로 가주세요" 부탁하고 → 길 찾기 선생님은 지도랑 라이다를 보면서 벽을 피해 길을 만들어주고 → 로봇이 그 길을 따라 움직여요.

RC카가 아주 가까워지면? 이제 지도 볼 필요 없이 로봇 자기 눈(카메라)으로 직접 "코 앞의 목표"만 보고 바짝 따라가요 (이건 원래 있던 기능이에요, 그대로 씁니다).

---

## 2. 두 단계로 나눠서 움직여요

| 단계 | 언제 | 무엇으로 위치를 앎 | 누가 운전하나 |
|---|---|---|---|
| **PHASE1 (멀리 접근)** | RC카가 로봇 자기 카메라에는 아직 잘 안 보일 때 | 천장 웹캠 | **Nav2** (지도+라이다 보고 장애물 피해서 운전) |
| **PHASE2 (바짝 추적)** | 로봇 자기 카메라(OAK-D)로 RC카가 확실히 보일 때 | 로봇 자기 카메라 | **기존 chase_controller_node** (바로 앞의 것만 보고 딱 붙어 따라감) |

두 단계가 자동으로 바뀌어요 (로봇이 "어? 이제 내 카메라로도 잘 보이네!" 하면 자동으로 PHASE2로 넘어가요). 사람이 직접 안 눌러도 돼요.

---

## 3. 새로 만든/고친 파일들

### 3-1. 새로 만든 파일

| 파일 | 한 줄 설명 (아주 쉽게) |
|---|---|
| `rc_car_chase/nav_goal_bridge_node.py` | 웹캠이 알려준 RC카 위치를 받아서, "Nav2야, 여기로 가줘!"라고 계속 말해주는 **다리(전달자) 역할** 노드 |
| `launch/rc_car_chase_nav2.launch.py` | 필요한 모든 프로그램(웹캠, 컨트롤러, 다리, 지도+길찾기)을 **한 번에 다 켜주는 시작 버튼** |
| `config/localization_robot5.yaml` | "내가 지도 위 어디에 있지?"를 계산하는 프로그램(AMCL)의 설정값 모음 |
| `config/nav2_robot5.yaml` | "길을 어떻게 찾을지, 얼마나 빨리 갈지, 장애물을 얼마나 무서워할지"를 정하는 설정값 모음 |

### 3-2. 고친 파일

| 파일 | 무엇을 고쳤나 (쉽게) |
|---|---|
| `webcam_locator_node.py` | RC카 위치를 알려줄 때 "이건 odom이라는 기준점에서 잰 거야"라고 이름표를 붙이는데, 이 이름표를 바꿔 붙일 수 있게 만들었어요 (`odom_frame_id` 옵션 추가) |
| `chase_controller_node.py` | ① 지금이 PHASE1인지 PHASE2인지 다른 프로그램도 알 수 있게 방송(publish)하기 시작했어요. ② `use_nav2_phase1`이라는 스위치를 켜면, PHASE1일 때 이 노드는 운전을 안 하고 가만히 있어요 (Nav2한테 운전대를 넘겨준 거예요) |

---

## 4. 새 파일 안의 "변수"들이 뭘 뜻하는지

### `nav_goal_bridge_node.py`의 변수 (파라미터)

이 노드는 "웹캠이 본 위치 → Nav2한테 갈 곳 알려주기"를 하는 다리예요. 아래 값들로 동작을 조절해요.

| 변수 이름 | 기본값 | 쉬운 설명 |
|---|---|---|
| `webcam_target_topic` | `/rc_car_chase/webcam_target` | 웹캠이 "RC카 여기 있어요!" 하고 외치는 방송 채널 이름 |
| `phase_topic` | `/rc_car_chase/phase` | chase_controller_node가 "지금 PHASE1이야/PHASE2야" 하고 외치는 채널 이름 |
| `nav_action_name` | `/robot5/navigate_to_pose` | Nav2한테 "여기로 가주세요!"라고 부탁하는 전화번호 같은 것 |
| `map_frame` | `map` | 지도의 이름표 (모든 위치를 이 지도 기준으로 통일해서 말해야 해요) |
| `robot_base_frame` | `robot5/base_link` | 로봇 몸통의 이름표 (로봇이 지금 지도 위 어디 있는지 알아낼 때 씀) |
| `target_stale_timeout` | `1.0`초 | 웹캠 소식이 이 시간보다 오래 안 오면 "이제 못 믿겠다" 하고 무시함 |
| `phase_stale_timeout` | `2.0`초 | chase_controller의 "지금 몇 단계야" 소식이 이 시간보다 오래 안 오면 안전하게 아무것도 안 함 |
| `goal_update_dist_m` | `0.35`m | RC카가 이만큼(35cm)은 움직여야 "목표 위치 다시 알려줄게" 하고 Nav2한테 새로 부탁함 (너무 자주 부탁하면 로봇이 갈팡질팡하니까) |
| `goal_min_resend_interval` | `1.5`초 | 아무리 많이 움직여도 최소 이 시간(1.5초)은 기다렸다가 다시 부탁함 |
| `update_rate_hz` | `2.0` | 1초에 몇 번 "지금 목표 다시 정해야 하나?" 확인하는지 |
| `enable_nav` | `False` (꺼짐) | **제일 중요한 안전 스위치!** `False`면 "이렇게 부탁했을 거예요"라고 화면에 글자만 찍고 진짜로는 Nav2한테 안 보내요 (연습 모드). `True`로 바꿔야 진짜로 로봇이 움직여요 |

### `chase_controller_node.py`에 새로 생긴 변수

| 변수 이름 | 기본값 | 쉬운 설명 |
|---|---|---|
| `use_nav2_phase1` | `False` | `True`로 켜면: PHASE1일 때 이 노드가 직접 운전하는 걸 멈추고, Nav2(=nav_goal_bridge_node)한테 운전대를 넘겨줌 |
| `phase_topic` | `/rc_car_chase/phase` | "나 지금 PHASE1/PHASE2야" 하고 알려주는 방송 채널 이름 |

### `webcam_locator_node.py`에 새로 생긴 변수

| 변수 이름 | 기본값 | 쉬운 설명 |
|---|---|---|
| `odom_frame_id` | `odom` | RC카 위치를 알려줄 때 붙이는 기준점 이름표. 로봇이 `robot5`라는 이름을 쓰고 있으면 `robot5/odom`으로 맞춰줘야 해요 (실행 파일에서는 이미 `robot5/odom`으로 맞춰놨어요) |

---

## 5. 실행 방법 (순서대로 그대로 따라하기)

### 준비물 체크 (한 번만 확인하면 됨)

- [ ] `maps/turtle5_map.yaml`, `maps/turtle5_map.pgm` — 지도 파일 (이미 있음)
- [ ] `src/rc_car_chase/config/webcam_homography.yaml` — 웹캠 좌표 변환표 (없으면 `calibrate_webcam_homography` 먼저 실행해서 만들어야 해요)
- [ ] 로봇(터틀봇4)이 켜져 있고 `/robot5/odom`, `/robot5/scan`, `/robot5/oakd/...` 이 실제로 나오고 있는지
- [ ] 웹캠이 컴퓨터에 꽂혀 있는지 (`ls /dev/video*`)

### 1단계: 빌드하기 (프로그램 새로 짓기)

```bash
cd /home/rokey/b_4_map
source /opt/ros/humble/setup.bash
source /home/rokey/turtlebot4_ws/install/setup.bash
colcon build --packages-select rc_car_chase
source install/setup.bash
```

### 2단계: 아주 안전한 "연습 모드"로 먼저 켜보기

아무것도 실제로 안 움직이고, 화면에 글자만 찍히는 모드예요. **꼭 이걸 먼저 해보세요.**

```bash
ros2 launch rc_car_chase rc_car_chase_nav2.launch.py
```

- `enable_cmd_vel:=false` (기본값) → PHASE2에서도 로봇 안 움직임
- `enable_nav:=false` (기본값) → PHASE1에서도 Nav2한테 진짜 목표를 안 보냄, 화면에 "이렇게 보낼 거예요"만 찍음

화면에 이상한 에러(빨간 글씨)가 안 뜨고, `[DRY RUN]`이나 `nav2 owns cmd_vel` 같은 글자가 잘 찍히면 성공이에요.

### 3단계: 위치를 잘 알고 있는지 확인 (RViz로 눈으로 보기)

```bash
rviz2
```

RViz에서 지도가 잘 보이고, 로봇 모양이 지도 위 실제 있는 자리에 그려지는지 확인하세요. 만약 로봇이 엉뚱한 곳에 있다면, RViz의 "2D Pose Estimate" 버튼으로 로봇의 실제 위치를 한 번 콕 찍어서 알려주세요 (AMCL이라는 프로그램이 위치를 처음엔 잘 몰라서, 사람이 한 번 힌트를 줘야 해요).

### 4단계: 진짜로 로봇을 움직이기

연습 모드에서 로그가 이상하지 않고, RViz에서 로봇 위치도 잘 맞으면, 이제 진짜로 켭니다.

```bash
ros2 launch rc_car_chase rc_car_chase_nav2.launch.py enable_nav:=true enable_cmd_vel:=true
```

- `enable_nav:=true` → PHASE1(멀리 있을 때)에 진짜로 Nav2한테 "여기로 가주세요" 부탁함
- `enable_cmd_vel:=true` → PHASE2(가까이 있을 때)에 진짜로 로봇이 움직임

### 자주 바꾸는 옵션들

```bash
# 웹캠 장치가 다른 경우
ros2 launch rc_car_chase rc_car_chase_nav2.launch.py webcam_device:=/dev/video0

# 목표를 너무 자주/드물게 바꾸는 게 이상하면
ros2 launch rc_car_chase rc_car_chase_nav2.launch.py goal_update_dist_m:=0.5 goal_min_resend_interval:=2.0

# 다른 지도를 쓰고 싶으면
ros2 launch rc_car_chase rc_car_chase_nav2.launch.py map:=/경로/다른지도.yaml
```

---

## 6. 꼭 알아야 할 주의사항

1. **`robot5/base_link`, `robot5/odom` 이름이 실제 로봇과 다를 수 있어요.**
   이 프로젝트는 로봇 이름을 전부 `robot5`로 맞춰서 만들었어요 (`/robot5/cmd_vel`, `/robot5/odom` 등 기존 코드와 통일). 만약 실제 로봇의 좌표 이름표(tf frame)가 다르면(`ros2 run tf2_tools view_frames`로 확인 가능), `config/localization_robot5.yaml`과 `config/nav2_robot5.yaml` 안의 `robot5/base_link`, `robot5/odom` 부분을 실제 이름으로 바꿔줘야 해요.

2. **처음엔 항상 연습 모드(`enable_nav:=false enable_cmd_vel:=false`)로 확인하세요.**
   갑자기 로봇이 엉뚱하게 움직이는 걸 막기 위한 안전장치예요.

3. **속도를 일부러 느리게 맞춰놨어요** (최고 속도 0.18m/s 정도). 실제로 잘 되는 걸 확인한 다음에 `config/nav2_robot5.yaml`의 `max_vel_x` 같은 값을 올려도 돼요.

4. **PHASE2 → PHASE1로 다시 못 돌아가요** (원래 코드부터 그랬어요). 로봇 자기 카메라로 한 번 확실히 봤으면 계속 그걸로만 따라가요.
