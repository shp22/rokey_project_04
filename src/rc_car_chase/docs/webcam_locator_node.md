# `webcam_locator_node.py` 상세 설명

외부(고정) 웹캠 영상에서 YOLO로 RC카(타겟)를 탐지·추적하고, 픽셀 좌표를 호모그래피 변환으로
**지면 기준 월드 좌표(x, y)**로 바꿔 `/rc_car_chase/webcam_target`에 발행하는 노드.
`chase_controller_node`의 PHASE1_APPROACH(장거리 접근) 단계가 이 값을 구독해서 사용한다.

---

## 1. 클래스 개요

```
WebcamLocatorNode(Node)
├── __init__           : 파라미터 로드, 호모그래피 로드, YOLO 모델 로드, 카메라 오픈, 퍼블리셔/타이머 생성
├── on_timer            : 매 프레임 캡처 → YOLO 추적 → 좌표 변환 → 퍼블리시 (핵심 루프)
├── destroy_node        : 카메라 릴리즈, 윈도우 정리
└── main                : rclpy 진입점
```

---

## 2. 파라미터 (`declare_parameter`)

| 파라미터명 | 기본값 | 타입 | 설명 |
|---|---|---|---|
| `device` | `/dev/video2` | str | OpenCV로 열 웹캠 장치 경로. **이 노드를 실행하는 컴퓨터에 물리적으로 연결된 장치여야 함** |
| `capture_width` | `1280` | int | 캡처 해상도 가로 |
| `capture_height` | `720` | int | 캡처 해상도 세로 |
| `capture_fps` | `30.0` | float | 캡처/처리 주기 (타이머 주파수로 사용) |
| `model_path` | `/home/rokey/rokey_ws/best_v11.pt` | str | 웹캠 전용 YOLO 가중치 파일 경로 |
| `conf_threshold` | `0.5` | float | YOLO 탐지 신뢰도 임계값 |
| `tracker` | `bytetrack.yaml` | str | Ultralytics `.track()`에 사용할 트래커 설정 |
| `target_class_id` | `0` | int | 추적 대상으로 볼 클래스 ID |
| `homography_yaml_path` | `.../config/webcam_homography.yaml` | str | `calibrate_webcam_homography.py`로 미리 계산해둔 3x3 호모그래피 행렬 파일 경로 |
| `target_topic` | `/rc_car_chase/webcam_target` | str | 타겟 월드 좌표를 발행할 토픽 |
| `overlay_topic` | `/rc_car_chase/webcam_debug_image` | str | 탐지 결과를 그린 디버그 이미지를 발행할 토픽 |
| `publish_overlay` | `True` | bool | 디버그 오버레이 이미지 발행 여부 |
| `show_window` | `True` | bool | `cv2.imshow`로 로컬 화면에 창을 띄울지 여부 |

---

## 3. 인스턴스 변수 (파라미터 로드 이후 생성되는 상태)

| 변수명 | 타입 | 기능 |
|---|---|---|
| `self.conf_threshold` / `self.tracker_cfg` / `self.target_class_id` | - | 파라미터 값을 멤버로 캐싱 (매 프레임 재조회 방지) |
| `self.publish_overlay` / `self.show_window` | bool | 위와 동일한 이유로 캐싱 |
| `self.homography` | `np.ndarray (3x3)` | `load_homography_yaml`로 로드한 픽셀→지면좌표 변환 행렬. 로드 실패 시 노드 생성 자체가 `RuntimeError`로 실패 (먼저 캘리브레이션 필요하다는 안내 메시지 포함) |
| `self.model` | `ultralytics.YOLO` | 웹캠 전용 탐지/추적 모델 인스턴스 |
| `self.bridge` | `CvBridge` | OpenCV ↔ ROS `Image` 메시지 변환기 |
| `self.cap` | `cv2.VideoCapture` | 실제 웹캠 캡처 핸들. 열기 실패 시 `RuntimeError` |
| `self.target_pub` | `Publisher[PointStamped]` | 타겟 월드 좌표 발행자 |
| `self.overlay_pub` | `Publisher[Image] \| None` | 디버그 오버레이 발행자 (`publish_overlay=False`면 `None`) |
| `self.locked_track_id` | `int \| None` | 현재 "고정 추적 중"인 ByteTrack 트랙 ID. 타겟을 놓치면 `None`으로 리셋 |
| `self.frame_count` | int | 처리한 프레임 수 (30프레임마다 로그 출력용) |
| `self.timer` | `Timer` | `1.0 / capture_fps` 주기로 `on_timer` 호출 |

**QoS**: `depth=1`, `RELIABLE`, `KEEP_LAST` — 최신 값 1개만 신뢰성 있게 전달 (오도메트리 기반 접근 제어에 쓰이므로 순간값이 중요, 누적 큐 불필요).

---

## 4. 함수 상세

### `__init__(self)`
1. 파라미터 선언 + 값 읽기
2. 호모그래피 YAML 로드 (`load_homography_yaml`) — 실패 시 즉시 예외 발생시켜 잘못된 좌표로 동작하는 것을 방지
3. YOLO 모델 로드
4. `cv2.VideoCapture(device)`로 카메라 오픈, 해상도 설정, 오픈 실패 체크
5. 퍼블리셔(`target_pub`, `overlay_pub`) 및 타이머 생성

### `on_timer(self)` — 핵심 처리 루프 (매 프레임)
1. `self.cap.read()`로 프레임 캡처. 실패 시 경고 로그만 찍고 리턴 (2초 throttle)
2. 30프레임마다 진행 상황 로그
3. `self.model.track(...)`로 YOLO 탐지 + ByteTrack 추적 수행
4. `select_target_box(boxes, target_class_id, locked_track_id)`로 최종 타겟 박스 선택
   - 타겟이 있으면:
     - `locked_track_id` 갱신 (있으면 계속 같은 트랙 ID를 우선 고정)
     - 바운딩박스에서 `u = 중심 x좌표`, `v = y2(박스 하단)` 사용 → **차량이 지면과 닿는 접지점**을 기준으로 삼음 (박스 중심이 아니라 하단을 쓰는 이유: 호모그래피는 "지면 평면" 변환이라 접지점이라야 정확함)
     - `apply_homography(u, v, homography)`로 픽셀 → 실제 지면 좌표 (x, y) 변환
     - `PointStamped` 메시지 생성 (`frame_id='odom'`, z=0.0) 후 발행
   - 타겟이 없으면 `locked_track_id = None`으로 리셋 (트랙 놓침)
5. `publish_overlay=True`면 `results[0].plot()`으로 오버레이 이미지를 만들어 발행
6. `show_window=True`면 `cv2.imshow`로 로컬 창에 표시

### `destroy_node(self)`
- 카메라 핸들 릴리즈, 필요 시 OpenCV 창 정리 후 부모 클래스 정리 호출 (노드 종료 시 리소스 누수 방지)

### `main(args=None)`
- `rclpy` 초기화 → 노드 생성 → `spin()` → `KeyboardInterrupt` 처리 → 종료 시 `destroy_node` + `rclpy.shutdown()`

---

## 5. 이 노드가 사용하는 `vision_utils.py` 함수

| 함수 | 입력 | 출력 | 기능 |
|---|---|---|---|
| `select_target_box(boxes, target_cls_id, locked_track_id)` | YOLO 결과 박스들, 타겟 클래스 ID, 현재 고정된 트랙 ID | 선택된 박스 또는 `None` | 대상 클래스에 해당하는 박스들 중 **고정 트랙 ID가 아직 존재하면 그것을 우선 선택**, 없으면 신뢰도(conf)가 가장 높은 박스 선택. 후보 자체가 없으면 `None` |
| `apply_homography(u, v, H)` | 픽셀 좌표 (u, v), 3x3 호모그래피 행렬 | (x, y) 지면 좌표 | `cv2.perspectiveTransform`으로 카메라 픽셀 → 실세계 지면 평면 좌표 변환 |
| `load_homography_yaml(path)` | YAML 파일 경로 | `np.ndarray` (3x3) | `calibrate_webcam_homography.py`가 저장해둔 `homography_matrix` 값을 읽어 3x3 행렬로 reshape |

---

## 6. 데이터 흐름 요약

```
웹캠 프레임
   │
   ▼
YOLO track() ──► 여러 박스 (클래스, conf, track id 포함)
   │
   ▼
select_target_box() ──► 최종 타겟 박스 1개 (또는 없음)
   │
   ▼
박스 하단 중심점 (u, v = 접지점)
   │
   ▼
apply_homography() ──► 지면 기준 (x, y) [미터 단위 월드 좌표]
   │
   ▼
PointStamped 발행 → /rc_car_chase/webcam_target
   (chase_controller_node의 PHASE1_APPROACH가 이 값 + odom으로 접근 방향 계산)
```

---

## 7. 참고: 전제 조건

- 이 노드를 돌리기 전에 **`calibrate_webcam_homography` 실행 → `webcam_homography.yaml` 생성**이 선행되어야 함 (호모그래피 로드 실패 시 `RuntimeError`로 명확히 안내됨)
- `device` 파라미터가 가리키는 카메라는 **이 노드를 실행하는 컴퓨터에 실제로 연결**되어 있어야 함 (ROS 토픽 구독이 아니라 `cv2.VideoCapture`로 로컬 장치를 직접 여는 방식이기 때문)
