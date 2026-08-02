# `vision_utils.py` 상세 설명

`webcam_locator_node.py`와 `chase_controller_node.py`가 공통으로 사용하는 **순수 함수 모음**
(ROS와 무관, 상태 없음 — 입력만으로 출력이 결정되는 유틸리티 함수들).

| 함수 | 사용하는 노드 |
|---|---|
| `decode_compressed_depth` | chase_controller_node |
| `sample_depth_patch` | chase_controller_node |
| `select_target_box` | chase_controller_node, webcam_locator_node |
| `apply_homography` | webcam_locator_node |
| `load_homography_yaml` | webcam_locator_node |

---

## 1. `decode_compressed_depth(data: bytes)`

**입력**: `sensor_msgs/CompressedImage.data` (compressedDepth 트랜스포트의 raw 바이트)
**출력**: 디코딩된 뎁스 이미지 (`np.ndarray`, 보통 16UC1) 또는 실패 시 `None`

```python
if len(data) <= 12:
    return None
buf = np.frombuffer(data[12:], dtype=np.uint8)
img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
return img
```

| 코드 | 설명 |
|---|---|
| `len(data) <= 12` 체크 | `compressed_depth_image_transport`는 앞에 **12바이트 헤더**(depthQuantA, depthQuantB — float32 2개, quantization 파라미터)를 붙이고 그 뒤에 PNG로 압축된 실제 뎁스 데이터를 담음. 헤더만 있고 실데이터가 없으면 디코딩 불가로 판단 |
| `data[12:]` | 헤더를 건너뛰고 PNG 바이트만 추출 |
| `cv2.imdecode(..., IMREAD_UNCHANGED)` | 8/16비트 그대로(색공간 변환 없이) 디코딩 → 16UC1 뎁스 이미지 복원 |

> 참고: `chase_controller_node`에서 뎁스 토픽으로 `.../compressed`가 아니라 반드시 `.../compressedDepth`를 구독해야 하는 이유가 바로 이 함수 — 일반 `compressed` 트랜스포트는 이 12바이트 뎁스 헤더가 없어서 이 함수로 디코딩할 수 없음.

---

## 2. `sample_depth_patch(depth_img, u, v, patch_size, depth_scale)`

**입력**: 뎁스 이미지, 중심 픽셀 좌표 `(u, v)`, 패치 한 변 크기, 미터 변환 배율
**출력**: 패치 내 유효(0이 아닌) 값들의 **중앙값**(median) × `depth_scale` (미터), 유효값이 없으면 `None`

```python
h, w = depth_img.shape[:2]
half = patch_size // 2
u0, u1 = max(0, u - half), min(w, u + half + 1)
v0, v1 = max(0, v - half), min(h, v + half + 1)
region = depth_img[v0:v1, u0:u1].astype(np.float32)
valid = region[region > 0]
if valid.size == 0:
    return None
return float(np.median(valid)) * depth_scale
```

- 픽셀 하나만 보면 노이즈나 구멍(0값)에 취약하므로, **중심 주변 patch_size×patch_size 영역**을 보고
- 그 중 **0(무효값)을 제외한** 픽셀들의 **중앙값**을 사용 (평균보다 이상치에 강함)
- 경계를 넘어가지 않도록 `max(0, ...)`/`min(w or h, ...)`로 클리핑
- 최종적으로 `depth_scale`(예: 0.001 = mm→m)을 곱해 미터 단위로 반환

---

## 3. `select_target_box(boxes, target_cls_id, locked_track_id)`

**입력**: YOLO `results[0].boxes`, 추적 대상 클래스 ID, 현재 고정된 트랙 ID
**출력**: 선택된 박스 객체 또는 `None`

```python
candidates = [b for b in boxes if int(b.cls[0]) == target_cls_id]
if not candidates:
    return None
if locked_track_id is not None:
    for b in candidates:
        if b.id is not None and int(b.id[0]) == locked_track_id:
            return b
return max(candidates, key=lambda b: float(b.conf[0]))
```

선택 우선순위:
1. **탐지된 박스가 없거나 대상 클래스가 하나도 없으면** → `None`
2. **이미 고정 추적 중인 트랙 ID가 후보 중에 있으면** → 그 박스를 그대로 선택 (다른 후보가 더 신뢰도 높아도 무시 — 트랙 흔들림/스위칭 방지)
3. **고정된 트랙이 없거나 사라졌으면** → 후보 중 **신뢰도(conf)가 가장 높은** 박스 선택 (새로 락온)

이 함수 덕분에 두 노드 모두 "한 번 물고 있는 타겟을 계속 같은 개체로 취급"하는 안정적인 락온 동작을 함.

---

## 4. `apply_homography(u, v, H)`

**입력**: 픽셀 좌표 `(u, v)`, 3x3 호모그래피 행렬 `H`
**출력**: 지면 평면 기준 실세계 좌표 `(x, y)`

```python
pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
out = cv2.perspectiveTransform(pt, H)
x, y = out[0, 0]
return float(x), float(y)
```

- `cv2.perspectiveTransform`으로 픽셀 좌표를 호모그래피 변환 → **평평한 지면**이라는 가정 하에 카메라가 보는 픽셀이 실제 바닥의 어느 지점인지 계산
- `webcam_locator_node`에서 바운딩박스 **하단 중심(접지점)**을 이 함수에 넣는 이유: 호모그래피는 지면 평면에만 유효하므로, 차량의 지붕이나 중심이 아니라 바닥에 닿는 점을 넣어야 정확한 좌표가 나옴

---

## 5. `load_homography_yaml(path)`

**입력**: YAML 파일 경로
**출력**: `np.ndarray` shape `(3, 3)` — 호모그래피 행렬

```python
with open(path, 'r') as f:
    data = yaml.safe_load(f)
values = data['homography_matrix']
H = np.array(values, dtype=np.float64).reshape(3, 3)
return H
```

- `calibrate_webcam_homography.py`(별도 캘리브레이션 스크립트, 추적 실행 전에 한 번 실행해서 웹캠 픽셀↔실세계 좌표 매핑을 구해 YAML로 저장해두는 도구)가 만들어 둔 파일을 읽음
- YAML의 `homography_matrix` 키에 담긴 9개 값을 3x3으로 reshape
- `webcam_locator_node.__init__`에서 이 로드가 실패하면(`OSError`/`KeyError`/`ValueError`) 노드가 즉시 `RuntimeError`로 종료되고 "캘리브레이션 먼저 실행하라"는 메시지를 냄

---

## 요약: 두 함수 그룹

| 그룹 | 함수 | 역할 |
|---|---|---|
| **뎁스 처리** (자체 카메라, 거리 측정) | `decode_compressed_depth`, `sample_depth_patch` | 압축된 뎁스 데이터를 복원하고, 특정 픽셀 위치의 실제 거리(m)를 안정적으로 추출 |
| **탐지 선택** (양쪽 카메라 공통) | `select_target_box` | 여러 탐지 후보 중 "지금 쫓고 있는 그 차"를 일관되게 선택 |
| **좌표 변환** (웹캠 전용) | `apply_homography`, `load_homography_yaml` | 웹캠 픽셀 좌표를 로봇이 이해할 수 있는 지면 월드 좌표로 변환 |
