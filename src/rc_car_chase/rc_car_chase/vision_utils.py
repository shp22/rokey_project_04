import cv2
import numpy as np
import yaml


def decode_compressed_depth(data: bytes):
    """Decode ROS compressed_depth_image_transport payload: 12-byte header
    (depthQuantA/depthQuantB float32 params) followed by PNG-compressed 16UC1 data."""
    if len(data) <= 12:
        return None
    buf = np.frombuffer(data[12:], dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
    return img


def sample_depth_patch(depth_img, u, v, patch_size, depth_scale):
    """Median of valid (nonzero) depth values in a patch_size x patch_size window
    centered at pixel (u, v), converted to meters via depth_scale. Returns None if
    no valid pixels are found."""
    h, w = depth_img.shape[:2]
    half = patch_size // 2
    u0, u1 = max(0, u - half), min(w, u + half + 1)
    v0, v1 = max(0, v - half), min(h, v + half + 1)
    region = depth_img[v0:v1, u0:u1].astype(np.float32)
    valid = region[region > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) * depth_scale


def select_target_box(boxes, target_cls_id, locked_track_id):
    """Pick the box matching target_cls_id, preferring the locked track id if still
    present, otherwise the highest-confidence candidate. Returns None if no match."""
    if boxes is None or len(boxes) == 0:
        return None
    candidates = [b for b in boxes if int(b.cls[0]) == target_cls_id]
    if not candidates:
        return None
    if locked_track_id is not None:
        for b in candidates:
            if b.id is not None and int(b.id[0]) == locked_track_id:
                return b
    return max(candidates, key=lambda b: float(b.conf[0]))


def apply_homography(u, v, H):
    """Map a pixel (u, v) through homography H (3x3) to a ground-plane (x, y)."""
    pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, H)
    x, y = out[0, 0]
    return float(x), float(y)


def load_homography_yaml(path):
    """Load a 3x3 homography matrix saved by calibrate_webcam_homography.py."""
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    values = data['homography_matrix']
    H = np.array(values, dtype=np.float64).reshape(3, 3)
    return H
