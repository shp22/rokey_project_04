from collections import deque

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from rc_car_chase.vision_utils import apply_homography, load_homography_yaml, select_target_box


class WebcamLocatorNode(Node):
    def __init__(self):
        super().__init__('webcam_locator_node')

        self.declare_parameter('device', '/dev/video2')
        self.declare_parameter('capture_width', 640)
        self.declare_parameter('capture_height', 480)
        self.declare_parameter('capture_fps', 30.0)
        self.declare_parameter('model_path', '/home/rokey/rokey_ws/best_v11.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('tracker', 'bytetrack.yaml')
        self.declare_parameter('target_class_id', 0)
        self.declare_parameter('homography_yaml_path',
                                '/home/rokey/rokey_ws/src/rc_car_chase/config/webcam_homography.yaml')
        self.declare_parameter('target_topic', '/rc_car_chase/webcam_target')
        self.declare_parameter('overlay_topic', '/rc_car_chase/webcam_debug_image')
        self.declare_parameter('publish_overlay', True)
        self.declare_parameter('show_window', True)
        self.declare_parameter('confirm_window_size', 8)
        self.declare_parameter('confirm_min_hits', 6)

        device = self.get_parameter('device').value
        capture_width = self.get_parameter('capture_width').value
        capture_height = self.get_parameter('capture_height').value
        capture_fps = self.get_parameter('capture_fps').value
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.tracker_cfg = self.get_parameter('tracker').value
        self.target_class_id = self.get_parameter('target_class_id').value
        homography_yaml_path = self.get_parameter('homography_yaml_path').value
        target_topic = self.get_parameter('target_topic').value
        overlay_topic = self.get_parameter('overlay_topic').value
        self.publish_overlay = self.get_parameter('publish_overlay').value
        self.show_window = self.get_parameter('show_window').value
        self.confirm_min_hits = self.get_parameter('confirm_min_hits').value
        confirm_window_size = self.get_parameter('confirm_window_size').value

        try:
            self.homography = load_homography_yaml(homography_yaml_path)
        except (OSError, KeyError, ValueError) as exc:
            raise RuntimeError(
                f'Failed to load webcam homography from {homography_yaml_path}: {exc}. '
                f'Run calibrate_webcam_homography first.'
            ) from exc

        self.get_logger().info(f'Loading model: {model_path}')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, capture_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_height)
        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open webcam device {device}')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.target_pub = self.create_publisher(PointStamped, target_topic, qos)
        self.overlay_pub = self.create_publisher(Image, overlay_topic, 1) if self.publish_overlay else None

        self.locked_track_id = None
        self.frame_count = 0
        self.detection_hits = deque(maxlen=confirm_window_size)

        self.timer = self.create_timer(1.0 / capture_fps, self.on_timer)

        self.get_logger().info(
            f'Webcam locator running on {device}, publishing targets on {target_topic}, '
            f'show_window={self.show_window}'
        )

    def on_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('webcam frame grab failed', throttle_duration_sec=2.0)
            return

        self.frame_count += 1
        if self.frame_count % 30 == 1:
            self.get_logger().info(f'webcam: processed {self.frame_count} frames')

        results = self.model.track(
            frame, persist=True, conf=self.conf_threshold,
            tracker=self.tracker_cfg, verbose=False,
        )
        boxes = results[0].boxes
        target = select_target_box(boxes, self.target_class_id, self.locked_track_id)

        self.detection_hits.append(target is not None)
        confirmed = sum(self.detection_hits) >= self.confirm_min_hits

        if target is not None:
            self.locked_track_id = int(target.id[0]) if target.id is not None else self.locked_track_id
            if confirmed:
                x1, y1, x2, y2 = target.xyxy[0].tolist()
                u = (x1 + x2) / 2.0
                v = y2  # bbox bottom = car's ground contact point
                x, y = apply_homography(u, v, self.homography)

                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'
                msg.point.x = x
                msg.point.y = y
                msg.point.z = 0.0
                self.target_pub.publish(msg)
        else:
            self.locked_track_id = None

        if self.publish_overlay:
            overlay = results[0].plot()
            out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            self.overlay_pub.publish(out_msg)
            display_frame = overlay
        else:
            display_frame = frame

        if self.show_window:
            cv2.imshow('webcam_locator', display_frame)
            cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamLocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
