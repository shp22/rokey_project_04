import os

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from ultralytics import YOLO

_MODEL_CANDIDATES = [
    '/home/rokey/rokey_ws/runs/detect/runs_train/car_dum_yolo11n_seqsplit/weights/best.pt',
    '/home/rokey/rokey_ws/runs/detect/runs_train/car_dum_yolo11n/weights/best.pt',
]


def _default_model_path():
    for path in _MODEL_CANDIDATES:
        if os.path.exists(path):
            return path
    return 'yolo11n.pt'


class TrackerNode(Node):
    def __init__(self):
        super().__init__('car_dum_tracker_node')

        self.declare_parameter('image_topic', '/robot5/oakd/rgb/image_raw/compressed')
        self.declare_parameter('compressed', True)
        self.declare_parameter('overlay_topic', 'image_overlay')
        self.declare_parameter('model_path', _default_model_path())
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('tracker', 'bytetrack.yaml')
        self.declare_parameter('show_window', True)
        self.declare_parameter('publish_overlay', True)

        image_topic = self.get_parameter('image_topic').value
        self.compressed = self.get_parameter('compressed').value
        overlay_topic = self.get_parameter('overlay_topic').value
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.tracker_cfg = self.get_parameter('tracker').value
        self.show_window = self.get_parameter('show_window').value
        self.publish_overlay = self.get_parameter('publish_overlay').value

        self.get_logger().info(f'Loading model: {model_path}')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        msg_type = CompressedImage if self.compressed else Image
        callback = self.on_compressed_image if self.compressed else self.on_image
        self.sub = self.create_subscription(msg_type, image_topic, callback, qos)

        self.pub = None
        if self.publish_overlay:
            self.pub = self.create_publisher(Image, overlay_topic, 1)

        self.get_logger().info(
            f'Subscribed to {image_topic}, overlay -> '
            f'{overlay_topic if self.publish_overlay else "(disabled)"}, '
            f'show_window={self.show_window}'
        )
        self.frame_count = 0

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_frame(frame, msg.header)

    def on_compressed_image(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_frame(frame, msg.header)

    def process_frame(self, frame, header):
        self.frame_count += 1
        if self.frame_count % 30 == 1:
            self.get_logger().info(f'Processed {self.frame_count} frames so far')

        results = self.model.track(
            frame,
            persist=True,
            conf=self.conf_threshold,
            tracker=self.tracker_cfg,
            verbose=False,
        )
        overlay = results[0].plot()

        if self.publish_overlay and self.pub is not None:
            out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            out_msg.header = header
            self.pub.publish(out_msg)

        if self.show_window:
            cv2.imshow('car_dum_tracker', overlay)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
