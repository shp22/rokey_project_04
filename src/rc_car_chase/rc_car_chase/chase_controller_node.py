import math

import cv2
import message_filters
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from ultralytics import YOLO

from rc_car_chase.vision_utils import decode_compressed_depth, sample_depth_patch, select_target_box

PHASE1_APPROACH = 'PHASE1_APPROACH'
PHASE2_FOLLOW = 'PHASE2_FOLLOW'

SUB_MODE_IDLE = 'IDLE'
SUB_MODE_EXPLORING = 'EXPLORING'
SUB_MODE_NAVIGATING = 'NAVIGATING_TO_TARGET'


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ChaseControllerNode(Node):
    def __init__(self):
        super().__init__('chase_controller_node')

        self._declare_parameters()
        self._read_parameters()

        self.get_logger().info(f'Loading own-camera model: {self.own_cam_model_path}')
        self.own_cam_model = YOLO(self.own_cam_model_path)
        self.bridge = CvBridge()

        # ---- cached sensor state (updated by callbacks, consumed by control timer) ----
        self.state = self.initial_state
        self.latest_webcam_target = None
        self.latest_webcam_stamp = None
        self.latest_own_cam_bbox = None
        self.latest_own_cam_conf = None
        self.latest_own_cam_stamp = None
        self.latest_depth_image = None
        self.latest_depth_stamp = None
        self.latest_lidar_min_front = None
        self.latest_lidar_stamp = None
        self.own_cam_confirm_count = 0
        self.locked_track_id = None
        self.last_known_bearing_sign = 1
        self.search_start_time = None
        self._depth_debug_printed = False
        self.frame_count = 0

        # ---- Nav2 / explore_lite state ----
        self._nav2_goal_handle = None
        self._nav2_goal_pending = False
        self._nav2_last_goal_xy = None
        self._phase1_sub_mode = None
        self._explore_resume_published = None

        reliable_qos = QoSProfile(depth=1)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.history = HistoryPolicy.KEEP_LAST

        depth_qos = QoSProfile(depth=1)
        depth_qos.reliability = (
            ReliabilityPolicy.BEST_EFFORT if self.depth_qos_best_effort else ReliabilityPolicy.RELIABLE
        )
        depth_qos.history = HistoryPolicy.KEEP_LAST

        cb_light = MutuallyExclusiveCallbackGroup()
        cb_vision = MutuallyExclusiveCallbackGroup()
        cb_lidar = MutuallyExclusiveCallbackGroup()
        cb_nav2 = MutuallyExclusiveCallbackGroup()
        cb_timer = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            PointStamped, self.webcam_target_topic, self.on_webcam_target, reliable_qos,
            callback_group=cb_light,
        )
        self.create_subscription(
            LaserScan, self.lidar_topic, self.on_lidar, reliable_qos,
            callback_group=cb_lidar,
        )

        rgb_sub = message_filters.Subscriber(
            self, CompressedImage, self.own_cam_rgb_topic, qos_profile=reliable_qos, callback_group=cb_vision,
        )
        depth_sub = message_filters.Subscriber(
            self, CompressedImage, self.own_cam_depth_topic, qos_profile=depth_qos, callback_group=cb_vision,
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=self.rgb_depth_sync_slop,
        )
        self._sync.registerCallback(self.on_oakd_synced)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 1)
        self.debug_pub = (
            self.create_publisher(Image, self.debug_image_topic, 1) if self.publish_debug_image else None
        )
        self.explore_resume_pub = self.create_publisher(Bool, self.explore_resume_topic, reliable_qos)
        self.handoff_event_pub = self.create_publisher(Marker, self.handoff_event_topic, 1)
        self.nav2_action_client = ActionClient(
            self, NavigateToPose, self.nav2_action_name, callback_group=cb_nav2,
        )

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.on_control_timer, callback_group=cb_timer)

        self.get_logger().info(
            f'chase_controller started: state={self.state}, enable_cmd_vel={self.enable_cmd_vel}, '
            f'enable_autonomous_exploration={self.enable_autonomous_exploration}'
        )

    # ------------------------------------------------------------------ params

    def _declare_parameters(self):
        defaults = {
            'own_cam_rgb_topic': '/robot5/oakd/rgb/image_raw/compressed',
            'own_cam_depth_topic': '/robot5/oakd/stereo/image_raw/compressedDepth',
            'cmd_vel_topic': '/robot5/cmd_vel',
            'webcam_target_topic': '/rc_car_chase/webcam_target',
            'own_cam_model_path':
                '/home/rokey/rokey_ws/runs/detect/runs_train/car_dum_yolo11n_seqsplit/weights/best.pt',
            'own_cam_conf_threshold': 0.5,
            'own_cam_confirm_conf': 0.6,
            'own_cam_confirm_frames': 4,
            'own_cam_handoff_max_depth_m': 0.7,
            'tracker': 'bytetrack.yaml',
            'target_class_id': 0,
            'target_distance': 0.5,
            'distance_deadband': 0.03,
            'depth_scale': 0.001,
            'depth_patch_size': 5,
            'depth_stale_timeout': 0.5,
            'detection_loss_timeout': 1.5,
            'webcam_stale_timeout': 1.0,
            'depth_qos_best_effort': False,
            'rgb_depth_sync_slop': 0.1,
            'fx': 565.6582641601562,
            'cx': 355.6732177734375,
            'phase2_kp_ang': 1.0,
            'phase2_max_ang': 0.6,
            'phase2_kp_lin': 0.5,
            'phase2_max_lin': 0.15,
            'phase2_max_lin_reverse': 0.08,
            'allow_reverse': True,
            'search_ang_speed': 0.3,
            'search_timeout_sec': 15.0,
            'control_rate_hz': 10.0,
            'publish_debug_image': True,
            'debug_image_topic': '/rc_car_chase/debug_image',
            'show_window': False,
            'enable_cmd_vel': False,
            'initial_state': PHASE1_APPROACH,
            'lidar_topic': '/robot5/scan',
            'lidar_front_half_angle': 0.44,
            'lidar_safety_stop_distance': 0.3,
            'lidar_stale_timeout': 1.0,
            'lidar_forward_offset_rad': 0.0,
            'nav2_action_name': '/robot5/navigate_to_pose',
            'explore_resume_topic': '/robot5/explore/resume',
            'handoff_event_topic': '/rc_car_chase/handoff_event',
            'nav2_goal_update_threshold_m': 0.3,
            'nav2_goal_frame_id': 'odom',
            'enable_autonomous_exploration': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.own_cam_rgb_topic = g('own_cam_rgb_topic')
        self.own_cam_depth_topic = g('own_cam_depth_topic')
        self.cmd_vel_topic = g('cmd_vel_topic')
        self.webcam_target_topic = g('webcam_target_topic')
        self.own_cam_model_path = g('own_cam_model_path')
        self.own_cam_conf_threshold = g('own_cam_conf_threshold')
        self.own_cam_confirm_conf = g('own_cam_confirm_conf')
        self.own_cam_confirm_frames = g('own_cam_confirm_frames')
        self.own_cam_handoff_max_depth_m = g('own_cam_handoff_max_depth_m')
        self.tracker_cfg = g('tracker')
        self.target_class_id = g('target_class_id')
        self.target_distance = g('target_distance')
        self.distance_deadband = g('distance_deadband')
        self.depth_scale = g('depth_scale')
        self.depth_patch_size = g('depth_patch_size')
        self.depth_stale_timeout = g('depth_stale_timeout')
        self.detection_loss_timeout = g('detection_loss_timeout')
        self.webcam_stale_timeout = g('webcam_stale_timeout')
        self.depth_qos_best_effort = g('depth_qos_best_effort')
        self.rgb_depth_sync_slop = g('rgb_depth_sync_slop')
        self.fx = g('fx')
        self.cx = g('cx')
        self.phase2_kp_ang = g('phase2_kp_ang')
        self.phase2_max_ang = g('phase2_max_ang')
        self.phase2_kp_lin = g('phase2_kp_lin')
        self.phase2_max_lin = g('phase2_max_lin')
        self.phase2_max_lin_reverse = g('phase2_max_lin_reverse')
        self.allow_reverse = g('allow_reverse')
        self.search_ang_speed = g('search_ang_speed')
        self.search_timeout_sec = g('search_timeout_sec')
        self.control_rate_hz = g('control_rate_hz')
        self.publish_debug_image = g('publish_debug_image')
        self.debug_image_topic = g('debug_image_topic')
        self.show_window = g('show_window')
        self.enable_cmd_vel = g('enable_cmd_vel')
        self.initial_state = g('initial_state')
        self.lidar_topic = g('lidar_topic')
        self.lidar_front_half_angle = g('lidar_front_half_angle')
        self.lidar_safety_stop_distance = g('lidar_safety_stop_distance')
        self.lidar_stale_timeout = g('lidar_stale_timeout')
        self.lidar_forward_offset_rad = g('lidar_forward_offset_rad')
        self.nav2_action_name = g('nav2_action_name')
        self.explore_resume_topic = g('explore_resume_topic')
        self.handoff_event_topic = g('handoff_event_topic')
        self.nav2_goal_update_threshold_m = g('nav2_goal_update_threshold_m')
        self.nav2_goal_frame_id = g('nav2_goal_frame_id')
        self.enable_autonomous_exploration = g('enable_autonomous_exploration')

    # --------------------------------------------------------------- callbacks

    def on_webcam_target(self, msg: PointStamped):
        self.latest_webcam_target = msg.point
        self.latest_webcam_stamp = self.get_clock().now()

    def on_lidar(self, msg: LaserScan):
        front = []
        angle = msg.angle_min
        for r in msg.ranges:
            a = normalize_angle(angle - self.lidar_forward_offset_rad)
            if abs(a) <= self.lidar_front_half_angle and math.isfinite(r) and r > 0.01:
                front.append(r)
            angle += msg.angle_increment
        self.latest_lidar_min_front = min(front) if front else float('inf')
        self.latest_lidar_stamp = self.get_clock().now()

    def on_oakd_synced(self, rgb_msg: CompressedImage, depth_msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_img = decode_compressed_depth(depth_msg.data)
        now = self.get_clock().now()

        if depth_img is None:
            self.get_logger().warn('depth decode failed', throttle_duration_sec=2.0)
        else:
            if not self._depth_debug_printed:
                nz = depth_img[depth_img > 0]
                self.get_logger().info(
                    f'DEPTH SANITY CHECK: dtype={depth_img.dtype} min={int(depth_img.min())} '
                    f'max={int(depth_img.max())} nonzero_min={int(nz.min()) if nz.size else "n/a"} '
                    f'-> verify against depth_scale param ({self.depth_scale})'
                )
                self._depth_debug_printed = True
            self.latest_depth_image = depth_img
            self.latest_depth_stamp = now

        self.frame_count += 1
        if self.frame_count % 30 == 1:
            self.get_logger().info(f'own-cam: processed {self.frame_count} synced frames, state={self.state}')

        results = self.own_cam_model.track(
            frame, persist=True, conf=self.own_cam_conf_threshold,
            tracker=self.tracker_cfg, verbose=False,
        )
        boxes = results[0].boxes
        target = select_target_box(boxes, self.target_class_id, self.locked_track_id)

        if target is not None:
            self.locked_track_id = int(target.id[0]) if target.id is not None else self.locked_track_id
            self.latest_own_cam_bbox = tuple(target.xyxy[0].tolist())
            self.latest_own_cam_conf = float(target.conf[0])
            self.latest_own_cam_stamp = now
            x1, _, x2, _ = self.latest_own_cam_bbox
            u_center = (x1 + x2) / 2.0
            self.last_known_bearing_sign = 1 if (u_center - frame.shape[1] / 2.0) >= 0 else -1
            if self.latest_own_cam_conf >= self.own_cam_confirm_conf:
                self.own_cam_confirm_count = min(self.own_cam_confirm_count + 1, self.own_cam_confirm_frames)
            else:
                self.own_cam_confirm_count = 0
        else:
            self.locked_track_id = None
            self.own_cam_confirm_count = 0

        if self.publish_debug_image:
            overlay = results[0].plot()
            out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            out_msg.header = rgb_msg.header
            self.debug_pub.publish(out_msg)
            if self.show_window:
                cv2.imshow('chase_controller', overlay)
                cv2.waitKey(1)

    # ---------------------------------------------------------- control timer

    def on_control_timer(self):
        now = self.get_clock().now()
        if self.state == PHASE1_APPROACH:
            self.tick_phase1(now)
            return

        twist = self.tick_phase2(now)
        twist = self._apply_lidar_safety(twist, now)

        if self.enable_cmd_vel:
            self.cmd_pub.publish(twist)
        else:
            self.get_logger().info(
                f'[DRY RUN] state={self.state} would publish lin={twist.linear.x:.3f} '
                f'ang={twist.angular.z:.3f}', throttle_duration_sec=1.0,
            )

    def tick_phase1(self, now):
        handoff_ok = self.own_cam_confirm_count >= self.own_cam_confirm_frames
        if handoff_ok and self.own_cam_handoff_max_depth_m > 0:
            z = self._sample_depth_at_bbox_center(now)
            if z is not None and z > self.own_cam_handoff_max_depth_m:
                handoff_ok = False

        if handoff_ok:
            self.get_logger().info('HANDOFF: own camera confirmed car -> PHASE2_FOLLOW')
            self._publish_handoff_event()
            self._cancel_nav2_goal_if_active()
            self.state = PHASE2_FOLLOW
            self.search_start_time = None
            return

        webcam_fresh = (
            self.latest_webcam_target is not None and self.latest_webcam_stamp is not None
            and (now - self.latest_webcam_stamp) <= Duration(seconds=self.webcam_stale_timeout)
        )

        if webcam_fresh:
            self._enter_phase1_sub_mode(SUB_MODE_NAVIGATING)
            self._set_explore_resume(False)
            self._maybe_send_nav2_goal(self.latest_webcam_target.x, self.latest_webcam_target.y)
        elif self.enable_autonomous_exploration:
            self._enter_phase1_sub_mode(SUB_MODE_EXPLORING)
            self._cancel_nav2_goal_if_active()
            self._set_explore_resume(True)
        else:
            self._enter_phase1_sub_mode(SUB_MODE_IDLE)
            self._cancel_nav2_goal_if_active()
            self._set_explore_resume(False)

    def tick_phase2(self, now) -> Twist:
        have_detection = (
            self.latest_own_cam_stamp is not None
            and (now - self.latest_own_cam_stamp) < Duration(seconds=self.detection_loss_timeout)
        )
        if not have_detection:
            return self._search_twist(now)

        self.search_start_time = None
        x1, _, x2, _ = self.latest_own_cam_bbox
        u_center = (x1 + x2) / 2.0
        bearing_angle = math.atan2(u_center - self.cx, self.fx)

        twist = Twist()
        twist.angular.z = clamp(-self.phase2_kp_ang * bearing_angle, -self.phase2_max_ang, self.phase2_max_ang)

        z = self._sample_depth_at_bbox_center(now)
        if z is None:
            twist.linear.x = 0.0
        else:
            err = z - self.target_distance
            if abs(err) < self.distance_deadband:
                twist.linear.x = 0.0
            elif err > 0:
                twist.linear.x = clamp(self.phase2_kp_lin * err, 0.0, self.phase2_max_lin)
            elif self.allow_reverse:
                twist.linear.x = clamp(self.phase2_kp_lin * err, -self.phase2_max_lin_reverse, 0.0)
            else:
                twist.linear.x = 0.0
        return twist

    def _search_twist(self, now) -> Twist:
        if self.search_start_time is None:
            self.search_start_time = now
            self.get_logger().warn('car lost on own camera - starting rotate-search')
        if (now - self.search_start_time) > Duration(seconds=self.search_timeout_sec):
            self.get_logger().error(
                'search timed out - stopping and waiting for re-acquisition', throttle_duration_sec=5.0,
            )
            return Twist()
        twist = Twist()
        twist.angular.z = self.search_ang_speed * self.last_known_bearing_sign
        return twist

    def _sample_depth_at_bbox_center(self, now=None):
        if self.latest_own_cam_bbox is None or self.latest_depth_image is None:
            return None
        if now is not None and (now - self.latest_depth_stamp) > Duration(seconds=self.depth_stale_timeout):
            return None
        x1, y1, x2, y2 = self.latest_own_cam_bbox
        u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
        return sample_depth_patch(self.latest_depth_image, u, v, self.depth_patch_size, self.depth_scale)

    def _apply_lidar_safety(self, twist: Twist, now) -> Twist:
        if (self.latest_lidar_min_front is None or self.latest_lidar_stamp is None
                or (now - self.latest_lidar_stamp) > Duration(seconds=self.lidar_stale_timeout)):
            return twist  # no fresh lidar data - do not block, just skip the safety layer
        if self.latest_lidar_min_front < self.lidar_safety_stop_distance:
            if twist.linear.x > 0.0:
                self.get_logger().warn(
                    f'obstacle {self.latest_lidar_min_front:.2f}m ahead - blocking forward motion',
                    throttle_duration_sec=1.0,
                )
            twist.linear.x = min(twist.linear.x, 0.0)
        return twist

    def _publish_handoff_event(self):
        """Fire a brief RViz marker at the robot's own location when PHASE1 -> PHASE2
        handoff happens, so the transition is visible in the visualization instead of
        only showing up in the log."""
        now_msg = self.get_clock().now().to_msg()
        flash_lifetime = Duration(seconds=2.0).to_msg()

        sphere = Marker()
        sphere.header.frame_id = 'base_link'
        sphere.header.stamp = now_msg
        sphere.ns = 'rc_car_chase/handoff'
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (1.0, 0.85, 0.0, 0.85)
        sphere.lifetime = flash_lifetime
        self.handoff_event_pub.publish(sphere)

        text = Marker()
        text.header.frame_id = 'base_link'
        text.header.stamp = now_msg
        text.ns = 'rc_car_chase/handoff'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.z = 0.5
        text.scale.z = 0.3
        text.color.r, text.color.g, text.color.b, text.color.a = (1.0, 1.0, 1.0, 1.0)
        text.text = 'HANDOFF: PHASE2_FOLLOW'
        text.lifetime = flash_lifetime
        self.handoff_event_pub.publish(text)

    # -------------------------------------------------------- Nav2 / explore

    def _enter_phase1_sub_mode(self, name):
        if self._phase1_sub_mode != name:
            self.get_logger().info(f'PHASE1 sub-mode -> {name}')
            self._phase1_sub_mode = name

    def _set_explore_resume(self, want_resume: bool):
        if not self.enable_cmd_vel:
            want_resume = False  # dry-run: never let explore_lite actually drive
        if want_resume == self._explore_resume_published:
            return
        self.explore_resume_pub.publish(Bool(data=want_resume))
        self._explore_resume_published = want_resume
        self.get_logger().info(f'EXPLORE: resume={want_resume}')

    def _maybe_send_nav2_goal(self, x, y):
        if self._nav2_goal_pending:
            return  # still waiting on accept/reject for a previous send

        if self._nav2_last_goal_xy is not None:
            moved = math.hypot(x - self._nav2_last_goal_xy[0], y - self._nav2_last_goal_xy[1])
            if moved < self.nav2_goal_update_threshold_m:
                return  # target hasn't moved enough - let the current goal keep running

        if not self.enable_cmd_vel:
            self.get_logger().info(
                f'[DRY RUN] would send nav2 goal to ({x:.2f}, {y:.2f})', throttle_duration_sec=1.0,
            )
            self._nav2_last_goal_xy = (x, y)
            return

        self._send_nav2_goal(x, y)

    def _send_nav2_goal(self, x, y):
        if not self.nav2_action_client.server_is_ready():
            self.get_logger().warn('nav2 action server not ready yet, skipping goal', throttle_duration_sec=2.0)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.nav2_goal_frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self._nav2_goal_pending = True
        self._nav2_last_goal_xy = (x, y)
        self.get_logger().info(f'NAV2: sending goal to ({x:.2f}, {y:.2f})')
        send_future = self.nav2_action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_nav2_goal_response)

    def _on_nav2_goal_response(self, future):
        self._nav2_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'nav2 send_goal_async failed: {exc}')
            return
        if not goal_handle.accepted:
            self.get_logger().warn('NAV2: goal rejected')
            return
        self._nav2_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_nav2_result)

    def _on_nav2_result(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'nav2 get_result_async failed: {exc}')
            return
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }.get(result.status, str(result.status))
        self.get_logger().info(f'NAV2: goal finished, status={status_name}')

    def _cancel_nav2_goal_if_active(self):
        if self._nav2_goal_handle is not None:
            cancel_future = self._nav2_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda f: self.get_logger().info('NAV2: cancel request completed')
            )
        self._nav2_goal_handle = None
        self._nav2_goal_pending = False
        self._nav2_last_goal_xy = None

    # ------------------------------------------------------------------ misc

    def destroy_node(self):
        stop = Twist()
        for _ in range(3):
            self.cmd_pub.publish(stop)
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChaseControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
