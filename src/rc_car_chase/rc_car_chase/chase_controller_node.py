import math

import cv2
import message_filters
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point  # noqa: F401  (registers PointStamped support)
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from ultralytics import YOLO

from rc_car_chase.vision_utils import decode_compressed_depth, sample_depth_patch, select_target_box

PHASE1_APPROACH = 'PHASE1_APPROACH'
PHASE2_FOLLOW = 'PHASE2_FOLLOW'

SUB_MODE_IDLE = 'IDLE'
SUB_MODE_EXPLORING = 'EXPLORING'
SUB_MODE_NAVIGATING = 'NAVIGATING_TO_TARGET'
SUB_MODE_UNDOCKING = 'UNDOCKING'


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ChaseControllerNode(Node):
    def __init__(self):
        super().__init__('chase_controller_node')

        self._declare_parameters()
        self._read_parameters()

        self.get_logger().info(f'Loading own-camera model: {self.own_cam_model_path}')
        self.own_cam_model = YOLO(self.own_cam_model_path)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- cached sensor state (updated by callbacks, consumed by control timer) ----
        self.state = self.initial_state
        self.latest_webcam_target = None
        self.latest_webcam_stamp = None
        self.latest_own_cam_bbox = None
        self.latest_own_cam_conf = None
        self.latest_own_cam_stamp = None
        self.latest_depth_image = None
        self.latest_depth_stamp = None
        self.own_cam_K = None  # (fx, fy, cx, cy) from CameraInfo, once received
        self.own_cam_frame_id = None
        self.latest_own_cam_world_xy = None
        self.latest_own_cam_world_stamp = None
        self.own_cam_confirm_count = 0
        self.locked_track_id = None
        self._depth_debug_printed = False
        self.frame_count = 0
        self.latest_is_docked = None
        self.latest_dock_stamp = None
        self.latest_odom_xy = None
        self.latest_odom_yaw = None

        # ---- PHASE2 rotate-search state (both webcam and own-cam fallback lost) ----
        self._search_active = False
        self._search_base_yaw = None
        self._search_toggle = False
        self._search_last_sent_time = None

        # ---- Nav2 / explore_lite state ----
        self._nav2_goal_handle = None
        self._nav2_goal_pending = False
        self._nav2_last_goal_xy = None
        self._phase1_sub_mode = None
        self._explore_resume_published = None
        self._undock_goal_pending = False

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
        cb_nav2 = MutuallyExclusiveCallbackGroup()
        cb_timer = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            PointStamped, self.webcam_target_topic, self.on_webcam_target, reliable_qos,
            callback_group=cb_light,
        )
        self.create_subscription(
            DockStatus, self.dock_status_topic, self.on_dock_status, reliable_qos,
            callback_group=cb_light,
        )
        self.create_subscription(
            Odometry, self.odom_topic, self.on_odom, reliable_qos,
            callback_group=cb_light,
        )
        self.create_subscription(
            CameraInfo, self.own_cam_info_topic, self.on_own_cam_info, reliable_qos,
            callback_group=cb_light,
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

        self.debug_pub = (
            self.create_publisher(Image, self.debug_image_topic, 1) if self.publish_debug_image else None
        )
        self.debug_depth_pub = (
            self.create_publisher(CompressedImage, self.debug_depth_image_topic, 1)
            if self.publish_debug_depth_image else None
        )
        self.explore_resume_pub = self.create_publisher(Bool, self.explore_resume_topic, reliable_qos)
        self.handoff_event_pub = self.create_publisher(Marker, self.handoff_event_topic, 1)
        self.nav2_action_client = ActionClient(
            self, NavigateToPose, self.nav2_action_name, callback_group=cb_nav2,
        )
        self.undock_action_client = ActionClient(
            self, Undock, self.undock_action_name, callback_group=cb_nav2,
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
            'own_cam_info_topic': '/robot5/oakd/rgb/camera_info',
            'webcam_target_topic': '/rc_car_chase/webcam_target',
            'own_cam_model_path':
                '/home/rokey/rokey_ws/runs/detect/runs_train/car_dum_yolo11n_seqsplit/weights/best.pt',
            'own_cam_conf_threshold': 0.5,
            'own_cam_confirm_conf': 0.6,
            'own_cam_confirm_frames': 4,
            'own_cam_handoff_max_depth_m': 0.7,
            'tracker': 'bytetrack.yaml',
            'target_class_id': 0,
            'target_distance': 0.6,
            'depth_scale': 0.001,
            'depth_patch_size': 5,
            'depth_stale_timeout': 0.5,
            'webcam_stale_timeout': 1.0,
            'depth_qos_best_effort': False,
            'rgb_depth_sync_slop': 0.1,
            'odom_topic': '/robot5/odom',
            'control_rate_hz': 10.0,
            'publish_debug_image': True,
            'debug_image_topic': '/rc_car_chase/debug_image',
            'publish_debug_depth_image': True,
            'debug_depth_image_topic': '/rc_car_chase/debug_depth_image/compressed',
            'debug_depth_max_mm': 5000.0,
            'show_window': False,
            'enable_cmd_vel': False,
            'initial_state': PHASE1_APPROACH,
            'nav2_action_name': '/robot5/navigate_to_pose',
            'explore_resume_topic': '/robot5/explore/resume',
            'handoff_event_topic': '/rc_car_chase/handoff_event',
            'nav2_goal_update_threshold_m': 0.5,
            'nav2_goal_frame_id': 'odom',
            'enable_autonomous_exploration': False,
            'dock_status_topic': '/robot5/dock_status',
            'undock_action_name': '/robot5/undock',
            'auto_undock': True,
            'search_sweep_deg': 45.0,
            'search_goal_resend_interval_sec': 4.0,
            # PHASE2 is close-range following (target_distance is typically well under 1m),
            # so it needs a much finer goal-resend threshold than PHASE1's long-range
            # approach - otherwise the car can drift most of the way to target_distance
            # before a correction is even sent.
            'phase2_goal_update_threshold_m': 0.1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.own_cam_rgb_topic = g('own_cam_rgb_topic')
        self.own_cam_depth_topic = g('own_cam_depth_topic')
        self.own_cam_info_topic = g('own_cam_info_topic')
        self.webcam_target_topic = g('webcam_target_topic')
        self.own_cam_model_path = g('own_cam_model_path')
        self.own_cam_conf_threshold = g('own_cam_conf_threshold')
        self.own_cam_confirm_conf = g('own_cam_confirm_conf')
        self.own_cam_confirm_frames = g('own_cam_confirm_frames')
        self.own_cam_handoff_max_depth_m = g('own_cam_handoff_max_depth_m')
        self.tracker_cfg = g('tracker')
        self.target_class_id = g('target_class_id')
        self.target_distance = g('target_distance')
        self.depth_scale = g('depth_scale')
        self.depth_patch_size = g('depth_patch_size')
        self.depth_stale_timeout = g('depth_stale_timeout')
        self.webcam_stale_timeout = g('webcam_stale_timeout')
        self.depth_qos_best_effort = g('depth_qos_best_effort')
        self.rgb_depth_sync_slop = g('rgb_depth_sync_slop')
        self.odom_topic = g('odom_topic')
        self.control_rate_hz = g('control_rate_hz')
        self.publish_debug_image = g('publish_debug_image')
        self.debug_image_topic = g('debug_image_topic')
        self.publish_debug_depth_image = g('publish_debug_depth_image')
        self.debug_depth_image_topic = g('debug_depth_image_topic')
        self.debug_depth_max_mm = g('debug_depth_max_mm')
        self.show_window = g('show_window')
        self.enable_cmd_vel = g('enable_cmd_vel')
        self.initial_state = g('initial_state')
        self.nav2_action_name = g('nav2_action_name')
        self.explore_resume_topic = g('explore_resume_topic')
        self.handoff_event_topic = g('handoff_event_topic')
        self.nav2_goal_update_threshold_m = g('nav2_goal_update_threshold_m')
        self.nav2_goal_frame_id = g('nav2_goal_frame_id')
        self.enable_autonomous_exploration = g('enable_autonomous_exploration')
        self.dock_status_topic = g('dock_status_topic')
        self.undock_action_name = g('undock_action_name')
        self.auto_undock = g('auto_undock')
        self.search_sweep_deg = g('search_sweep_deg')
        self.search_goal_resend_interval_sec = g('search_goal_resend_interval_sec')
        self.phase2_goal_update_threshold_m = g('phase2_goal_update_threshold_m')

    # --------------------------------------------------------------- callbacks

    def on_webcam_target(self, msg: PointStamped):
        self.latest_webcam_target = msg.point
        self.latest_webcam_stamp = self.get_clock().now()

    def on_dock_status(self, msg: DockStatus):
        self.latest_is_docked = msg.is_docked
        self.latest_dock_stamp = self.get_clock().now()

    def on_odom(self, msg: Odometry):
        # position: PHASE2 standoff-offset calculation (PHASE1 leaves all positioning to Nav2).
        # yaw: base heading for the PHASE2 rotate-search sweep when the target is lost.
        p = msg.pose.pose.position
        self.latest_odom_xy = (p.x, p.y)
        self.latest_odom_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def on_own_cam_info(self, msg: CameraInfo):
        self.own_cam_K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy
        self.own_cam_frame_id = msg.header.frame_id

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
            self._publish_debug_depth(depth_img, depth_msg.header)

        self.frame_count += 1
        if self.frame_count % 30 == 1:
            self.get_logger().info(f'own-cam: processed {self.frame_count} synced frames, state={self.state}')

        # Own camera is only used to CONFIRM the car is really close (for the PHASE1 ->
        # PHASE2 handoff gate) - actual PHASE2 driving is done via the webcam target, same
        # as PHASE1, just with a standoff offset. See tick_phase2().
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
            if self.latest_own_cam_conf >= self.own_cam_confirm_conf:
                self.own_cam_confirm_count = min(self.own_cam_confirm_count + 1, self.own_cam_confirm_frames)
            else:
                self.own_cam_confirm_count = 0
            self._update_own_cam_world_fallback(now)
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

    def _publish_debug_depth(self, depth_img, header):
        """Colorized depth visualization (16UC1 mm -> BGR heatmap), published as a
        CompressedImage since the raw depth stream itself uses ROS's special
        compressedDepth format that most viewers can't open directly - this is a
        normal JPEG-compressed color image instead, viewable in rqt_image_view etc."""
        if self.debug_depth_pub is None and not self.show_window:
            return
        clipped = np.clip(depth_img.astype(np.float32), 0, self.debug_depth_max_mm)
        depth_8u = (clipped / self.debug_depth_max_mm * 255.0).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)
        depth_color[depth_img == 0] = (0, 0, 0)  # invalid/no-return pixels -> black

        if self.debug_depth_pub is not None:
            out_msg = self.bridge.cv2_to_compressed_imgmsg(depth_color, dst_format='jpg')
            out_msg.header = header
            self.debug_depth_pub.publish(out_msg)

        if self.show_window:
            cv2.imshow('chase_controller_depth', depth_color)
            cv2.waitKey(1)

    # ---------------------------------------------------------- control timer

    def on_control_timer(self):
        now = self.get_clock().now()
        if self.state == PHASE1_APPROACH:
            self.tick_phase1(now)
        else:
            self.tick_phase2(now)

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
            return

        webcam_fresh = (
            self.latest_webcam_target is not None and self.latest_webcam_stamp is not None
            and (now - self.latest_webcam_stamp) <= Duration(seconds=self.webcam_stale_timeout)
        )

        if webcam_fresh and self.auto_undock and self.latest_is_docked:
            self._enter_phase1_sub_mode(SUB_MODE_UNDOCKING)
            self._set_explore_resume(False)
            self._maybe_send_undock_goal()
        elif webcam_fresh:
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

    def tick_phase2(self, now):
        """PHASE2: still webcam-driven, like PHASE1, but the goal is offset back along
        the robot->target line by target_distance so the robot stops short of the car
        instead of driving into it. Requires knowing our own position (from odom) to
        compute that offset. Falls back to the own-camera depth-derived world position
        (_update_own_cam_world_fallback) when the webcam target goes stale - e.g. the
        car is close enough that it's out of the fixed webcam's view but still visible
        to the robot's own depth camera."""
        webcam_fresh = (
            self.latest_webcam_target is not None and self.latest_webcam_stamp is not None
            and (now - self.latest_webcam_stamp) <= Duration(seconds=self.webcam_stale_timeout)
        )
        own_cam_world_fresh = (
            self.latest_own_cam_world_xy is not None and self.latest_own_cam_world_stamp is not None
            and (now - self.latest_own_cam_world_stamp) <= Duration(seconds=self.depth_stale_timeout)
        )

        if webcam_fresh:
            self._search_active = False
            target_x, target_y = self.latest_webcam_target.x, self.latest_webcam_target.y
        elif own_cam_world_fresh:
            self._search_active = False
            self.get_logger().info(
                'PHASE2: webcam target stale, following via own-camera depth instead',
                throttle_duration_sec=2.0,
            )
            target_x, target_y = self.latest_own_cam_world_xy
        else:
            self._enter_search_mode(now)
            return

        if self.latest_odom_xy is None:
            self._cancel_nav2_goal_if_active()
            return
        robot_x, robot_y = self.latest_odom_xy
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.hypot(dx, dy)

        if dist <= self.target_distance:
            # already within the standoff distance - cancel any in-flight approach goal
            # so the robot doesn't keep coasting forward on stale momentum and crowd the car.
            self._cancel_nav2_goal_if_active()
            return

        ratio = (dist - self.target_distance) / dist
        goal_x = robot_x + dx * ratio
        goal_y = robot_y + dy * ratio
        yaw = math.atan2(dy, dx)
        self._maybe_send_nav2_goal(goal_x, goal_y, yaw=yaw, threshold=self.phase2_goal_update_threshold_m)

    def _enter_search_mode(self, now):
        """PHASE2 target lost on both webcam and own-camera fallback: sweep the robot's
        heading back and forth between -search_sweep_deg and +search_sweep_deg (relative
        to the heading it had when it lost the target) via small in-place-rotation Nav2
        goals, until either source reacquires the car. Bypasses _maybe_send_nav2_goal's
        XY-distance resend gate since these goals share the same (x, y) and only the
        yaw changes between sends."""
        if self.latest_odom_xy is None or self.latest_odom_yaw is None:
            self._cancel_nav2_goal_if_active()
            return

        if not self._search_active:
            self._search_active = True
            self._search_base_yaw = self.latest_odom_yaw
            self._search_toggle = False
            self._search_last_sent_time = None
            self.get_logger().warn(
                f'PHASE2: target lost on both sources - rotate-search '
                f'(+/-{self.search_sweep_deg:.0f} deg)'
            )

        due = (
            self._search_last_sent_time is None
            or (now - self._search_last_sent_time) >= Duration(seconds=self.search_goal_resend_interval_sec)
        )
        if not due:
            return

        sign = 1.0 if self._search_toggle else -1.0
        target_yaw = self._search_base_yaw + sign * math.radians(self.search_sweep_deg)
        self._search_toggle = not self._search_toggle
        self._search_last_sent_time = now

        if not self.enable_cmd_vel:
            self.get_logger().info(
                f'[DRY RUN] would send search-rotation goal to yaw={math.degrees(target_yaw):.0f} deg',
                throttle_duration_sec=1.0,
            )
            return

        x, y = self.latest_odom_xy
        self._send_nav2_goal(x, y, yaw=target_yaw)

    def _sample_depth_at_bbox_center(self, now=None):
        if self.latest_own_cam_bbox is None or self.latest_depth_image is None:
            return None
        if now is not None and (now - self.latest_depth_stamp) > Duration(seconds=self.depth_stale_timeout):
            return None
        x1, y1, x2, y2 = self.latest_own_cam_bbox
        u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
        return sample_depth_patch(self.latest_depth_image, u, v, self.depth_patch_size, self.depth_scale)

    def _update_own_cam_world_fallback(self, now):
        """PHASE2 fallback source: project the own-camera bbox+depth into a 3D point in
        the camera frame, then TF-transform it into nav2_goal_frame_id, for when the
        webcam can't see the car (e.g. it's right in front of the robot, out of the
        fixed webcam's view) but the robot's own depth camera still can."""
        if self.own_cam_K is None or self.own_cam_frame_id is None:
            self.get_logger().warn(
                f'own-cam world fallback unavailable: no CameraInfo received yet on '
                f'{self.own_cam_info_topic} - check the topic name/QoS if this persists',
                throttle_duration_sec=5.0,
            )
            return
        z = self._sample_depth_at_bbox_center(now)
        if z is None:
            self.get_logger().warn(
                'own-cam world fallback unavailable: no valid depth at bbox center',
                throttle_duration_sec=5.0,
            )
            return

        x1, y1, x2, y2 = self.latest_own_cam_bbox
        u = (x1 + x2) / 2.0
        v = (y1 + y2) / 2.0
        fx, fy, cx, cy = self.own_cam_K

        pt_cam = PointStamped()
        pt_cam.header.frame_id = self.own_cam_frame_id
        # Zero stamp = "use the latest available transform" - same reasoning as the Nav2
        # goal stamp: avoids extrapolation errors if this lookup is delayed.
        pt_cam.header.stamp = Time().to_msg()
        pt_cam.point.x = (u - cx) * z / fx
        pt_cam.point.y = (v - cy) * z / fy
        pt_cam.point.z = z

        try:
            pt_world = self.tf_buffer.transform(pt_cam, self.nav2_goal_frame_id, timeout=Duration(seconds=0.2))
        except Exception as exc:
            self.get_logger().warn(f'own-cam depth->world TF transform failed: {exc}', throttle_duration_sec=2.0)
            return
        self.latest_own_cam_world_xy = (pt_world.point.x, pt_world.point.y)
        self.latest_own_cam_world_stamp = now

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

    def _maybe_send_nav2_goal(self, x, y, yaw=None, threshold=None):
        if threshold is None:
            threshold = self.nav2_goal_update_threshold_m  # PHASE1 long-range default

        if self._nav2_goal_pending:
            return  # still waiting on accept/reject for a previous send

        if self._nav2_last_goal_xy is not None:
            moved = math.hypot(x - self._nav2_last_goal_xy[0], y - self._nav2_last_goal_xy[1])
            if moved < threshold:
                return  # target hasn't moved enough - let the current goal keep running

        if not self.enable_cmd_vel:
            self.get_logger().info(
                f'[DRY RUN] would send nav2 goal to ({x:.2f}, {y:.2f})', throttle_duration_sec=1.0,
            )
            self._nav2_last_goal_xy = (x, y)
            return

        self._send_nav2_goal(x, y, yaw)

    def _send_nav2_goal(self, x, y, yaw=None):
        if not self.nav2_action_client.server_is_ready():
            self.get_logger().warn('nav2 action server not ready yet, skipping goal', throttle_duration_sec=2.0)
            return

        if self._nav2_goal_handle is not None:
            # cancel the in-flight goal ourselves rather than relying on Nav2's implicit
            # preemption - keeps our own goal-handle bookkeeping unambiguous and avoids a
            # stray result callback for the superseded goal.
            self._nav2_goal_handle.cancel_goal_async()
            self._nav2_goal_handle = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.nav2_goal_frame_id
        # Zero stamp (rather than "now") tells tf2 to use the latest available transform
        # instead of the one at this exact instant - if the goal message itself sits in a
        # congested DDS queue for a while before Nav2 processes it, a "now" stamp can fall
        # outside the TF buffer's retention window by the time it's looked up, causing an
        # immediate extrapolation-error ABORT even though the goal was fine when sent.
        goal_msg.pose.header.stamp = Time().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        if yaw is None:
            goal_msg.pose.pose.orientation.w = 1.0
        else:
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._nav2_goal_pending = True
        self.get_logger().info(f'NAV2: sending goal to ({x:.2f}, {y:.2f})')
        send_future = self.nav2_action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f: self._on_nav2_goal_response(f, x, y))

    def _on_nav2_goal_response(self, future, x, y):
        self._nav2_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'nav2 send_goal_async failed: {exc}')
            return
        if not goal_handle.accepted:
            # do NOT record (x, y) as the last-sent goal here - a rejected goal must not
            # suppress the next attempt via the goal-update-threshold check below.
            self.get_logger().warn('NAV2: goal rejected')
            return
        self._nav2_last_goal_xy = (x, y)
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

    def _maybe_send_undock_goal(self):
        if self._undock_goal_pending:
            return  # still waiting on accept/reject or result for a previous send

        if not self.enable_cmd_vel:
            self.get_logger().info(
                '[DRY RUN] would send undock goal (car detected while docked)', throttle_duration_sec=2.0,
            )
            return

        if not self.undock_action_client.server_is_ready():
            self.get_logger().warn('undock action server not ready yet', throttle_duration_sec=2.0)
            return

        self._undock_goal_pending = True
        self.get_logger().info('AUTO-UNDOCK: car detected while docked - undocking')
        send_future = self.undock_action_client.send_goal_async(Undock.Goal())
        send_future.add_done_callback(self._on_undock_goal_response)

    def _on_undock_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'undock send_goal_async failed: {exc}')
            self._undock_goal_pending = False
            return
        if not goal_handle.accepted:
            self.get_logger().warn('AUTO-UNDOCK: goal rejected')
            self._undock_goal_pending = False
            return
        goal_handle.get_result_async().add_done_callback(self._on_undock_result)

    def _on_undock_result(self, future):
        self._undock_goal_pending = False
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f'undock get_result_async failed: {exc}')
            return
        self.get_logger().info('AUTO-UNDOCK: finished')

    # ------------------------------------------------------------------ misc

    def destroy_node(self):
        if rclpy.ok():
            self._cancel_nav2_goal_if_active()
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
