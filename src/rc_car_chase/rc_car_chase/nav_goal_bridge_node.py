import math

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

PHASE1_APPROACH = 'PHASE1_APPROACH'


def yaw_to_quaternion_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class NavGoalBridgeNode(Node):
    """Turns webcam_locator_node's moving-target point into periodic Nav2 NavigateToPose
    goals while chase_controller_node is in PHASE1_APPROACH, so the TurtleBot drives toward
    the RC car through the mapped space while Nav2's costmaps handle obstacle avoidance.
    Goes idle (cancels any active goal) once chase_controller_node hands off to PHASE2_FOLLOW,
    since PHASE2 drives cmd_vel directly from the onboard camera instead.
    """

    def __init__(self):
        super().__init__('nav_goal_bridge_node')

        self._declare_parameters()
        self._read_parameters()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.create_subscription(PointStamped, self.webcam_target_topic, self.on_webcam_target, qos)
        self.create_subscription(String, self.phase_topic, self.on_phase, qos)

        self._nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        self.latest_target = None
        self.latest_target_stamp = None
        self.phase = None
        self.last_phase_stamp = None
        self.last_sent_goal_xy = None
        self.last_sent_stamp = None
        self._goal_handle = None
        self._goal_active = False

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.on_timer)

        self.get_logger().info(
            f'nav_goal_bridge started: enable_nav={self.enable_nav}, '
            f'nav_action={self.nav_action_name}, map_frame={self.map_frame}, '
            f'robot_base_frame={self.robot_base_frame}'
        )

    # ------------------------------------------------------------------ params

    def _declare_parameters(self):
        defaults = {
            'webcam_target_topic': '/rc_car_chase/webcam_target',
            'phase_topic': '/rc_car_chase/phase',
            'nav_action_name': '/robot5/navigate_to_pose',
            'map_frame': 'map',
            'robot_base_frame': 'robot5/base_link',
            'target_stale_timeout': 1.0,
            'phase_stale_timeout': 2.0,
            'goal_update_dist_m': 0.35,
            'goal_min_resend_interval': 1.5,
            'update_rate_hz': 2.0,
            'enable_nav': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.webcam_target_topic = g('webcam_target_topic')
        self.phase_topic = g('phase_topic')
        self.nav_action_name = g('nav_action_name')
        self.map_frame = g('map_frame')
        self.robot_base_frame = g('robot_base_frame')
        self.target_stale_timeout = g('target_stale_timeout')
        self.phase_stale_timeout = g('phase_stale_timeout')
        self.goal_update_dist_m = g('goal_update_dist_m')
        self.goal_min_resend_interval = g('goal_min_resend_interval')
        self.update_rate_hz = g('update_rate_hz')
        self.enable_nav = g('enable_nav')

    # --------------------------------------------------------------- callbacks

    def on_webcam_target(self, msg: PointStamped):
        self.latest_target = msg
        self.latest_target_stamp = self.get_clock().now()

    def on_phase(self, msg: String):
        self.phase = msg.data
        self.last_phase_stamp = self.get_clock().now()

    # ---------------------------------------------------------- control timer

    def on_timer(self):
        now = self.get_clock().now()

        phase_fresh = (
            self.last_phase_stamp is not None
            and (now - self.last_phase_stamp) < Duration(seconds=self.phase_stale_timeout)
        )
        if not phase_fresh or self.phase != PHASE1_APPROACH:
            self._cancel_goal_if_active('phase left PHASE1_APPROACH (or chase_controller phase unknown/stale)')
            return

        if (self.latest_target is None or self.latest_target_stamp is None
                or (now - self.latest_target_stamp) > Duration(seconds=self.target_stale_timeout)):
            self._cancel_goal_if_active('webcam target stale/absent')
            return

        map_xy = self._transform_to_map(self.latest_target)
        if map_xy is None:
            return

        tx, ty = map_xy
        if self._should_send_goal(tx, ty, now):
            self._send_goal(tx, ty, now)

    # ------------------------------------------------------------------- core

    def _transform_to_map(self, point_stamped: PointStamped):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, point_stamped.header.frame_id, Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'tf lookup {point_stamped.header.frame_id}->{self.map_frame} failed: {exc}',
                throttle_duration_sec=2.0,
            )
            return None
        transformed = do_transform_point(point_stamped, tf)
        return transformed.point.x, transformed.point.y

    def _lookup_robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_base_frame, Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y

    def _should_send_goal(self, tx, ty, now) -> bool:
        if self.last_sent_goal_xy is None:
            return True
        dx = tx - self.last_sent_goal_xy[0]
        dy = ty - self.last_sent_goal_xy[1]
        moved_enough = math.hypot(dx, dy) > self.goal_update_dist_m
        stale_enough = (
            self.last_sent_stamp is None
            or (now - self.last_sent_stamp) > Duration(seconds=self.goal_min_resend_interval)
        )
        return moved_enough and stale_enough

    def _send_goal(self, tx, ty, now):
        yaw = 0.0
        robot_xy = self._lookup_robot_xy()
        if robot_xy is not None:
            yaw = math.atan2(ty - robot_xy[1], tx - robot_xy[0])

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = now.to_msg()
        pose.pose.position.x = tx
        pose.pose.position.y = ty
        qz, qw = yaw_to_quaternion_zw(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.last_sent_goal_xy = (tx, ty)
        self.last_sent_stamp = now

        if not self.enable_nav:
            self.get_logger().info(
                f'[DRY RUN] would send NavigateToPose goal x={tx:.2f} y={ty:.2f} yaw={yaw:.2f}',
                throttle_duration_sec=1.0,
            )
            return

        if not self._nav_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('navigate_to_pose action server not available', throttle_duration_sec=5.0)
            return

        if self._goal_active and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._goal_active = True
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('NavigateToPose goal rejected')
            self._goal_active = False
            self._goal_handle = None
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._goal_active = False
        self._goal_handle = None

    def _cancel_goal_if_active(self, reason):
        if self._goal_active and self._goal_handle is not None:
            self.get_logger().info(f'cancelling nav goal: {reason}', throttle_duration_sec=2.0)
            self._goal_handle.cancel_goal_async()
        self._goal_active = False
        self._goal_handle = None
        self.last_sent_goal_xy = None
        self.last_sent_stamp = None


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
