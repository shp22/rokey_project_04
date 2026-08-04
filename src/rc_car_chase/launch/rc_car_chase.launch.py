from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('webcam_device', default_value='/dev/video2'),
        DeclareLaunchArgument('webcam_model_path', default_value='/home/rokey/rokey_ws/best_v11.pt'),
        DeclareLaunchArgument(
            'homography_yaml_path',
            default_value='/home/rokey/rokey_ws/src/rc_car_chase/config/webcam_homography.yaml',
        ),
        DeclareLaunchArgument(
            'own_cam_model_path',
            default_value='/home/rokey/rokey_ws/runs/detect/runs_train/car_dum_yolo11n_seqsplit/weights/best.pt',
        ),
        # target_distance doubles as the PHASE2 standoff offset - both phases drive off
        # the webcam target, PHASE2 just aims target_distance short of it (see node docs).
        DeclareLaunchArgument('target_distance', default_value='0.6'),
        DeclareLaunchArgument('own_cam_handoff_max_depth_m', default_value='0.7'),
        DeclareLaunchArgument('own_cam_confirm_frames', default_value='4'),
        DeclareLaunchArgument('odom_topic', default_value='/robot5/odom'),
        DeclareLaunchArgument('enable_cmd_vel', default_value='false'),   # SAFE DEFAULT
        DeclareLaunchArgument('initial_state', default_value='PHASE1_APPROACH'),
        DeclareLaunchArgument('webcam_show_window', default_value='true'),
        DeclareLaunchArgument('controller_show_window', default_value='false'),
        # Set to false when webcam_locator_node is already running on another machine
        # (e.g. the webcam is physically attached there) - its /rc_car_chase/webcam_target
        # topic is visible here over DDS regardless of which host published it.
        DeclareLaunchArgument('bringup_webcam', default_value='true'),
        # Visualization
        DeclareLaunchArgument('bringup_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('rc_car_chase'), 'config', 'rc_car_chase.rviz']),
        ),
        # Nav2 / explore_lite
        DeclareLaunchArgument('bringup_nav2_stack', default_value='true'),
        DeclareLaunchArgument('nav2_action_name', default_value='/robot5/navigate_to_pose'),
        DeclareLaunchArgument('explore_resume_topic', default_value='/robot5/explore/resume'),
        DeclareLaunchArgument('nav2_goal_update_threshold_m', default_value='0.5'),
        DeclareLaunchArgument('nav2_goal_frame_id', default_value='odom'),
        DeclareLaunchArgument('enable_autonomous_exploration', default_value='false'),  # SAFE DEFAULT
        # Auto-undock: undock automatically once the webcam detects the car while docked
        DeclareLaunchArgument('auto_undock', default_value='true'),
        DeclareLaunchArgument('dock_status_topic', default_value='/robot5/dock_status'),
        DeclareLaunchArgument('undock_action_name', default_value='/robot5/undock'),
    ]

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rc_car_chase'), 'launch', 'nav2_stack.launch.py'])),
        condition=IfCondition(LaunchConfiguration('bringup_nav2_stack')),
    )

    webcam_node = Node(
        package='rc_car_chase', executable='webcam_locator_node', name='webcam_locator_node',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('webcam_device'),
            'model_path': LaunchConfiguration('webcam_model_path'),
            'homography_yaml_path': LaunchConfiguration('homography_yaml_path'),
            'show_window': LaunchConfiguration('webcam_show_window'),
        }],
        condition=IfCondition(LaunchConfiguration('bringup_webcam')),
    )

    controller_node = Node(
        package='rc_car_chase', executable='chase_controller_node', name='chase_controller_node',
        output='screen',
        parameters=[{
            'own_cam_model_path': LaunchConfiguration('own_cam_model_path'),
            'target_distance': LaunchConfiguration('target_distance'),
            'own_cam_handoff_max_depth_m': LaunchConfiguration('own_cam_handoff_max_depth_m'),
            'own_cam_confirm_frames': LaunchConfiguration('own_cam_confirm_frames'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'enable_cmd_vel': LaunchConfiguration('enable_cmd_vel'),
            'initial_state': LaunchConfiguration('initial_state'),
            'show_window': LaunchConfiguration('controller_show_window'),
            'nav2_action_name': LaunchConfiguration('nav2_action_name'),
            'explore_resume_topic': LaunchConfiguration('explore_resume_topic'),
            'nav2_goal_update_threshold_m': LaunchConfiguration('nav2_goal_update_threshold_m'),
            'nav2_goal_frame_id': LaunchConfiguration('nav2_goal_frame_id'),
            'enable_autonomous_exploration': LaunchConfiguration('enable_autonomous_exploration'),
            'auto_undock': LaunchConfiguration('auto_undock'),
            'dock_status_topic': LaunchConfiguration('dock_status_topic'),
            'undock_action_name': LaunchConfiguration('undock_action_name'),
        }],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rc_car_chase_rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        # robot5's TF is published on the namespaced /robot5/tf topics, not the bare
        # /tf RViz listens to by default.
        remappings=[('/tf', '/robot5/tf'), ('/tf_static', '/robot5/tf_static')],
        condition=IfCondition(LaunchConfiguration('bringup_rviz')),
    )

    return LaunchDescription(args + [nav2_stack, webcam_node, controller_node, rviz_node])
