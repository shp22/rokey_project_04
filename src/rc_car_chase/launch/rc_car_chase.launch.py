from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        DeclareLaunchArgument('target_distance', default_value='0.35'),
        DeclareLaunchArgument('own_cam_confirm_frames', default_value='4'),
        DeclareLaunchArgument('detection_loss_timeout', default_value='1.5'),
        DeclareLaunchArgument('phase1_forward_speed', default_value='0.18'),
        DeclareLaunchArgument('phase1_max_lin', default_value='0.20'),
        DeclareLaunchArgument('phase1_max_ang', default_value='0.9'),
        DeclareLaunchArgument('phase2_max_lin', default_value='0.22'),
        DeclareLaunchArgument('phase2_max_lin_reverse', default_value='0.10'),
        DeclareLaunchArgument('phase2_max_ang', default_value='0.9'),
        DeclareLaunchArgument('lidar_safety_stop_distance', default_value='0.3'),
        DeclareLaunchArgument('enable_cmd_vel', default_value='false'),   # SAFE DEFAULT
        DeclareLaunchArgument('initial_state', default_value='PHASE1_APPROACH'),
        DeclareLaunchArgument('webcam_show_window', default_value='true'),
        DeclareLaunchArgument('controller_show_window', default_value='false'),
    ]

    webcam_node = Node(
        package='rc_car_chase', executable='webcam_locator_node', name='webcam_locator_node',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('webcam_device'),
            'model_path': LaunchConfiguration('webcam_model_path'),
            'homography_yaml_path': LaunchConfiguration('homography_yaml_path'),
            'show_window': LaunchConfiguration('webcam_show_window'),
        }],
    )

    controller_node = Node(
        package='rc_car_chase', executable='chase_controller_node', name='chase_controller_node',
        output='screen',
        parameters=[{
            'own_cam_model_path': LaunchConfiguration('own_cam_model_path'),
            'target_distance': LaunchConfiguration('target_distance'),
            'own_cam_confirm_frames': LaunchConfiguration('own_cam_confirm_frames'),
            'detection_loss_timeout': LaunchConfiguration('detection_loss_timeout'),
            'phase1_forward_speed': LaunchConfiguration('phase1_forward_speed'),
            'phase1_max_lin': LaunchConfiguration('phase1_max_lin'),
            'phase1_max_ang': LaunchConfiguration('phase1_max_ang'),
            'phase2_max_lin': LaunchConfiguration('phase2_max_lin'),
            'phase2_max_lin_reverse': LaunchConfiguration('phase2_max_lin_reverse'),
            'phase2_max_ang': LaunchConfiguration('phase2_max_ang'),
            'lidar_safety_stop_distance': LaunchConfiguration('lidar_safety_stop_distance'),
            'enable_cmd_vel': LaunchConfiguration('enable_cmd_vel'),
            'initial_state': LaunchConfiguration('initial_state'),
            'show_window': LaunchConfiguration('controller_show_window'),
        }],
    )

    return LaunchDescription(args + [webcam_node, controller_node])
