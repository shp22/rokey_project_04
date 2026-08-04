"""Runs only webcam_locator_node - for the case where the webcam is physically
attached to a different machine than the one running Nav2/chase_controller_node.

DDS makes /rc_car_chase/webcam_target visible on any machine sharing the same
ROS_DOMAIN_ID (and discovery server, if one is configured) - no special network
setup beyond that is needed. On the Nav2/controller machine, launch
rc_car_chase.launch.py with bringup_webcam:=false so it doesn't also try to
open the (nonexistent, on that machine) webcam device.
"""
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
        DeclareLaunchArgument('webcam_show_window', default_value='true'),
        DeclareLaunchArgument('target_topic', default_value='/rc_car_chase/webcam_target'),
    ]

    webcam_node = Node(
        package='rc_car_chase', executable='webcam_locator_node', name='webcam_locator_node',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('webcam_device'),
            'model_path': LaunchConfiguration('webcam_model_path'),
            'homography_yaml_path': LaunchConfiguration('homography_yaml_path'),
            'show_window': LaunchConfiguration('webcam_show_window'),
            'target_topic': LaunchConfiguration('target_topic'),
        }],
    )

    return LaunchDescription(args + [webcam_node])
