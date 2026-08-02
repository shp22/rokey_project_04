from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/robot5/oakd/rgb/image_raw/compressed'
    )
    compressed_arg = DeclareLaunchArgument('compressed', default_value='true')
    conf_threshold_arg = DeclareLaunchArgument('conf_threshold', default_value='0.5')
    show_window_arg = DeclareLaunchArgument('show_window', default_value='true')
    publish_overlay_arg = DeclareLaunchArgument('publish_overlay', default_value='true')

    params = {
        'image_topic': LaunchConfiguration('image_topic'),
        'compressed': LaunchConfiguration('compressed'),
        'conf_threshold': LaunchConfiguration('conf_threshold'),
        'show_window': LaunchConfiguration('show_window'),
        'publish_overlay': LaunchConfiguration('publish_overlay'),
    }

    node = Node(
        package='car_dum_tracker',
        executable='tracker_node',
        name='car_dum_tracker_node',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([
        image_topic_arg,
        compressed_arg,
        conf_threshold_arg,
        show_window_arg,
        publish_overlay_arg,
        node,
    ])
