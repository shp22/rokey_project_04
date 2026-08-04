"""Brings up SLAM (online mapping) + Nav2 + explore_lite frontier exploration
under the robot's namespace. Launchable standalone for staged testing, or
included from rc_car_chase.launch.py.

Requires both this workspace and turtlebot4_ws to be sourced (this launch
file resolves turtlebot4_navigation's share directory via FindPackageShare).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('namespace', default_value='robot5'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam_sync', default_value='true'),
        # true (default) = SLAM, build a fresh map every run.
        # false = localization (AMCL) against the saved `map` yaml below - faster startup,
        # no repeated mapping, but the saved map must already cover the space you drive in.
        DeclareLaunchArgument('use_slam', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value='/home/rokey/rokey_ws/maps/turtle5_map.yaml',
        ),
        # Keep this false for a first cautious bring-up (SLAM+Nav2 only, stage (a) of the
        # verification plan) - explore_lite starts exploring on its own as soon as it's up,
        # with no external node here to hold it paused via explore/resume.
        DeclareLaunchArgument('bringup_explore', default_value='true'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('turtlebot4_navigation'), 'config', 'nav2.yaml']),
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('turtlebot4_navigation'), 'config', 'slam.yaml']),
        ),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('turtlebot4_navigation'), 'config', 'localization.yaml']),
        ),
        DeclareLaunchArgument(
            'explore_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('rc_car_chase'), 'config', 'explore_params.yaml']),
        ),
    ]

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('turtlebot4_navigation'), 'launch', 'slam.launch.py'])),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sync': LaunchConfiguration('slam_sync'),
            'params': LaunchConfiguration('slam_params_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam')),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('turtlebot4_navigation'), 'launch', 'localization.launch.py'])),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'params': LaunchConfiguration('localization_params_file'),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_slam')),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('turtlebot4_navigation'), 'launch', 'nav2.launch.py'])),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
        }.items(),
    )

    # Not using explore_lite's own explore.launch.py: it has no override arg for its
    # config path, and we need costmap_topic/costmap_updates_topic pointed at Nav2's
    # inflated global costmap rather than the raw SLAM map (see config/explore_params.yaml).
    explore_node = Node(
        package='explore_lite', executable='explore', name='explore_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('explore_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('bringup_explore')),
    )

    return LaunchDescription(args + [slam, localization, nav2, explore_node])
