from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rc_car_chase = get_package_share_directory('rc_car_chase')
    pkg_tb4_nav = get_package_share_directory('turtlebot4_navigation')

    args = [
        # --- robot / map ---
        DeclareLaunchArgument('namespace', default_value='robot5'),     # robot5
        DeclareLaunchArgument('map', default_value='/home/rokey/b_4_map/maps/turtle5_map.yaml'),    # 생성한 지도
        DeclareLaunchArgument('use_sim_time', default_value='false'),  # 실제 로봇으로 사용
        DeclareLaunchArgument(
            'localization_params',
            default_value=PathJoinSubstitution([pkg_rc_car_chase, 'config', 'localization_robot5.yaml']),
        ),      # 설정값 경로(로봇이 지도상에 어디에 위치해있는지)
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([pkg_rc_car_chase, 'config', 'nav2_robot5.yaml']),
        ),      # 설정값 경로(로봇의 자율주행)

        # --- webcam locator ---
        DeclareLaunchArgument('webcam_device', default_value='/dev/video2'),    # 웹캠 장치
        DeclareLaunchArgument('webcam_model_path', default_value='/home/rokey/b_4/src/rc_car_chase/best_v11.pt'),   # YOLO 모델
        DeclareLaunchArgument(
            'homography_yaml_path',
            default_value=PathJoinSubstitution([pkg_rc_car_chase, 'config', 'webcam_homography.yaml']),
        ),  # 웹캠의 픽셀 좌표를 지도 좌표로 변환하는 호모그래피 설정 파일
        DeclareLaunchArgument('webcam_show_window', default_value='true'),  # 웹캠 창 띄우기
        DeclareLaunchArgument('odom_frame_id', default_value='robot5/odom'),    # odom 이름 맟추기

        # --- chase controller (PHASE2 close-range follow; PHASE1 delegated to Nav2 below) ---
        DeclareLaunchArgument(
            'own_cam_model_path',
            default_value='/home/rokey/b_4/runs/detect/runs_train/car_dum_yolo11n_seqsplit/weights/best.pt',
        ),      # 로봇이 사용하는 YOLO 모델
        DeclareLaunchArgument('target_distance', default_value='0.6'),    # 로봇이 RC카랑 유지할 거리 
        DeclareLaunchArgument('own_cam_confirm_frames', default_value='4'),     # 4프레임 이상 검출되어야 RC카 인식
        DeclareLaunchArgument('detection_loss_timeout', default_value='1.5'),   # RC카 인식을 1.5초 이상 못하면 타겟 놓침 상태 전환
        DeclareLaunchArgument('phase2_max_lin', default_value='0.15'),      #  근접 추적 시 최대 선속도
        DeclareLaunchArgument('phase2_max_ang', default_value='0.6'),       # 근접 추적 시 최대 각속도
        DeclareLaunchArgument('lidar_safety_stop_distance', default_value='0.3'),   # 라이다 장애물 인식시 비상 정지하는 거리
        DeclareLaunchArgument('enable_cmd_vel', default_value='false'),   # SAFE DEFAULT (PHASE2 dry-run)
        DeclareLaunchArgument('initial_state', default_value='PHASE1_APPROACH'),    # 처음 상태 : 웹캠으로 RC카 찾기
        DeclareLaunchArgument('controller_show_window', default_value='false'),     

        # --- nav goal bridge (PHASE1: webcam target -> map frame -> NavigateToPose) ---
        DeclareLaunchArgument('goal_update_dist_m', default_value='0.35'),      # Nav가 실행되기 위한 RC카와 로봇의 거리
        DeclareLaunchArgument('goal_min_resend_interval', default_value='1.5'), # Nav 목표 재전송 주기
        DeclareLaunchArgument('enable_nav', default_value='false'),   # SAFE DEFAULT (dry-run, no goals sent)
    ]

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_nav, 'launch', 'localization.launch.py'])),       # 로봇의 위치 추정
        launch_arguments={
            'namespace': namespace,
            'map': LaunchConfiguration('map'),
            'use_sim_time': use_sim_time,
            'params': LaunchConfiguration('localization_params'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_nav, 'launch', 'nav2.launch.py'])),       # 로봇의 자율주행 경로 계획
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('nav2_params'),
        }.items(),
    )

    webcam_node = Node(                                                             # 웹캠 기반 RC카 위치 검출
        package='rc_car_chase', executable='webcam_locator_node', name='webcam_locator_node',   
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('webcam_device'),
            'model_path': LaunchConfiguration('webcam_model_path'),
            'homography_yaml_path': LaunchConfiguration('homography_yaml_path'),
            'show_window': LaunchConfiguration('webcam_show_window'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
        }],
    )

    controller_node = Node(                                                         # 로봇 카메라로 RC카 인식 및 거리 제어
        package='rc_car_chase', executable='chase_controller_node', name='chase_controller_node',
        output='screen',
        parameters=[{
            'own_cam_model_path': LaunchConfiguration('own_cam_model_path'),
            'target_distance': LaunchConfiguration('target_distance'),
            'own_cam_confirm_frames': LaunchConfiguration('own_cam_confirm_frames'),
            'detection_loss_timeout': LaunchConfiguration('detection_loss_timeout'),
            'phase2_max_lin': LaunchConfiguration('phase2_max_lin'),
            'phase2_max_ang': LaunchConfiguration('phase2_max_ang'),
            'lidar_safety_stop_distance': LaunchConfiguration('lidar_safety_stop_distance'),
            'enable_cmd_vel': LaunchConfiguration('enable_cmd_vel'),
            'initial_state': LaunchConfiguration('initial_state'),
            'show_window': LaunchConfiguration('controller_show_window'),
            'use_nav2_phase1': True,
        }],
    )

    nav_goal_bridge_node = Node(                                                        # 웹캠 좌표를 Nav의 목표로 전환 및 발행
        package='rc_car_chase', executable='nav_goal_bridge_node', name='nav_goal_bridge_node',
        output='screen',
        parameters=[{
            'nav_action_name': PathJoinSubstitution(['/', namespace, 'navigate_to_pose']),
            'robot_base_frame': PathJoinSubstitution([namespace, 'base_link']),
            'goal_update_dist_m': LaunchConfiguration('goal_update_dist_m'),
            'goal_min_resend_interval': LaunchConfiguration('goal_min_resend_interval'),
            'enable_nav': LaunchConfiguration('enable_nav'),
        }],
    )

    return LaunchDescription(
        args + [localization, nav2, webcam_node, controller_node, nav_goal_bridge_node]
    )
