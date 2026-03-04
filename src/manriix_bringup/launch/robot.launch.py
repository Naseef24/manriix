#!/usr/bin/env python3
"""
Manriix Robot Bringup Launch File
Launches the complete robot with configurable hardware mode
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package directories
    pkg_description = get_package_share_directory('manriix_description')
    pkg_hardware = get_package_share_directory('manriix_hardware')
    pkg_control = get_package_share_directory('manriix_control')
    pkg_rplidar = get_package_share_directory('rplidar_ros')
    # pkg_bringup = get_package_share_directory('manriix_bringup')   

    # Check if ZED wrapper is available
    # try:
    #     pkg_zed_wrapper = get_package_share_directory('zed_wrapper')
    #     zed_available = True
    # except Exception:
    #     pkg_zed_wrapper = None
    #     zed_available = False

    # ZED Body Tracking + Object Detection Config (used as ros_params_override_path)
    # zed_config = os.path.join(pkg_bringup, 'config', 'zed', 'zedx_body_tracking.yaml')
    # zed_config = os.path.join(pkg_bringup, 'config', 'zed', 'zedx_yolo_detection.yaml')

    # ==================== LAUNCH ARGUMENTS ====================
    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode',
        default_value='mock',
        description='Hardware mode: mock, gazebo, ignition, real',
        choices=['mock', 'gazebo', 'ignition', 'real']
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # launch_sensors_arg = DeclareLaunchArgument(
    #     'launch_sensors',
    #     default_value='true',
    #     description='Launch sensors (RPLidar, ZED X cameras)'
    # )

    launch_2dlidar_arg = DeclareLaunchArgument(
        'launch_2dlidar',
        default_value='true',
        description='Launch RPLidar 2D laser scanner'
    )

    # launch_cameras_arg = DeclareLaunchArgument(
    #     'launch_cameras',
    #     default_value='true',
    #     description='Launch ZED X cameras (3x)'
    # )

    launch_teleop_arg = DeclareLaunchArgument(
        'launch_teleop',
        default_value='true',
        description='Launch teleop'
    )

    launch_ekf_arg = DeclareLaunchArgument(
        'launch_ekf',
        default_value='true',
        description='Launch EKF localization'
    )

    # Get launch configurations
    hardware_mode = LaunchConfiguration('hardware_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # launch_sensors = LaunchConfiguration('launch_sensors')
    launch_2dlidar = LaunchConfiguration('launch_2dlidar')
    # launch_cameras = LaunchConfiguration('launch_cameras')
    launch_teleop = LaunchConfiguration('launch_teleop')
    launch_ekf = LaunchConfiguration('launch_ekf')

    # ==================== ROBOT DESCRIPTION ====================
    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_description, 'urdf', 'manriix2.urdf.xacro'),
        ' hardware_mode:=', hardware_mode
    ])

    controller_config = os.path.join(pkg_hardware, 'config', 'manriix_controllers.yaml')

    # ==================== CORE NODES ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': use_sim_time,
            'use_tf_static': False,  # ADD THIS - fixes timestamp issue
        }],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(robot_description_content, value_type=str)},
            controller_config
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            )
        ]
    )

    controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['manriix_4ws_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    # ==================== MOTOR DIAGNOSTICS (REAL HARDWARE ONLY) ====================
    motor_diagnostics = Node(
        package='manriix_hardware',
        executable='motor_diagnostics_node.py',
        name='motor_diagnostics',
        output='screen',
        condition=IfCondition(PythonExpression(["'", hardware_mode, "' == 'real'"]))
    )

    # ==================== SENSORS (OPTIONAL) ====================
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'rplidar_a2m12_launch.py')
        ),
        condition=IfCondition(launch_2dlidar)
    )

    # # ==================== ZED X CAMERAS (3x) WITH BODY TRACKING + OBJECT DETECTION ====================
    # zed_launch_actions = []
    
    # if zed_available:
    #     # ZED X Front Camera - launches at 5.0 seconds
    #     zed_front_launch = TimerAction(
    #         period=5.0,
    #         actions=[
    #             GroupAction(
    #                 condition=IfCondition(launch_cameras),
    #                 actions=[
    #                     LogInfo(msg="Launching ZED X Front Camera (SN: 41911351) with Body Tracking + Object Detection..."),
    #                     IncludeLaunchDescription(
    #                         PythonLaunchDescriptionSource(
    #                             os.path.join(pkg_zed_wrapper, 'launch', 'zed_camera.launch.py')
    #                         ),
    #                         launch_arguments={
    #                             'camera_model': 'zedx',
    #                             'camera_name': 'zedx_front',
    #                             'node_name': 'zed_node',
    #                             'serial_number': '41911351',
    #                             'publish_urdf': 'false',
    #                             'publish_tf': 'false',
    #                             'publish_map_tf': 'false',
    #                             'publish_imu_tf': 'false',
    #                             'ros_params_override_path': zed_config,  # CORRECT: Override parameters
    #                         }.items(),
    #                     ),
    #                 ]
    #             )
    #         ]
    #     )
    #     zed_launch_actions.append(zed_front_launch)

    #     # ZED X Left Camera - launches at 10.0 seconds
    #     zed_left_launch = TimerAction(
    #         period=10.0,
    #         actions=[
    #             GroupAction(
    #                 condition=IfCondition(launch_cameras),
    #                 actions=[
    #                     LogInfo(msg="Launching ZED X Left Camera (SN: 40487925) with Body Tracking + Object Detection..."),
    #                     IncludeLaunchDescription(
    #                         PythonLaunchDescriptionSource(
    #                             os.path.join(pkg_zed_wrapper, 'launch', 'zed_camera.launch.py')
    #                         ),
    #                         launch_arguments={
    #                             'camera_model': 'zedx',
    #                             'camera_name': 'zedx_left',
    #                             'node_name': 'zed_node',
    #                             'serial_number': '40487925',
    #                             'publish_urdf': 'false',
    #                             'publish_tf': 'false',
    #                             'publish_map_tf': 'false',
    #                             'publish_imu_tf': 'false',
    #                             'ros_params_override_path': zed_config,  # CORRECT: Override parameters
    #                         }.items(),
    #                     ),
    #                 ]
    #             )
    #         ]
    #     )
    #     zed_launch_actions.append(zed_left_launch)

    #     # ZED X Right Camera - launches at 15.0 seconds
    #     zed_right_launch = TimerAction(
    #         period=15.0,
    #         actions=[
    #             GroupAction(
    #                 condition=IfCondition(launch_cameras),
    #                 actions=[
    #                     LogInfo(msg="Launching ZED X Right Camera (SN: 46311875) with Body Tracking + Object Detection..."),
    #                     IncludeLaunchDescription(
    #                         PythonLaunchDescriptionSource(
    #                             os.path.join(pkg_zed_wrapper, 'launch', 'zed_camera.launch.py')
    #                         ),
    #                         launch_arguments={
    #                             'camera_model': 'zedx',
    #                             'camera_name': 'zedx_right',
    #                             'node_name': 'zed_node',
    #                             'serial_number': '46311875',
    #                             'publish_urdf': 'false',
    #                             'publish_tf': 'false',
    #                             'publish_map_tf': 'false',
    #                             'publish_imu_tf': 'false',
    #                             'ros_params_override_path': zed_config,  # CORRECT: Override parameters
    #                         }.items(),
    #                     ),
    #                 ]
    #             )
    #         ]
    #     )
    #     zed_launch_actions.append(zed_right_launch)
    # else:
    #     zed_warning = LogInfo(
    #         msg="WARNING: zed_wrapper package not found. ZED X cameras will not be launched.",
    #         condition=IfCondition(launch_cameras)
    #     )
    #     zed_launch_actions.append(zed_warning)

    # ==================== TELEOP (OPTIONAL) ====================
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'teleop.launch.py')
        ),
        condition=IfCondition(launch_teleop)
    )

    # ==================== EKF LOCALIZATION (OPTIONAL) ====================
    ekf_config = os.path.join(pkg_control, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_ekf)
    )

    # ==================== RETURN LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        SetEnvironmentVariable('DISPLAY', ':0'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),

        # Arguments
        hardware_mode_arg,
        use_sim_time_arg,
        # launch_sensors_arg,
        launch_2dlidar_arg,
        # launch_cameras_arg,
        launch_teleop_arg,
        launch_ekf_arg,

        # Core nodes
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        controller_spawner,

        # Motor diagnostics (real hardware only)
        motor_diagnostics,

        # Sensors (RPLidar)
        rplidar_launch,

        # ZED X Cameras (3x) with Body Tracking + Object Detection
        # *zed_launch_actions,
        
        # Teleop
        teleop_launch,
        
        # EKF
        ekf_node,
    ])