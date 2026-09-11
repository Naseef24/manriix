#!/usr/bin/env python3
"""
Manriix Robot - Official Stereolabs Multi-Camera Composition Pattern
All 3 ZED cameras in ONE shared container with shared namespace
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_description = get_package_share_directory('manriix_description')
    pkg_hardware = get_package_share_directory('manriix_hardware')
    pkg_control = get_package_share_directory('manriix_control')
    pkg_rplidar = get_package_share_directory('rplidar_ros')
    pkg_bringup = get_package_share_directory('manriix_bringup')
    
    try:
        pkg_zed_wrapper = get_package_share_directory('zed_wrapper')
        zed_available = True
    except:
        zed_available = False

    zed_config = os.path.join(pkg_bringup, 'config', 'zed', 'zedx_yolo_detection.yaml')

    hardware_mode_arg = DeclareLaunchArgument('hardware_mode', default_value='mock')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    launch_sensors_arg = DeclareLaunchArgument('launch_sensors', default_value='true')
    launch_teleop_arg = DeclareLaunchArgument('launch_teleop', default_value='true')
    launch_ekf_arg = DeclareLaunchArgument('launch_ekf', default_value='true')

    hardware_mode = LaunchConfiguration('hardware_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_sensors = LaunchConfiguration('launch_sensors')
    launch_teleop = LaunchConfiguration('launch_teleop')
    launch_ekf = LaunchConfiguration('launch_ekf')

    robot_description_content = Command(['xacro ', os.path.join(pkg_description, 'urdf', 'manriix2.urdf.xacro'), ' hardware_mode:=', hardware_mode])
    controller_config = os.path.join(pkg_hardware, 'config', 'manriix_controllers.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str), 
                    'use_sim_time': use_sim_time, 'use_tf_static': False}])

    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}, controller_config])

    joint_state_broadcaster_spawner = TimerAction(period=2.0, actions=[
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen')])

    controller_spawner = TimerAction(period=3.0, actions=[
        Node(package='controller_manager', executable='spawner', 
             arguments=['manriix_4ws_controller', '--controller-manager', '/controller_manager'], output='screen')])

    motor_diagnostics = Node(
        package='manriix_hardware', executable='motor_diagnostics_node.py', name='motor_diagnostics', output='screen',
        condition=IfCondition(PythonExpression(["'", hardware_mode, "' == 'real'"])))

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_rplidar, 'launch', 'rplidar_a2m12_launch.py')),
        condition=IfCondition(launch_sensors))

    # ===== ZED CAMERAS - OFFICIAL STEREOLABS SHARED NAMESPACE PATTERN =====
    zed_launch_actions = []
    
    if zed_available:
        # CRITICAL: Shared namespace for all cameras (Stereolabs official pattern)
        shared_namespace = 'manriix_cameras'
        container_name = 'zed_container'
        
        # Create ONE container in shared namespace
        zed_container = ComposableNodeContainer(
            name=container_name,
            namespace=shared_namespace,  # All cameras share this namespace
            package='rclcpp_components',
            executable='component_container_isolated',
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen',
            condition=IfCondition(launch_sensors)
        )
        zed_launch_actions.append(zed_container)
        
        # Camera configurations
        cameras = [
            {'name': 'zedx_front', 'serial': '41911351', 'delay': 2.0, 'publish_tf': 'true'},
            {'name': 'zedx_left', 'serial': '40487925', 'delay': 4.0, 'publish_tf': 'false'},
            {'name': 'zedx_right', 'serial': '46311875', 'delay': 6.0, 'publish_tf': 'false'},
        ]
        
        for cam in cameras:
            cam_launch = TimerAction(period=cam['delay'], actions=[
                LogInfo(msg=f"Loading {cam['name']} into shared container..."),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_zed_wrapper, 'launch', 'zed_camera.launch.py')),
                    launch_arguments={
                        'container_name': container_name,
                        'namespace': shared_namespace,  # CRITICAL: Same namespace as container
                        'camera_model': 'zedx',
                        'camera_name': cam['name'],
                        'node_name': 'zed_node',
                        'serial_number': cam['serial'],
                        'publish_urdf': 'false',
                        'publish_tf': cam['publish_tf'],
                        'publish_map_tf': cam['publish_tf'],
                        'publish_imu_tf': 'false',
                        'ros_params_override_path': zed_config,
                    }.items(),
                    condition=IfCondition(launch_sensors))
            ])
            zed_launch_actions.append(cam_launch)
    else:
        zed_launch_actions.append(LogInfo(msg="WARNING: zed_wrapper not found", 
                                         condition=IfCondition(launch_sensors)))

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_control, 'launch', 'teleop.launch.py')),
        condition=IfCondition(launch_teleop))

    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[os.path.join(pkg_control, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_ekf))

    return LaunchDescription([
        SetEnvironmentVariable('DISPLAY', ':0'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),
        hardware_mode_arg, use_sim_time_arg, launch_sensors_arg, launch_teleop_arg, launch_ekf_arg,
        robot_state_publisher, controller_manager, joint_state_broadcaster_spawner, controller_spawner,
        motor_diagnostics, rplidar_launch,
        *zed_launch_actions,
        teleop_launch, ekf_node,
    ])
