#!/usr/bin/env python3
"""
Manriix Hardware Launch File
Supports multiple hardware modes: mock, gazebo, ignition, real
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
    
    # Launch arguments
    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode',
        default_value='real',
        description='Hardware mode: mock, gazebo, ignition, real',
        choices=['mock', 'gazebo', 'ignition', 'real']
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    launch_sensors_arg = DeclareLaunchArgument(
        'launch_sensors',
        default_value='true',
        description='Launch sensors (RPLidar, RealSense)'
    )
    
    launch_teleop_arg = DeclareLaunchArgument(
        'launch_teleop',
        default_value='true',
        description='Launch teleop'
    )
    
    # Get launch configurations
    hardware_mode = LaunchConfiguration('hardware_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_sensors = LaunchConfiguration('launch_sensors')
    launch_teleop = LaunchConfiguration('launch_teleop')
    
    # Process xacro with hardware_mode argument
    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_description, 'urdf', 'manriix.urdf.xacro'),
        ' hardware_mode:=', hardware_mode
    ])
    
    # Controller config
    controller_config = os.path.join(pkg_hardware, 'config', 'manriix_controllers.yaml')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': use_sim_time
        }],
    )
    
    # Controller manager (ros2_control node)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(robot_description_content, value_type=str)},
            controller_config
        ],
        output='screen',
    )
    
    # Joint state broadcaster (delayed to wait for controller_manager)
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
    
    # 4WS controller spawner (delayed)
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
    
    # Sensor launches (conditional)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a2m12_launch.py')
        ),
        condition=IfCondition(launch_sensors)
    )
    
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': '',
            'enable_color': 'true',
            'enable_depth': 'true',
            'depth_module.profile': '640x480x15',
            'rgb_camera.profile': '640x480x15',
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'gyro_fps': '200',
            'accel_fps': '100',
            'unite_imu_method': '2',
        }.items(),
        condition=IfCondition(launch_sensors)
    )
    
    # Teleop launch (conditional)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'teleop.launch.py')
        ),
        condition=IfCondition(launch_teleop)
    )
    
    # EKF localization
    ekf_config = os.path.join(pkg_control, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        condition=IfCondition(launch_sensors)
    )
    
    return LaunchDescription([
        # Arguments
        hardware_mode_arg,
        use_sim_time_arg,
        launch_sensors_arg,
        launch_teleop_arg,
        
        # Core nodes
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        controller_spawner,
        
        # Optional: sensors and teleop
        rplidar_launch,
        realsense_launch,
        teleop_launch,
        ekf_node,
    ])
