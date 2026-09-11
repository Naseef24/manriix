#!/usr/bin/env python3
"""
================================================================================
Manriix Robot Bringup — cuVSLAM + nvblox stack

Architecture:
  - 3× ZED X cameras: front has depth ON (for nvblox), sides depth OFF
  - cuVSLAM consumes 6 gray rectified streams (3 stereo pairs) + ZED front IMU
  - cuVSLAM publishes BOTH tf transforms:
      map -> odom            (global, with loop closure)
      odom -> base_footprint (short-term VIO)
  - EKF is DISABLED by default (cuVSLAM owns the tf chain)
  - nvblox subscribes to front ZED depth, reads pose from tf
  - Nav2 consumes nvblox slice via NvbloxCostmapLayer

Launch sequence:

  Terminal 1: this file
    ros2 launch manriix_bringup robot_nav_nvblox.launch.py hardware_mode:=real

  Terminal 2: cuVSLAM
    ros2 launch manriix_navigation cuvslam.launch.py

  Terminal 3: nvblox + Nav2
    ros2 launch manriix_navigation nvblox.launch.py

File: /home/hype/manriix2_ws/src/manriix_bringup/launch/robot_nav_nvblox.launch.py
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    GroupAction, LogInfo, SetEnvironmentVariable
)
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
    pkg_bringup = get_package_share_directory('manriix_bringup')
    pkg_zed_wrapper = get_package_share_directory('zed_wrapper')

    # ==================== LAUNCH ARGUMENTS ====================
    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode', default_value='mock',
        description='Hardware mode: mock, gazebo, ignition, real',
        choices=['mock', 'gazebo', 'ignition', 'real']
    )

    use_motor_diagnostics_arg = DeclareLaunchArgument(
        'use_motor_diagnostics', default_value='true',
        description='Launch motor diagnostics node (real hardware only)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time'
    )

    use_rplidar_arg = DeclareLaunchArgument(
        'use_rplidar', default_value='true',
        description='Use RPLidar 2D laser scanner'
    )

    use_zedx_arg = DeclareLaunchArgument(
        'use_zedx', default_value='true',
        description='Launch 3× ZED X cameras (front with depth, sides no depth)'
    )

    launch_teleop_arg = DeclareLaunchArgument(
        'launch_teleop', default_value='true', description='Launch teleop'
    )

    launch_ekf_arg = DeclareLaunchArgument(
        'launch_ekf', default_value='false',
        description='Launch EKF. Default FALSE because cuVSLAM publishes both '
                    'map->odom AND odom->base_footprint transforms directly. '
                    'Set to true only if you want EKF as a diagnostic/parallel '
                    'odometry source.'
    )

    hardware_mode = LaunchConfiguration('hardware_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_motor_diagnostics = LaunchConfiguration('use_motor_diagnostics')
    use_rplidar = LaunchConfiguration('use_rplidar')
    use_zedx = LaunchConfiguration('use_zedx')
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
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': use_sim_time,
            'use_tf_static': False,
        }],
    )

    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(robot_description_content, value_type=str)},
            controller_config
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster'], output='screen',
        )]
    )

    controller_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=['manriix_4ws_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    # ==================== MOTOR DIAGNOSTICS ====================
    motor_diagnostics = Node(
        package='manriix_hardware', executable='motor_diagnostics_node.py',
        name='motor_diagnostics', output='screen',
        condition=IfCondition(PythonExpression([
            "'", hardware_mode, "' == 'real' and '",
            use_motor_diagnostics, "' == 'true'"
        ]))
    )

    # ==================== RPLIDAR ====================
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'rplidar_a2m12_launch.py')
        ),
        condition=IfCondition(use_rplidar)
    )

    # ==================== TELEOP ====================
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'teleop.launch.py')
        ),
        condition=IfCondition(launch_teleop)
    )

    # ==================== EKF (off by default for nvblox path) ====================
    ekf_config = os.path.join(pkg_control, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_ekf)
    )

    # ==================== LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        SetEnvironmentVariable('DISPLAY', ':0'),
        SetEnvironmentVariable('XAUTHORITY', '/run/user/1000/gdm/Xauthority'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),

        hardware_mode_arg,
        use_sim_time_arg,
        use_motor_diagnostics_arg,
        use_rplidar_arg,
        use_zedx_arg,
        launch_teleop_arg,
        launch_ekf_arg,

        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        controller_spawner,
        motor_diagnostics,
        teleop_launch,
        ekf_node,
        rplidar_launch,
    ])
