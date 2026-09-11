#!/usr/bin/env python3
"""
================================================================================
Manriix Hardware Only Bringup - For Isaac Perceptor Testing
================================================================================
Launches ONLY the hardware stack - NO cameras, NO ZED, NO perception nodes.
This is the underlay for Isaac Perceptor tests.

What this launches:
  - robot_state_publisher (URDF + TF tree)
  - controller_manager + spawners (ros2_control)
  - EKF (wheel odometry fusion)
  - RPLidar
  - Teleop (optional)

What this intentionally EXCLUDES:
  - direct_zed_detection_node  (replaced by ZED wrapper in perceptor stack)
  - zed_zmq_bridge_node        (camera dependent)
  - photo_capture_bridge_node  (camera dependent)
  - oak_d_pro_w_node           (not needed for perceptor test)
  - d455_node                  (not needed for perceptor test)

Usage:
  ros2 launch manriix_bringup hardware_only.launch.py
  ros2 launch manriix_bringup hardware_only.launch.py use_teleop:=false
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, SetEnvironmentVariable, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────────────
    pkg_description = get_package_share_directory('manriix_description')
    pkg_hardware    = get_package_share_directory('manriix_hardware')
    pkg_control     = get_package_share_directory('manriix_control')
    pkg_rplidar     = get_package_share_directory('rplidar_ros')

    # ── Launch arguments ──────────────────────────────────────────────────────
    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode', default_value='real',
        description='Hardware mode: mock, real',
        choices=['mock', 'gazebo', 'ignition', 'real']
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )
    use_rplidar_arg = DeclareLaunchArgument(
        'use_rplidar', default_value='true',
        description='Launch RPLidar'
    )
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop', default_value='true',
        description='Launch teleop'
    )
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', default_value='true',
        description='Launch EKF'
    )

    hardware_mode = LaunchConfiguration('hardware_mode')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    use_rplidar   = LaunchConfiguration('use_rplidar')
    use_teleop    = LaunchConfiguration('use_teleop')
    use_ekf       = LaunchConfiguration('use_ekf')

    # ── Robot description ─────────────────────────────────────────────────────
    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_description, 'urdf', 'manriix2.urdf.xacro'),
        ' hardware_mode:=', hardware_mode
    ])

    controller_config = os.path.join(
        pkg_hardware, 'config', 'manriix_controllers.yaml'
    )
    ekf_config = os.path.join(pkg_control, 'config', 'ekf.yaml')

    # ── Nodes ─────────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description_content, value_type=str
            ),
            'use_sim_time': use_sim_time,
            'use_tf_static': False,
        }]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(
                robot_description_content, value_type=str
            )},
            controller_config
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )]
    )

    controller_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'manriix_4ws_controller',
                '--controller-manager', '/controller_manager'
            ],
            output='screen',
        )]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_ekf)
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'rplidar_a2m12_launch.py')
        ),
        condition=IfCondition(use_rplidar)
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'teleop.launch.py')
        ),
        condition=IfCondition(use_teleop)
    )

    # ── Return ────────────────────────────────────────────────────────────────
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT',
            '[{severity}] [{name}]: {message}'
        ),

        LogInfo(msg="\n[hardware_only] Starting hardware stack WITHOUT cameras.\n"
                    "Run your perceptor stack in a second terminal.\n"),

        hardware_mode_arg,
        use_sim_time_arg,
        use_rplidar_arg,
        use_teleop_arg,
        use_ekf_arg,

        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        controller_spawner,
        ekf_node,
        rplidar_launch,
        teleop_launch,
    ])