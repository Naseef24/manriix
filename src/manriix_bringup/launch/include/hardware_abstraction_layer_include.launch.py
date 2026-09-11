#!/usr/bin/env python3
"""
================================================================================
Manriix Hardware Abstraction Layer Include

Purpose: bring up hardware-level components only.

Remote camera/Foxglove monitoring deliberately does NOT belong here.
ZED cameras, cuVSLAM, nvblox and monitoring remain in the perception /
visualization layers so hardware timing is isolated from remote UI traffic.
================================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('manriix_description')
    pkg_hardware = get_package_share_directory('manriix_hardware')
    pkg_control = get_package_share_directory('manriix_control')
    pkg_rplidar = get_package_share_directory('rplidar_ros')

    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode',
        default_value='real',
        choices=['real', 'mock', 'gazebo', 'ignition', 'simulation', 'rosbag'],
        description='Hardware mode (used by URDF xacro and conditional nodes).',
    )

    enable_rplidar_arg = DeclareLaunchArgument(
        'enable_rplidar',
        default_value='True',
        description='Launch RPLidar A2M12 2D laser scanner.',
    )

    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='True',
        description='Launch RC + joystick teleop.',
    )

    enable_ekf_arg = DeclareLaunchArgument(
        'enable_ekf',
        default_value='False',
        description=(
            'Launch EKF. Default False because cuVSLAM publishes both '
            'map->odom AND odom->base_footprint directly.'
        ),
    )

    enable_motor_diagnostics_arg = DeclareLaunchArgument(
        'enable_motor_diagnostics',
        default_value='True',
        description='Launch motor diagnostics node (real hardware only).',
    )

    enable_oak_voxel_costmap_arg = DeclareLaunchArgument(
        'enable_oak_voxel_costmap',
        default_value='True',
        description='Forwarded OAK perception enable flag.',
    )

    hardware_mode = LaunchConfiguration('hardware_mode')
    enable_rplidar = LaunchConfiguration('enable_rplidar')
    enable_teleop = LaunchConfiguration('enable_teleop')
    enable_ekf = LaunchConfiguration('enable_ekf')
    enable_motor_diagnostics = LaunchConfiguration('enable_motor_diagnostics')

    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_description, 'urdf', 'manriix3.urdf.xacro'),
        ' hardware_mode:=', hardware_mode,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description_content,
                value_type=str,
            ),
            'use_sim_time': False,
            'use_tf_static': False,
        }],
    )

    controller_config = os.path.join(
        pkg_hardware,
        'config',
        'manriix_controllers.yaml',
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {
                'robot_description': ParameterValue(
                    robot_description_content,
                    value_type=str,
                )
            },
            controller_config,
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
        )],
    )

    controller_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'manriix_4ws_controller',
                '--controller-manager',
                '/controller_manager',
            ],
            output='screen',
        )],
    )

    motor_diagnostics = Node(
        package='manriix_hardware',
        executable='motor_diagnostics_node.py',
        name='motor_diagnostics',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", hardware_mode, "' == 'real' and '",
            enable_motor_diagnostics, "' == 'True'",
        ])),
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_rplidar,
                'launch',
                'rplidar_a2m12_launch.py',
            )
        ),
        condition=IfCondition(enable_rplidar),
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'teleop.launch.py')
        ),
        condition=IfCondition(enable_teleop),
    )

    ekf_config = os.path.join(pkg_control, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': False}],
        condition=IfCondition(enable_ekf),
    )

    return LaunchDescription([
        hardware_mode_arg,
        enable_rplidar_arg,
        enable_teleop_arg,
        enable_ekf_arg,
        enable_motor_diagnostics_arg,
        enable_oak_voxel_costmap_arg,

        LogInfo(msg=[
            '\n', '=' * 70, '\n',
            '[HARDWARE LAYER] Bringing up:\n',
            '  Mode:               ', hardware_mode, '\n',
            '  robot_state_publisher: yes\n',
            '  ros2_control:       yes (4WS + joint_state_broadcaster)\n',
            '  Motor diagnostics:  ', enable_motor_diagnostics, '\n',
            '  RPLidar A2M12:      ', enable_rplidar, '\n',
            '  Teleop:             ', enable_teleop, '\n',
            '  EKF:                ', enable_ekf, ' (False=cuVSLAM owns TF)\n',
            '=' * 70,
        ]),

        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        controller_spawner,
        rplidar_launch,
        motor_diagnostics,
        teleop_launch,
        ekf_node,
    ])