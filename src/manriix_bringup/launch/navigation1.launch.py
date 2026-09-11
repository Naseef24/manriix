#!/usr/bin/env python3
"""
================================================================================
Manriix Navigation — Top-Level Entry Point (Carter-style architecture)

Modeled on NVIDIA's nova_carter_bringup/launch/navigation.launch.py.

Purpose: thin orchestrator. This file does THREE things only:
  1. Declare top-level arguments (enable_* flags, modes, configs)
  2. Include the workhorse (navigation_include.launch.py)
  3. Create the two composable containers (perception + Nav2-isolated)

All actual work happens in the sub-includes.

Architecture:
  navigation.launch.py (this file)
      │
      ├─→ launch/include/navigation_include.launch.py (workhorse)
      │      │
      │      ├─→ perceptor_include.launch.py
      │      │     ├─→ hardware_abstraction_layer_include.launch.py
      │      │     └─→ perceptor_general (zed, vslam, nvblox)
      │      │
      │      ├─→ oak_obstacle.launch.py (from manriix_oak_obstacles)
      │      │     └─→ on-chip edge detection + dense marking/clearing clouds
      │      │
      │      └─→ nav2_include.launch.py
      │
      ├─→ launch/include/visualization_include.launch.py (Foxglove)
      │
      ├─→ container: manriix_container (perception)
      └─→ container: navigation_container (Nav2 isolated)

USAGE:
  ros2 launch manriix_bringup navigation.launch.py \\
      hardware_mode:=real \\
      enable_navigation:=True \\
      enable_oak_obstacles:=True \\
      enable_nvblox_costmap:=True \\
      enable_2d_lidar_costmap:=True \\
      stereo_camera_configuration:=front_left_right_configuration \\
      use_foxglove:=True \\
      foxglove_layout:=manriix_navigation

OAK PIPELINE NOTES:
  - The new OAK obstacle node owns the OAK USB device end-to-end.
  - Previous RANSAC C++ node (oak_ground_remover_node) and DFKI plugin
    are NO LONGER LAUNCHED in production. The source code stays in
    the repo for reference / fallback via git.
  - To re-enable the old stack, revert this commit + the YAML.

File: ~/manriix2_ws/src/manriix_bringup/launch/navigation.launch.py
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    # ---------------------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------------------
    pkg_bringup = get_package_share_directory('manriix_bringup')

    # ---------------------------------------------------------------------
    # Top-level arguments
    # ---------------------------------------------------------------------
    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode',
        default_value='real',
        choices=['real', 'mock', 'gazebo', 'ignition', 'simulation', 'rosbag'],
        description='Hardware mode for URDF and sensor source selection.',
    )

    stereo_camera_configuration_arg = DeclareLaunchArgument(
        'stereo_camera_configuration',
        default_value='front_left_right_configuration',
        choices=[
            'no_cameras',
            'front_configuration',
            'front_left_right_configuration',
            'front_only_vslam',
        ],
        description='Stereo camera setup profile (matches Carter pattern).',
    )

    enable_navigation_arg = DeclareLaunchArgument(
        'enable_navigation', default_value='True',
        description='Launch Nav2 stack.',
    )
    enable_2d_lidar_costmap_arg = DeclareLaunchArgument(
        'enable_2d_lidar_costmap', default_value='True',
        description='Enable RPLidar 2D scan obstacle layer in costmap.',
    )
    enable_oak_obstacles_arg = DeclareLaunchArgument(
        'enable_oak_obstacles', default_value='True',
        description=(
            'Enable OAK-D edge-based obstacle detection (manriix_oak_obstacles). '
            'Replaces previous RANSAC + DFKI plugin. '
            'When True: oak_obstacle_node owns the OAK device and publishes '
            '/oak_obstacles/marking_points + /oak_obstacles/clearing_points '
            'to feed an ObstacleLayer in local_costmap.'
        ),
    )

    # enable_oak_obstacles_arg = DeclareLaunchArgument(
    #     'enable_oak_obstacles', default_value='True',
    #     description=(
    #         'paper: Leveraging Stereo-Camera Data for Real-Time Dynamic Obstacle Detection and Tracking'
    #     ),
    # )    
    
    enable_nvblox_costmap_arg = DeclareLaunchArgument(
        'enable_nvblox_costmap', default_value='True',
        description='Enable nvblox ESDF costmap layer.',
    )
    enable_wheel_odometry_arg = DeclareLaunchArgument(
        'enable_wheel_odometry', default_value='False',
        description=(
            'Use wheel odometry (/odom). Default False — cuVSLAM is primary '
            'odometry source. Set True to override cuVSLAM with wheel odom.'
        ),
    )

    map_yaml_path_arg = DeclareLaunchArgument(
        'map_yaml_path', default_value='None',
        description='Path to map.yaml. None = local navigation (no global map).',
    )

    enable_battery_monitor_arg = DeclareLaunchArgument(
        'enable_battery_monitor', default_value='True',
        description='Run JK BMS reader node, publishes /main_battery/state.',
    )

    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove', default_value='True',
        description='Launch foxglove_bridge for remote visualization.',
    )
    foxglove_layout_arg = DeclareLaunchArgument(
        'foxglove_layout', default_value='manriix_navigation',
        choices=['manriix_navigation', 'manriix_perception', 'manriix_debug'],
        description='Foxglove layout file to suggest.',
    )

    # ---------------------------------------------------------------------
    # Capture all configs for forwarding
    # ---------------------------------------------------------------------
    hardware_mode = LaunchConfiguration('hardware_mode')
    stereo_camera_configuration = LaunchConfiguration('stereo_camera_configuration')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_2d_lidar_costmap = LaunchConfiguration('enable_2d_lidar_costmap')
    enable_oak_obstacles = LaunchConfiguration('enable_oak_obstacles')
    enable_nvblox_costmap = LaunchConfiguration('enable_nvblox_costmap')
    enable_wheel_odometry = LaunchConfiguration('enable_wheel_odometry')
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    enable_battery_monitor = LaunchConfiguration('enable_battery_monitor')
    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_layout = LaunchConfiguration('foxglove_layout')

    # ---------------------------------------------------------------------
    # Includes
    # ---------------------------------------------------------------------
    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'include',
                         'navigation_include.launch.py')
        ),
        launch_arguments={
            'hardware_mode': hardware_mode,
            'stereo_camera_configuration': stereo_camera_configuration,
            'enable_navigation': enable_navigation,
            'enable_2d_lidar_costmap': enable_2d_lidar_costmap,
            'enable_oak_obstacles': enable_oak_obstacles,
            'enable_nvblox_costmap': enable_nvblox_costmap,
            'enable_wheel_odometry': enable_wheel_odometry,
            'map_yaml_path': map_yaml_path,
        }.items(),
    )

    battery_monitor_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'jk_bms.launch.py')
        ),
        condition=IfCondition(enable_battery_monitor),
    )

    visualization_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'include',
                         'visualization_include.launch.py')
        ),
        launch_arguments={
            'use_foxglove': use_foxglove,
            'foxglove_layout': foxglove_layout,
        }.items(),
    )

    # ---------------------------------------------------------------------
    # Component containers (Carter pattern: two containers)
    # ---------------------------------------------------------------------
    perception_container = ComposableNodeContainer(
        name='manriix_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        prefix='nice -n -10',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    navigation_container = ComposableNodeContainer(
        name='navigation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # ---------------------------------------------------------------------
    # Assemble
    # ---------------------------------------------------------------------
    return LaunchDescription([
        SetEnvironmentVariable('DISPLAY', ':0'),
        SetEnvironmentVariable('XAUTHORITY', '/run/user/1000/gdm/Xauthority'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT',
                               '[{severity}] [{name}]: {message}'),

        hardware_mode_arg,
        stereo_camera_configuration_arg,
        enable_navigation_arg,
        enable_2d_lidar_costmap_arg,
        enable_oak_obstacles_arg,
        enable_nvblox_costmap_arg,
        enable_wheel_odometry_arg,
        enable_battery_monitor_arg,
        map_yaml_path_arg,
        use_foxglove_arg,
        foxglove_layout_arg,

        LogInfo(msg=[
            '\n', '=' * 70, '\n',
            '[MANRIIX NAVIGATION — Carter-style architecture]\n',
            '  Stereo config: ', stereo_camera_configuration, '\n',
            '  Hardware mode: ', hardware_mode, '\n',
            '  Navigation enabled: ', enable_navigation, '\n',
            '  Costmap layers:\n',
            '    nvblox:           ', enable_nvblox_costmap, '\n',
            '    OAK obstacles:    ', enable_oak_obstacles, '\n',
            '    2D LiDAR scan:    ', enable_2d_lidar_costmap, '\n',
            '  Foxglove: ', use_foxglove, ' (layout: ', foxglove_layout, ')\n',
            '=' * 70,
        ]),

        navigation_include,
        battery_monitor_include,
        visualization_include,

        perception_container,
        navigation_container,
    ])