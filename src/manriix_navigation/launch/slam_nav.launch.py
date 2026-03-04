#!/usr/bin/env python3
"""
Manriix SLAM + Navigation Launch File
Combines SLAM Toolbox (mapping) with Nav2 (navigation)
Use for exploration - navigate while building map
Nav2 Goal Pose WORKS in this mode!

3D Perception Options:
  use_3d_perception:=false  - Default, uses only RPLidar (2D mode)
  use_3d_perception:=true   - Enables STVL with ZED cameras for 3D obstacle detection

Collision Monitor:
  Based on official Nav2 documentation for ROS2 Humble:
  https://docs.nav2.org/tutorials/docs/using_collision_monitor.html
  
  The collision_monitor node is launched separately with its own lifecycle_manager
  as it's not included in navigation_launch.py for Humble.

Usage:
   # Default 3D mode (recommended)
   ros2 launch manriix_navigation slam_nav.launch.py

   # 2D fallback (RPLidar only, no ZED cameras)
   ros2 launch manriix_navigation slam_nav.launch.py use_3d_perception:=false

   # Visualization runs on WORKSTATION (not Jetson):
   #   manriix2_jetson && rviz2 -d ~/manriix_rviz_configs/nav2_3d.rviz

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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    
    # ---------------------------------------------------------------------------
    # Package directories
    # ---------------------------------------------------------------------------
    nav_pkg = get_package_share_directory('manriix_navigation')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # ---------------------------------------------------------------------------
    # Config paths
    # ---------------------------------------------------------------------------
    slam_toolbox_config = os.path.join(
        nav_pkg, 'config', 'slam_toolbox_config.yaml'
    )
    nav2_params_2d = os.path.join(
        nav_pkg, 'config', 'nav2', 'nav2_params.yaml'
    )
    # Resolved Python string — needed by collision_monitor (cannot use LaunchConfig)
    nav2_params_3d_path = os.path.join(
        nav_pkg, 'config', 'nav2', 'nav2_params_3d_full.yaml'
    )
    rviz_config_2d = os.path.join(nav_pkg, 'rviz', 'nav2.rviz')
    rviz_config_3d = os.path.join(nav_pkg, 'rviz', 'nav2_3d.rviz')

    
    # 3D params file path (resolved for collision monitor)
    nav2_params_3d_path = os.path.join(nav_pkg, 'config', 'nav2', 'nav2_params_3d_full.yaml')

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_toolbox_config,
        description='Full path to SLAM Toolbox parameters file',
    )

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=nav2_params_2d,
        description='Full path to Nav2 parameters file (2D fallback)',
    )

    declare_nav2_params_3d_file_cmd = DeclareLaunchArgument(
        'nav2_params_3d_file',
        default_value=nav2_params_3d_path,
        description='Full path to Nav2 parameters file (3D mode with STVL)',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 on this machine (prefer workstation instead)',
    )

    declare_use_3d_perception_cmd = DeclareLaunchArgument(
        'use_3d_perception',
        default_value='true',
        description='Enable 3D perception with STVL + ZED cameras (default: true)',
    )

    # ---------------------------------------------------------------------------
    # Launch configurations
    # ---------------------------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_params_3d_file = LaunchConfiguration('nav2_params_3d_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_3d_perception = LaunchConfiguration('use_3d_perception')

    # ---------------------------------------------------------------------------
    # Info messages
    # ---------------------------------------------------------------------------
    log_2d_mode_cmd = LogInfo(
        condition=UnlessCondition(use_3d_perception),
        msg='\n' + '=' * 60 + '\n'
            '[MANRIIX NAV] Starting in 2D MODE (fallback)\n'
            '  - RPLidar obstacle detection only\n'
            '=' * 60,
    )

    log_3d_mode_cmd = LogInfo(
        condition=IfCondition(use_3d_perception),
        msg='\n' + '=' * 60 + '\n'
            '[MANRIIX NAV] Starting in 3D MODE\n'
            '  - RPLidar + ZED STVL enabled\n'
            '  - Collision Monitor enabled\n'
            '  - Height range: 0.3m - 1.8m\n'
            '=' * 60,
    )

    # ---------------------------------------------------------------------------
    # SLAM Toolbox — provides /map topic, map→odom transform
    # Used in both 2D and 3D modes (2D map needed for global planning)
    # ---------------------------------------------------------------------------
    start_slam_toolbox_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ---------------------------------------------------------------------------
    # Nav2 stack — 2D fallback (RPLidar only)
    # ---------------------------------------------------------------------------
    start_nav2_2d_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items(),
        condition=UnlessCondition(use_3d_perception),
    )

    # ---------------------------------------------------------------------------
    # Nav2 stack — 3D mode (STVL + ZED cameras)
    # ---------------------------------------------------------------------------
    start_nav2_3d_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_3d_file,
            'autostart': 'true',
        }.items(),
        condition=IfCondition(use_3d_perception),
    )

    # ---------------------------------------------------------------------------
    # Collision Monitor — 3D mode only
    #
    # collision_monitor is NOT included in navigation_launch.py for Humble.
    # It needs its own lifecycle_manager (same pattern as localization_launch.py).
    # ---------------------------------------------------------------------------
    start_collision_monitor_cmd = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params_3d_path],
        condition=IfCondition(use_3d_perception),
    )

    start_collision_lifecycle_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['collision_monitor'],
            'bond_timeout': 4.0,
        }],
        condition=IfCondition(use_3d_perception),
    )


    # ---------------------------------------------------------------------------
    # RViz2 (optional — prefer running on workstation instead)
    # ---------------------------------------------------------------------------
    start_rviz_2d_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_2d],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(
            PythonExpression([
                "'", use_rviz, "' == 'true' and '",
                use_3d_perception, "' == 'false'",
            ])
        ),
    )

    start_rviz_3d_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_3d],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(
            PythonExpression([
                "'", use_rviz, "' == 'true' and '",
                use_3d_perception, "' == 'true'",
            ])
        ),
    )

    # ---------------------------------------------------------------------------
    # Pose publisher — publishes robot pose in map frame for web interface
    # Converts TF (map→base_link) to topic /robot_pose_map
    # ---------------------------------------------------------------------------
    start_pose_publisher_cmd = Node(
        package='manriix_mission',
        executable='pose_publisher',
        name='pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---------------------------------------------------------------------------
    # Compose LaunchDescription
    # ---------------------------------------------------------------------------
    ld = LaunchDescription()

    # Environment
    ld.add_action(SetEnvironmentVariable('DISPLAY', ':0'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'))
    ld.add_action(SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT',
        '[{severity}] [{name}]: {message}',
    ))

    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_nav2_params_3d_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_3d_perception_cmd)

    # Info
    ld.add_action(log_2d_mode_cmd)
    ld.add_action(log_3d_mode_cmd)

    # SLAM
    ld.add_action(start_slam_toolbox_cmd)

    # RViz (optional, prefer workstation)
    ld.add_action(start_rviz_2d_cmd)
    ld.add_action(start_rviz_3d_cmd)

    # Nav2
    ld.add_action(start_nav2_2d_cmd)
    ld.add_action(start_nav2_3d_cmd)

    # Collision Monitor (3D only)
    ld.add_action(start_collision_monitor_cmd)
    ld.add_action(start_collision_lifecycle_cmd)

    # Pose publisher
    ld.add_action(start_pose_publisher_cmd)

    return ld