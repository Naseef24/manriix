#!/usr/bin/env python3
"""
Manriix SLAM + Navigation Launch File

3D Perception (default): SLAM Toolbox + Nav2 STVL + Collision Monitor
2D Fallback:             SLAM Toolbox + Nav2 (RPLidar only)

Usage:
  # Default 3D mode
  ros2 launch manriix_navigation slam_nav.launch.py

  # 2D fallback
  ros2 launch manriix_navigation slam_nav.launch.py use_3d_perception:=false

  # Visualization (run on workstation, not Jetson):
  #   rviz2 -d ~/manriix_rviz_configs/nav2_3d.rviz
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

    # ── Package directories ───────────────────────────────────────────────────
    nav_pkg          = get_package_share_directory('manriix_navigation')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # ── Config paths ──────────────────────────────────────────────────────────
    slam_toolbox_config  = os.path.join(nav_pkg, 'config', 'slam_toolbox_config.yaml')
    nav2_params_2d       = os.path.join(nav_pkg, 'config', 'nav2', 'nav2_params.yaml')
    nav2_params_3d       = os.path.join(nav_pkg, 'config', 'nav2', 'nav2_params_3d_full.yaml')
    rviz_config_2d       = os.path.join(nav_pkg, 'rviz', 'nav2.rviz')
    rviz_config_3d       = os.path.join(nav_pkg, 'rviz', 'nav2_3d.rviz')

    # ── Launch arguments ──────────────────────────────────────────────────────
    # All resolved at top — used as variables throughout
    use_sim_time       = LaunchConfiguration('use_sim_time')
    slam_params_file   = LaunchConfiguration('slam_params_file')
    nav2_params_file   = LaunchConfiguration('nav2_params_file')
    nav2_params_3d_file = LaunchConfiguration('nav2_params_3d_file')
    use_rviz           = LaunchConfiguration('use_rviz')
    use_3d_perception  = LaunchConfiguration('use_3d_perception')

    ld = LaunchDescription()

    # ── Environment ───────────────────────────────────────────────────────────
    ld.add_action(SetEnvironmentVariable('DISPLAY', ':0'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'))
    ld.add_action(SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'))

    # ── Declare arguments ─────────────────────────────────────────────────────
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'))

    ld.add_action(DeclareLaunchArgument(
        'slam_params_file', default_value=slam_toolbox_config,
        description='SLAM Toolbox parameters file'))

    ld.add_action(DeclareLaunchArgument(
        'nav2_params_file', default_value=nav2_params_2d,
        description='Nav2 parameters file (2D fallback)'))

    ld.add_action(DeclareLaunchArgument(
        'nav2_params_3d_file', default_value=nav2_params_3d,
        description='Nav2 parameters file (3D mode with STVL)'))

    ld.add_action(DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2 on this machine (prefer workstation)'))

    ld.add_action(DeclareLaunchArgument(
        'use_3d_perception', default_value='true',
        description='Enable 3D perception with STVL + ZED cameras'))

    # ── Mode info ─────────────────────────────────────────────────────────────
    ld.add_action(LogInfo(
        condition=UnlessCondition(use_3d_perception),
        msg='\n' + '='*60 + '\n'
            '[MANRIIX NAV] 2D MODE (RPLidar only)\n' + '='*60))

    ld.add_action(LogInfo(
        condition=IfCondition(use_3d_perception),
        msg='\n' + '='*60 + '\n'
            '[MANRIIX NAV] 3D MODE (RPLidar + ZED STVL + Collision Monitor)\n' +
            '='*60))

    # ── SLAM Toolbox ─────────────────────────────────────────────────────────
    # NOTE: No respawn — restarting SLAM loses the map intentionally.
    #       recovery_manager should alert if slam_toolbox disappears.
    ld.add_action(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    ))

    # ── RViz2 (optional — prefer running on workstation) ──────────────────────
    # FIXED: .lower() in (...) matches all IfCondition-truthy values
    ld.add_action(Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config_2d],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression([
            "'", use_rviz, "'.lower() in ('true','1','yes','on') and '",
            use_3d_perception, "'.lower() not in ('true','1','yes','on')",
        ])),
    ))

    ld.add_action(Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config_3d],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression([
            "'", use_rviz, "'.lower() in ('true','1','yes','on') and '",
            use_3d_perception, "'.lower() in ('true','1','yes','on')",
        ])),
    ))

    # ── Nav2 — 2D fallback ────────────────────────────────────────────────────
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_file,
            'autostart':    'true',
        }.items(),
        condition=UnlessCondition(use_3d_perception),
    ))

    # ── Nav2 — 3D mode (STVL) ────────────────────────────────────────────────
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_3d_file,   # respects CLI override
            'autostart':    'true',
        }.items(),
        condition=IfCondition(use_3d_perception),
    ))

    # ── Collision Monitor — 3D mode only ─────────────────────────────────────
    # FIXED: was using hardcoded nav2_params_3d_path (raw string).
    # LaunchConfiguration IS valid as a parameters file path — using it here
    # ensures collision_monitor and Nav2 always load the same config file.
    ld.add_action(Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params_3d_file],   # now consistent with nav2 3d stack
        condition=IfCondition(use_3d_perception),
    ))

    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision',
        output='screen',
        parameters=[{
            'use_sim_time':  use_sim_time,
            'autostart':     True,
            'node_names':    ['collision_monitor'],
            'bond_timeout':  4.0,
        }],
        condition=IfCondition(use_3d_perception),
    ))

    # ── Pose publisher ────────────────────────────────────────────────────────
    ld.add_action(Node(
        package='manriix_mission',
        executable='pose_publisher',
        name='pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    return ld