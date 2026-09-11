#!/usr/bin/env python3
"""
================================================================================
Manriix Nav2 Include — Wraps Nav2 lifecycle bringup

Modeled on:
  nova_carter/nova_carter_navigation/launch/navigation.launch.py
  (Carter's Nav2 entry point inside their navigation package)

Purpose: bring up Nav2 lifecycle nodes with the right params file.

What this launches:
  1. nav2_bringup/launch/navigation_launch.py (standard Nav2 stack):
     - controller_server
     - planner_server
     - smoother_server
     - behavior_server
     - bt_navigator
     - waypoint_follower
     - velocity_smoother
     - collision_monitor (configured in our YAML)
     - lifecycle_manager_navigation
  2. collision_monitor (explicit node, configured in nav2_params_carter.yaml)   
  3. map_server (if map_yaml_path is provided)
  4. amcl (if map_yaml_path is provided and visual_localization is disabled)

What this does NOT launch:
  - Perception (in perceptor_include.launch.py)
  - Hardware drivers (in hardware_abstraction_layer_include.launch.py)
  - Plugin list configuration (set in navigation_include.launch.py)

USAGE (via navigation_include.launch.py):
  Not normally launched directly.

File: ~/manriix2_ws/src/manriix_bringup/launch/include/nav2_include.launch.py
================================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    # -----------------------------------------------------------------
    # Package paths
    # -----------------------------------------------------------------
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_navigation = get_package_share_directory('manriix_navigation')

    # -----------------------------------------------------------------
    # Arguments
    # -----------------------------------------------------------------
    navigation_parameters_path_arg = DeclareLaunchArgument(
        'navigation_parameters_path',
        default_value=os.path.join(
            pkg_navigation, 'config', 'nav2', 'nav2_params_carter.yaml'
        ),
        description='Nav2 params YAML.',
    )

    map_yaml_path_arg = DeclareLaunchArgument(
        'map_yaml_path', default_value='None',
        description='Path to map.yaml. None = no global map, local navigation only.',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation time.',
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically start Nav2 lifecycle.',
    )

    navigation_parameters_path = LaunchConfiguration('navigation_parameters_path')
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # -----------------------------------------------------------------
    # Condition: only launch map_server + amcl if a real map is given
    # -----------------------------------------------------------------
    map_provided = PythonExpression([
        "'", map_yaml_path, "' != 'None' and '", map_yaml_path, "' != ''"
    ])

    # -----------------------------------------------------------------
    # Nav2 bringup (the standard Nav2 stack)
    # -----------------------------------------------------------------
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': navigation_parameters_path,
            'autostart': autostart,
            # IMPORTANT: tell nav2_bringup to use our isolated container.
            # If left default ('navigation_container'), it matches the one
            # created in navigation.launch.py (Carter pattern). If you want
            # Nav2 in its own process, set use_composition:='True' and
            # container_name accordingly.
            'use_composition': 'False',
        }.items(),
    )

    # -----------------------------------------------------------------
    # Explicit Collision Monitor node (using your YAML section)
    # -----------------------------------------------------------------
    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[navigation_parameters_path],
        # No extra condition: if Nav2 is enabled, we always want safety.
    )

    # Lifecycle manager dedicated to collision_monitor
    lifecycle_manager_collision = Node(
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
    )
    # -----------------------------------------------------------------
    # Map server (only if a map is provided)
    # -----------------------------------------------------------------
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_path,
            'topic_name': 'map',
            'frame_id': 'map',
        }],
        condition=IfCondition(map_provided),
    )

    # -----------------------------------------------------------------
    # AMCL (only if map provided — currently disabled by default since
    # cuVSLAM provides map->odom transform directly. Enable later if you
    # add LiDAR-based localization for global maps.)
    # -----------------------------------------------------------------
    enable_amcl_arg = DeclareLaunchArgument(
        'enable_amcl', default_value='False',
        description='Launch AMCL (LiDAR localization). Default False because '
                    'cuVSLAM publishes map->odom directly.',
    )
    enable_amcl = LaunchConfiguration('enable_amcl')

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[navigation_parameters_path],
        condition=IfCondition(PythonExpression([
            "'", enable_amcl, "' == 'True' and '",
            map_yaml_path, "' != 'None'"
        ])),
    )

    # -----------------------------------------------------------------
    # Lifecycle manager for map_server + amcl (if either is enabled)
    # -----------------------------------------------------------------
    lifecycle_nodes_localization = []
    # We won't conditionally add to the list — instead use a separate
    # lifecycle_manager only when map is provided.
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
        condition=IfCondition(map_provided),
    )

    # -----------------------------------------------------------------
    # Assemble
    # -----------------------------------------------------------------
    return LaunchDescription([
        # Arguments
        navigation_parameters_path_arg,
        map_yaml_path_arg,
        use_sim_time_arg,
        autostart_arg,
        enable_amcl_arg,

        # Banner
        LogInfo(msg=[
            '\n', '=' * 70, '\n',
            '[NAV2 INCLUDE] Launching Nav2 stack\n',
            '  Params:    ', navigation_parameters_path, '\n',
            '  Map:       ', map_yaml_path, '\n',
            '  Autostart: ', autostart, '\n',
            '  AMCL:      ', enable_amcl, '\n',
            '=' * 70,
        ]),

        # Nav2 lifecycle stack
        nav2_bringup_launch,

        # Safety: explicit collision monitor node using nav2_params_carter.yaml
        collision_monitor_node,
        lifecycle_manager_collision,

        # Localization (only if map provided)
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
    ])