#!/usr/bin/env python3
"""
================================================================================
Manriix Visualization Include — Foxglove bridge + layout suggestion

Modeled on:
  isaac_ros_perceptor_bringup/launch/tools/visualization.launch.py
  (Carter uses this with a use_foxglove_whitelist flag)

Purpose:
  - Launch foxglove_bridge with optional whitelist (Carter-style)
  - Print Foxglove layout suggestion to console for the user to load

Foxglove layouts shipped with manriix_navigation:
  - manriix_navigation   (clean view: map, robot, nav2 plan, costmap)
  - manriix_perception   (camera images, point clouds, TF debug)
  - manriix_debug        (everything — heavy on bandwidth)

These layout JSON files live in:
  ~/manriix2_ws/src/manriix_navigation/config/foxglove_layouts/

USAGE (via top-level navigation.launch.py):
  ros2 launch manriix_bringup navigation.launch.py \\
      use_foxglove:=True \\
      foxglove_layout:=manriix_navigation

Connect from a remote machine:
  Foxglove Studio → Open Connection → ws://<jetson-ip>:8765
  Then: Layout → Import from file → select the .json from layout dir.

File: ~/manriix2_ws/src/manriix_bringup/launch/include/visualization_include.launch.py
================================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_navigation = get_package_share_directory('manriix_navigation')

    # -----------------------------------------------------------------
    # Arguments
    # -----------------------------------------------------------------
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove', default_value='True',
        description='Launch foxglove_bridge.',
    )

    foxglove_layout_arg = DeclareLaunchArgument(
        'foxglove_layout', default_value='manriix_navigation',
        choices=['manriix_navigation', 'manriix_perception', 'manriix_debug'],
        description='Foxglove layout to suggest to the user.',
    )

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port', default_value='8765',
        description='WebSocket port for foxglove_bridge.',
    )

    use_foxglove_whitelist_arg = DeclareLaunchArgument(
        'use_foxglove_whitelist', default_value='True',
        description=(
            'Carter-style: True = use curated topic whitelist (fast, clean). '
            'False = show all topics (heavy bandwidth).'
        ),
    )

    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_layout = LaunchConfiguration('foxglove_layout')
    foxglove_port = LaunchConfiguration('foxglove_port')
    use_foxglove_whitelist = LaunchConfiguration('use_foxglove_whitelist')

    # -----------------------------------------------------------------
    # Whitelisted topics — Carter-style
    #
    # If use_foxglove_whitelist is True, only these topic patterns are
    # exposed (as regex). Saves bandwidth and keeps the UI focused.
    #
    # The lists below are tuned for Manriix:
    #   - Nav2 essentials
    #   - Costmap layers (local + global)
    #   - Robot TF + odometry
    #   - Nvblox visualizations
    #   - Selected camera images (front rect only, not depth)
    # -----------------------------------------------------------------
    whitelisted_topics = [
        # Nav2 core
        '/tf',
        '/tf_static',
        '/clock',
        '/cmd_vel',
        '/cmd_vel_safe',
        '/cmd_vel_smoothed',

        # Costmaps
        '/local_costmap/costmap',
        '/local_costmap/costmap_updates',
        '/local_costmap/published_footprint',
        '/global_costmap/costmap',
        '/global_costmap/costmap_updates',

        # Planning + behavior
        '/plan',
        '/plan_smoothed',
        '/received_global_plan',
        '/behavior_server/transition_event',
        '/goal_pose',
        '/initialpose',

        # Robot state
        '/odom',
        '/visual_slam/tracking/odometry',
        '/visual_slam/tracking/vo_pose',
        '/visual_slam/visualization/.*',
        '/robot_description',
        '/joint_states',

        # Nvblox
        # '/nvblox_node/static_map_slice',
        '/nvblox_node/static_esdf_pointcloud',
        '/nvblox_node/static_occupancy',
        '/nvblox_node/mesh',
        '/nvblox_node/dynamic_occupancy_layer',
        # '/nvblox_node/pessimistic_static_map_slice',
        '/nvblox_node/combined_occupancy_grid',
        '/nvblox_node/.*',

        # Sensors (light)
        '/scan',
        # '/zedx_front/zed_node/left/image_rect_color/compressed',

        # OAK edge-based obstacle detection (manriix_oak_obstacles)
        # Replaces previous /oak/* depthai_ros_driver topics.
        # '/oak_obstacles/image_overlay/compressed',
        # '/oak_obstacles/detections',
        # '/oak_obstacles/marking_points',
        # '/oak_obstacles/clearing_points',
        # '/oak/points',
        '/oak/points_ground',
        '/oak/points_obstacles',
        # '/oak/points_reliable',

        # '/oak_obstacles/floor_plane',
        # '/oak_obstacles/.*',

        # Collision monitor
        '/polygon_stop',
        '/polygon_slowdown',
        '/collision_monitor_state',

        # Diagnostics
        '/diagnostics',
        '/rosout',
        '/parameter_events',

        # Battery monitor (JK BMS)
        # '/main_battery/state',
        # '/main_battery/.*',

        '/zone_manager/.*',

        '/manriix/classification_markers',
        '/manriix/cluster_markers',
        '/manriix/cluster_tracks',
        '/manriix/cluster_tracks_classified',
        '/manriix/cluster_tracks_fused',
        '/manriix/detection_image',
        '/manriix/fusion_markers',
        '/manriix/grid/aggregated',
        '/manriix/grid/dynamic',
        '/manriix/grid/static',
        '/manriix/grid/uncertain',
        '/manriix/motion_markers',
        '/manriix/oak_obstacles_cloud',
        '/manriix/objects',
        '/manriix/obstacle_cloud',
        '/manriix/person_boxes',
        '/manriix/person_markers',
        '/manriix/points_dense',
        '/manriix/points_filtered',
        '/manriix/points_labeled',
        # '/manriix_4ws_controller/cmd_vel',
        # '/manriix_4ws_controller/odom',
        # '/manriix_4ws_controller/transition_event',
    ]

    # -----------------------------------------------------------------
    # Foxglove bridge node — with whitelist
    # -----------------------------------------------------------------
    foxglove_bridge_whitelisted = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': foxglove_port,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': whitelisted_topics,
            'param_whitelist': ['.*'],
            'service_whitelist': ['.*'],
            'client_topic_whitelist': ['.*'],
            'min_qos_depth': 1,
            'max_qos_depth': 10,
            'num_threads': 0,
            'send_buffer_limit': 10000000,
            'capabilities': [
                'clientPublish',
                'parameters',
                'parametersSubscribe',
                'services',
                'connectionGraph',
                'assets',
            ],
        }],
        condition=IfCondition(
            # Both must be True
            LaunchConfiguration('use_foxglove')
        ),
    )

    # -----------------------------------------------------------------
    # Layout file path (for user reference — Foxglove can't auto-load
    # remote layouts, but we print the path)
    # -----------------------------------------------------------------
    layout_path = PathJoinSubstitution([
        pkg_navigation, 'config', 'foxglove_layouts',
        # Will resolve to e.g. 'manriix_navigation.json'
        LaunchConfiguration('foxglove_layout'),
    ])

    # -----------------------------------------------------------------
    # Assemble
    # -----------------------------------------------------------------
    return LaunchDescription([
        use_foxglove_arg,
        foxglove_layout_arg,
        foxglove_port_arg,
        use_foxglove_whitelist_arg,

        LogInfo(
            msg=[
                '\n', '=' * 70, '\n',
                '[VISUALIZATION] Foxglove Bridge\n',
                '  Port:        ', foxglove_port, '\n',
                '  Whitelist:   ', use_foxglove_whitelist, ' (Carter-style)\n',
                '  Layout file: ', layout_path, '.json\n',
                '\n',
                '  To connect from a remote machine:\n',
                '    1. Open Foxglove Studio\n',
                '    2. Open Connection → ws://<jetson-ip>:', foxglove_port, '\n',
                '    3. Layout menu → Import from file → select the .json above\n',
                '=' * 70,
            ],
            condition=IfCondition(use_foxglove),
        ),

        foxglove_bridge_whitelisted,
    ])