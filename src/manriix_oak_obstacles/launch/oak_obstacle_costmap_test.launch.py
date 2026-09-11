#!/usr/bin/env python3
"""
Standalone test harness for OAK obstacle detector + Nav2 costmap.

Pipeline:
  oak_obstacle_node   (detections from OAK)
       ↓ /oak_obstacles/detections
  detection_to_pointcloud_node
       ↓ /oak_obstacles/points
  nav2_costmap_2d (standalone)  ← uses ObstacleLayer
       ↓ /costmap/costmap

Plus static TF chain so the standalone costmap has a valid frame tree:
  map → base_footprint → oak_d_pro_w_rgb_camera_optical_frame

NO main navigation stack required. NO other launches conflict.
View results in Foxglove or RViz.

Usage:
  ros2 launch manriix_oak_obstacles oak_obstacle_costmap_test.launch.py

Topics to watch:
  /oak_obstacles/detections
  /oak_obstacles/image_overlay/compressed
  /oak_obstacles/points
  /costmap/costmap
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, TimerAction, ExecuteProcess)
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('manriix_oak_obstacles')
    detector_params = os.path.join(
        pkg, 'config', 'oak_obstacle_params.yaml')
    costmap_params = os.path.join(
        pkg, 'config', 'oak_obstacle_costmap_test.yaml')

    return LaunchDescription([
        LogInfo(msg=[
            '\n', '=' * 70, '\n',
            '[OAK OBSTACLE COSTMAP TEST HARNESS]\n',
            '  1) OAK detector publishes /oak_obstacles/detections\n',
            '  2) Bridge converts to /oak_obstacles/points (PointCloud2)\n',
            '  3) Standalone costmap_2d consumes the cloud\n',
            '  4) /costmap/costmap is published for viewing\n',
            '\n',
            '  Static TF chain:\n',
            '    map → base_footprint → oak_d_pro_w_rgb_camera_optical_frame\n',
            '\n',
            '  View in Foxglove or RViz:\n',
            '    /costmap/costmap                (Map panel)\n',
            '    /oak_obstacles/image_overlay/compressed   (Image)\n',
            '    /oak_obstacles/detections       (3D panel)\n',
            '    /oak_obstacles/points           (PointCloud panel)\n',
            '=' * 70,
        ]),

        # ── Static TFs published as separate background processes ──
        # We use ExecuteProcess (not Node) because multiple Node entries
        # for static_transform_publisher silently overwrite each other
        # in /tf_static. Spawning them as separate processes with
        # distinct ROS_DOMAIN_ID would also work, but this is simpler.

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'map',
                '--child-frame-id', 'base_footprint',
            ],
            output='screen',
            name='static_tf_map_base',
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', '0.400',
                '--y', '-0.037',
                '--z', '0.230',
                '--roll', '-1.5708',
                '--pitch', '0',
                '--yaw', '-1.5708',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'oak_d_pro_w_rgb_camera_optical_frame',
            ],
            output='screen',
            name='static_tf_base_oak',
        ),

        # ── OAK detector node ──
        Node(
            package='manriix_oak_obstacles',
            executable='oak_obstacle_node',
            name='oak_obstacle_node',
            output='screen',
            parameters=[
                detector_params,
                {'log_profile': False},
            ],
        ),

        # ── Detection → PointCloud2 bridge ──
        Node(
            package='manriix_oak_obstacles',
            executable='detection_to_pointcloud_node',
            name='detection_to_pointcloud_node',
            output='screen',
            parameters=[
                {'samples_per_box': 50},
            ],
        ),

        # ── Standalone Nav2 costmap (delayed 4s to ensure detections flow) ──
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg='[TEST] Launching standalone costmap...'),
                Node(
                    package='nav2_costmap_2d',
                    executable='nav2_costmap_2d',
                    name='costmap',
                    output='screen',
                    parameters=[costmap_params],
                ),
            ],
        ),

        # ── Lifecycle bring-up: configure + activate costmap (delayed 6s) ──
        TimerAction(
            period=6.0,
            actions=[
                LogInfo(msg='[TEST] Activating costmap lifecycle...'),
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set',
                         '/costmap/costmap', 'configure'],
                    output='screen',
                ),
            ],
        ),
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set',
                         '/costmap/costmap', 'activate'],
                    output='screen',
                ),
                LogInfo(msg=[
                    '\n[TEST] Setup complete. '
                    'Watch /costmap/costmap in Foxglove/RViz.',
                ]),
            ],
        ),
    ])