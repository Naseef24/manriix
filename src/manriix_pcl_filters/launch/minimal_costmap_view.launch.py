#!/usr/bin/env python3
"""
Standalone Nav2 costmap viewer for PCL OAK obstacle clusters.

Assumes already running:
  1) OAK-D camera / depthai pipeline
  2) PCL obstacle clustering node:
       /oak/points_filtered
       /oak/ground_plane
       /oak/obstacles
       /oak/obstacle_clusters

This launch starts only:
  - static TF map -> base_footprint
  - static TF base_footprint -> oak-d-base-frame
  - standalone nav2_costmap_2d
  - lifecycle configure + activate

Actual OAK TF chain:
  oak-d-base-frame
    -> oak
    -> oak_right_camera_frame
    -> oak_right_camera_optical_frame

Costmap input:
  marking:  /oak/obstacle_clusters
  clearing: /oak/points_filtered
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('manriix_pcl_filters')

    costmap_params = os.path.join(
        pkg,
        'config',
        'oak_nav2_nav.yaml'
    )

    return LaunchDescription([

        LogInfo(msg=[
            '\n', '=' * 70, '\n',
            '[PCL OAK OBSTACLE COSTMAP VIEWER]\n',
            '\n',
            'Required nodes already running:\n',
            '  - OAK camera publishes /oak/points\n',
            '  - PCL node publishes /oak/obstacle_clusters\n',
            '  - PCL node publishes /oak/points_filtered\n',
            '\n',
            'This launch starts only standalone Nav2 costmap.\n',
            '\n',
            'Correct TF chain used here:\n',
            '  map -> base_footprint -> oak-d-base-frame -> oak -> oak_right_camera_optical_frame\n',
            '\n',
            'Costmap inputs:\n',
            '  Marking  -> /oak/obstacle_clusters\n',
            '  Clearing -> /oak/points_filtered\n',
            '\n',
            'View in Foxglove:\n',
            '  /costmap/costmap\n',
            '  /oak/obstacle_clusters\n',
            '  /oak/points_filtered\n',
            '=' * 70,
        ]),

        # ------------------------------------------------------------
        # Static TF: map -> base_footprint
        # Standalone costmap needs a global frame.
        # ------------------------------------------------------------
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'base_footprint',
            ],
            output='screen',
            name='static_tf_map_base',
        ),

        # ------------------------------------------------------------
        # Static TF: base_footprint -> oak-d-base-frame
        #
        # IMPORTANT:
        # Do NOT publish base_footprint -> oak.
        # The OAK driver already publishes:
        #   oak-d-base-frame -> oak
        #
        # Your PointCloud2 frame is:
        #   oak_right_camera_optical_frame
        #
        # Existing OAK chain:
        #   oak-d-base-frame -> oak -> oak_right_camera_frame
        #   -> oak_right_camera_optical_frame
        # ------------------------------------------------------------
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', '0.400',
                '--y', '-0.0375',
                '--z', '0.270',
                '--roll', '0.0',
                '--pitch', '-0.08',
                '--yaw', '0.0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'oak-d-base-frame',
            ],
            output='screen',
            name='static_tf_base_oak_base',
        ),

        # ------------------------------------------------------------
        # Standalone Nav2 costmap only.
        # ------------------------------------------------------------
        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg='[COSTMAP TEST] Launching standalone costmap...'),
                Node(
                    package='nav2_costmap_2d',
                    executable='nav2_costmap_2d',
                    name='costmap',
                    output='screen',
                    parameters=[costmap_params],
                ),
            ],
        ),

        # ------------------------------------------------------------
        # Lifecycle bring-up.
        # ------------------------------------------------------------
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg='[COSTMAP TEST] Configuring costmap lifecycle...'),
                ExecuteProcess(
                    cmd=[
                        'ros2', 'lifecycle', 'set',
                        '/costmap/costmap',
                        'configure'
                    ],
                    output='screen',
                ),
            ],
        ),

        TimerAction(
            period=6.0,
            actions=[
                LogInfo(msg='[COSTMAP TEST] Activating costmap lifecycle...'),
                ExecuteProcess(
                    cmd=[
                        'ros2', 'lifecycle', 'set',
                        '/costmap/costmap',
                        'activate'
                    ],
                    output='screen',
                ),
            ],
        ),

        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg=[
                    '\n[COSTMAP TEST] Ready.\n',
                    'Watch /costmap/costmap in Foxglove.\n',
                ]),
            ],
        ),
    ])