#!/usr/bin/env python3
"""
================================================================================
ground_consistency_test.launch.py

Standalone test harness for DFKI's nav2_ground_consistency_costmap_plugin.

What this DOES:
  - Brings up a single nav2_costmap_2d node ('test_costmap')
  - Loads ONLY ground_consistency_layer + inflation_layer
  - Activates lifecycle (configure → activate)
  - Publishes static TF chain: map → base_footprint → oak_d_pro_w_*

What this does NOT do:
  - Launch the OAK driver (run oak_ground_remover.launch.py first)
  - Launch any other Nav2 component (no planner, no controller, no BT)
  - Launch ZED / cuVSLAM / nvblox

USAGE:
  Terminal 1: ros2 launch manriix_description oak_ground_remover.launch.py
  Terminal 2: ros2 launch manriix_description ground_consistency_test.launch.py

Then in Foxglove or RViz, subscribe to:
  /costmap/costmap                    (the costmap)
  /costmap/voxel_marked_cloud         (debug viz, if enabled by plugin)
  /oak/points_ground                       (green)
  /oak/points_obstacles                    (red)

File: ~/manriix2_ws/src/manriix_description/launch/ground_consistency_test.launch.py
================================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('manriix_description')
    config_file = os.path.join(
        pkg_description, 'config', 'ground_consistency_test.yaml'
    )

    # -----------------------------------------------------------------
    # Static TF: minimal tree so the costmap has something to work with.
    # Since this test runs without the URDF being loaded, we publish a
    # fixed chain map → base_footprint → oak_d_pro_w_rgb_camera_optical_frame.
    # The OAK position uses the same offsets as the real URDF
    # (manriix2.urdf.xacro: cam_pos=0.40, -0.0375, -0.27 relative to base_link).
    # Here base_footprint == base_link for simplicity in the test.
    # -----------------------------------------------------------------
    static_tf_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base',
        arguments=[
            '0', '0', '0',              # x y z
            '0', '0', '0',              # roll pitch yaw
            'map', 'base_footprint',
        ],
        output='screen',
    )

    # OAK frame is at (0.40, -0.0375, 0.23) above base_footprint and rotated
    # for optical convention. We approximate the URDF chain with one static TF.
    # The rotation is the optical convention: -90° yaw then -90° roll.
    static_tf_base_to_oak = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_oak',
        arguments=[
            '0.40', '-0.0375', '0.23',         # x y z (matches URDF)
            '-1.5708', '0', '-1.5708',         # yaw pitch roll for optical
            'base_footprint',
            'oak_d_pro_w_rgb_camera_optical_frame',
        ],
        output='screen',
    )

    # -----------------------------------------------------------------
    # Standalone costmap node
    # -----------------------------------------------------------------
    # NOTE: nav2_costmap_2d standalone executable hardcodes its node name
    # to 'costmap' and ignores 'name=' or remaps. The YAML root must be
    # 'costmap: costmap: ros__parameters:' to match.
    test_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        output='screen',
        parameters=[config_file],
    )

    # -----------------------------------------------------------------
    # Lifecycle manager — autostarts the costmap (configure + activate)
    # -----------------------------------------------------------------
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_test_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['/costmap/costmap'],   # full ROS path: namespace/name
            'bond_timeout': 0.0,
        }],
    )

    # -----------------------------------------------------------------
    # Delayed startup banner
    # -----------------------------------------------------------------
    banner = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg=[
                '\n', '=' * 72, '\n',
                '[GROUND CONSISTENCY TEST] Standalone costmap test running\n',
                '  Subscribed:\n',
                '    /oak/points_ground         (RANSAC inliers)\n',
                '    /oak/points_obstacles      (RANSAC outliers)\n',
                '  Published:\n',
                '    /costmap/costmap      (OccupancyGrid for viz)\n',
                '  KPI log: /tmp/costmap_kpi_*.csv\n',
                '\n',
                '  PREREQ: oak_ground_remover.launch.py must already be running!\n',
                '=' * 72,
            ]),
        ],
    )

    return LaunchDescription([
        static_tf_map_to_base,
        static_tf_base_to_oak,
        test_costmap,
        lifecycle_mgr,
        banner,
    ])