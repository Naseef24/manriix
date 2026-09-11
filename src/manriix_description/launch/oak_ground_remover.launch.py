#!/usr/bin/env python3
"""
================================================================================
oak_ground_remover.launch.py

Standalone diagnostic launch for the oak_ground_remover_node.

Includes the existing oak_pointcloud_bridged.launch.py (OAK driver + qos_bridge)
to produce /oak/points_reliable, then starts the C++ RANSAC ground remover.

Outputs:
  - /oak/points_obstacles  : non-ground points only
  - /oak/points_ground     : ground points only (for viz)

Does NOT integrate with Nav2 or STVL. Pure measurement experiment.

USAGE:
  ros2 launch manriix_description oak_ground_remover.launch.py

File: ~/manriix2_ws/src/manriix_description/launch/oak_ground_remover.launch.py
================================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('manriix_description')

    # ---- Step 1: bring up OAK + qos_bridge (the existing launch) ----
    oak_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch',
                         'oak_pointcloud_bridged.launch.py')
        )
    )

    # ---- Step 2: start the RANSAC ground remover, delayed for OAK warmup ----
    # qos_bridge starts at t=8s in oak_pointcloud_bridged.launch.py.
    # We wait until t=12s to ensure /oak/points_reliable is alive.
    ground_remover = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg=[
                "\n", "=" * 70, "\n",
                "[OAK GROUND REMOVER] Starting RANSAC plane segmentation\n",
                "  Input:           /oak/points_reliable\n",
                "  Output (obs):    /oak/points_obstacles\n",
                "  Output (ground): /oak/points_ground\n",
                "  Stats logged every 5s\n",
                "=" * 70,
            ]),
            Node(
                package='manriix_description',
                executable='oak_ground_remover_node',
                name='oak_ground_remover',
                output='screen',
                parameters=[{
                    'input_topic': '/oak/points_reliable',
                    'obstacles_topic': '/oak/points_obstacles',
                    'ground_topic': '/oak/points_ground',
                    'publish_ground': True,

                    'voxel_leaf_size': 0.05,
                    'plane_distance_threshold': 0.03,
                    'max_iterations': 100,
                    'epsilon_angle_deg': 15.0,
                    'min_inliers': 200,

                    # Clustering tuning
                    'cluster_tolerance': 0.07,      # 10 cm grouping radius
                    'cluster_min_size': 80,         # was 30 — catch smaller objects
                    'cluster_max_size': 25000,
                    
                    # Axis along which plane normal must lie.
                    # For *_optical_frame:  (0, -1, 0)   floor normal points up = -Y in optical
                    # For ROS convention:    (0,  0, 1)  floor normal = +Z
                    'axis_x': 0.0,
                    'axis_y': -1.0,
                    'axis_z': 0.0,

                    'stats_log_period_sec': 5.0,
                }],
            ),
        ]
    )

    return LaunchDescription([
        oak_launch,
        ground_remover,
    ])