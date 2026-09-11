#!/usr/bin/env python3
"""
============================================================================
oak_pointcloud_bridged.launch.py

Includes stock depthai_ros_driver pointcloud.launch.py + adds QoS bridge node.

Result:
  /oak/points          — BEST_EFFORT (from OAK driver)
  /oak/points_reliable — RELIABLE (from bridge, ready for ground_segmentation)

USAGE:
  Terminal 1:
    ros2 launch manriix_description oak_pointcloud_bridged.launch.py

  Terminal 2 (after OAK is ready):
    ros2 launch ground_segmentation_ros2 ground_segmentation.launch.py \\
        pointcloud_topic:=/oak/points_reliable

File: ~/manriix2_ws/src/manriix_description/launch/oak_pointcloud_bridged.launch.py
============================================================================
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
    # ---- OAK-D Pro W via our own minimal launch ----
    # Avoids broken upstream rgbd_pcl/pointcloud launches (OpenCV remap crash
    # in image_proc::Rectify against this depthai-ros version).
    pkg_description = get_package_share_directory('manriix_description')
    oak_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'oak_xyz_only.launch.py')
        )
    )

    # ---- QoS Bridge: BEST_EFFORT → RELIABLE for /oak/points ----
    # Delay startup to ensure OAK is publishing before bridge subscribes
    qos_bridge = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg=[
                "\n", "=" * 60, "\n",
                "[QOS BRIDGE] Starting BEST_EFFORT → RELIABLE bridge\n",
                "  Subscribes: /oak/points (BEST_EFFORT)\n",
                "  Publishes:  /oak/points_reliable (RELIABLE)\n",
                "=" * 60,
            ]),
            Node(
                package='manriix_description',
                executable='qos_bridge.py',
                name='oak_points_qos_bridge',
                output='screen',
                parameters=[{
                    'input_topic': '/oak/points',
                    'output_topic': '/oak/points_reliable',
                }],
            ),
        ]
    )

    return LaunchDescription([
        oak_launch,
        qos_bridge,
    ])