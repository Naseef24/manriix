#!/usr/bin/env python3
"""
Launch file for manriix_oak_obstacles.

Usage:
  ros2 launch manriix_oak_obstacles oak_obstacle.launch.py
  ros2 launch manriix_oak_obstacles oak_obstacle.launch.py \
       log_profile:=true publish_mask:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('manriix_oak_obstacles')
    default_params = os.path.join(pkg_share, 'config',
                                  'oak_obstacle_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to YAML params file')

    log_profile_arg = DeclareLaunchArgument(
        'log_profile', default_value='false',
        description='Print per-frame timing to console')

    publish_mask_arg = DeclareLaunchArgument(
        'publish_mask', default_value='false',
        description='Publish raw obstacle mask (bandwidth heavy)')

    params_file = LaunchConfiguration('params_file')
    log_profile = LaunchConfiguration('log_profile')
    publish_mask = LaunchConfiguration('publish_mask')

    return LaunchDescription([
        params_file_arg,
        log_profile_arg,
        publish_mask_arg,

        LogInfo(msg=[
            '\n', '=' * 60, '\n',
            '[MANRIIX OAK OBSTACLES]\n',
            '  Edge detection on OAK Myriad X (chip-side)\n',
            '  Detections published to /oak_obstacles/detections\n',
            '  Image overlay on /oak_obstacles/image_overlay/compressed\n',
            '=' * 60,
        ]),

        Node(
            package='manriix_oak_obstacles',
            executable='oak_obstacle_node',
            name='oak_obstacle_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'log_profile': log_profile,
                    'publish_mask': publish_mask,
                },
            ],
            respawn=True,
            respawn_delay=3.0,
        ),
    ])