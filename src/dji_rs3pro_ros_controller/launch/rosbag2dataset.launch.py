#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('dji_rs3pro_ros_controller')
    config_file = os.path.join(pkg_share, 'config', 'rosbag2dataset.yaml')
    
    return LaunchDescription([
        Node(
            package='dji_rs3pro_ros_controller',
            executable='rosbag2dataset_node.py',
            name='rosbag2dataset',
            output='screen',
            parameters=[config_file],
        ),
    ])
