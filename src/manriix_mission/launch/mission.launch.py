#!/usr/bin/env python3
"""
Manriix Mission Launch File
Launches all mission control nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Human Fusion Node
        Node(
            package='manriix_mission',
            executable='human_fusion_node',
            name='human_fusion_node',
            output='screen',
        ),
        
        # POI Manager Node
        Node(
            package='manriix_mission',
            executable='poi_manager_node',
            name='poi_manager_node',
            output='screen',
        ),
        
        # Exploration Node
        Node(
            package='manriix_mission',
            executable='exploration_node',
            name='exploration_node',
            output='screen',
        ),
        
        # Mission Controller Node
        Node(
            package='manriix_mission',
            executable='mission_controller_node',
            name='mission_controller_node',
            output='screen',
        ),
    ])
