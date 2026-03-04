#!/usr/bin/env python3
"""
Launch file for move_bot_xy navigation node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Enable debug logging'
    )
    
    # Move bot node
    move_bot_node = Node(
        package='manriix_navigation',
        executable='move_bot_xy',
        name='move_base_sequence',
        output='screen',
        parameters=[{
            'debug_mode': LaunchConfiguration('debug_mode')
        }]
    )
    
    return LaunchDescription([
        debug_mode_arg,
        move_bot_node
    ])