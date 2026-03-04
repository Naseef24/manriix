#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directory
    control_pkg = get_package_share_directory('manriix_control')
    
    # Launch arguments
    use_twist_mux = LaunchConfiguration('use_twist_mux', default='true')
    
    # Config file
    twist_mux_config = os.path.join(control_pkg, 'config', 'twist_mux.yaml')
    
    # Twist Mux node - multiplexes velocity commands by priority
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        remappings=[
            ('cmd_vel_out', '/manriix_4ws_controller/cmd_vel')
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_twist_mux',
            default_value='true',
            description='Use twist_mux for priority control'),
        
        # Launch twist_mux
        twist_mux_node,
    ])