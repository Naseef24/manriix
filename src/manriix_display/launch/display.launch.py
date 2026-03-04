#!/usr/bin/env python3
"""
Manriix Display Launch File
Launches the web-based initialization wizard and control panel
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directory
    display_pkg = get_package_share_directory('manriix_display')
    
    return LaunchDescription([
        # ==================== ARGUMENTS ====================
        DeclareLaunchArgument(
            'web_port',
            default_value='5000',
            description='Web server port'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # ==================== INFO ====================
        LogInfo(
            msg='\n' + '='*70 + '\n' +
                '[MANRIIX DISPLAY] Starting Web Control Panel\n' +
                '\n' +
                '  Web Interface: http://localhost:5000\n' +
                '  (or http://<robot_ip>:5000 from other devices)\n' +
                '\n' +
                '  Features:\n' +
                '  • Operation mode selection (Waypoint/Detection/Manual)\n' +
                '  • Map management (list, save, load)\n' +
                '  • Waypoint selection and navigation\n' +
                '  • Virtual joystick teleop\n' +
                '  • System status monitoring\n' +
                '\n' +
                '  NOTE: Run robot.launch.py first!\n' +
                '='*70
        ),
        
        # ==================== DISPLAY NODE ====================
        Node(
            package='manriix_display',
            executable='display_node.py',
            name='display_node',
            output='screen',
            parameters=[{
                'web_port': LaunchConfiguration('web_port'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),
        
        # ==================== COMPLETION MESSAGE ====================
        LogInfo(
            msg='\n' + '='*70 + '\n' +
                '[MANRIIX DISPLAY] Web Control Panel Ready!\n' +
                '\n' +
                'Open your browser and go to:\n' +
                '  http://localhost:5000\n' +
                '='*70
        ),
    ])
