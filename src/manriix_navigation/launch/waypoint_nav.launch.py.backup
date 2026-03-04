#!/usr/bin/env python3
"""
Waypoint Navigation Launch File
Launches localization + Nav2 + waypoint navigator for pre-defined route navigation
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    nav_pkg = get_package_share_directory('manriix_navigation')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    wait_mode = LaunchConfiguration('wait_mode')
    wait_time = LaunchConfiguration('wait_time')
    loop_continuous = LaunchConfiguration('loop_continuous')
    
    # Default map path
    default_map = os.path.join(nav_pkg, 'maps', 'office1.yaml')
    
    return LaunchDescription([
        # ==================== ARGUMENTS ====================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz'
        ),
        
        DeclareLaunchArgument(
            'wait_mode',
            default_value='time_based',
            description='Wait mode at waypoints: photo_command or time_based'
        ),
        
        DeclareLaunchArgument(
            'wait_time',
            default_value='300.0',
            description='Wait time in seconds (for time_based mode)'
        ),
        
        DeclareLaunchArgument(
            'loop_continuous',
            default_value='true',
            description='Loop through waypoints continuously'
        ),
        
        # ==================== INFO ====================
        LogInfo(
            msg='\n' + '='*70 + '\n' +
                '[WAYPOINT NAVIGATION] Starting...\n' +
                '  • Localization with pre-built map\n' +
                '  • Nav2 navigation stack (3D obstacle avoidance)\n' +
                '  • Waypoint navigator\n' +
                '  • Map manager\n' +
                '\n' +
                '  NOTE: Make sure robot.launch.py is running with:\n' +
                '        launch_cameras:=true (for 3D obstacle avoidance)\n' +
                '='*70
        ),
        
        # ==================== LOCALIZATION + NAV2 ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml,
                'use_rviz': use_rviz,
            }.items()
        ),
        
        # ==================== MAP MANAGER ====================
        Node(
            package='manriix_navigation',
            executable='map_manager.py',
            name='map_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'maps_directory': os.path.join(nav_pkg, 'maps')
            }]
        ),
        
        # ==================== INITIAL POSE SETTER ====================
        Node(
            package='manriix_navigation',
            executable='set_initial_pose.py',
            name='initial_pose_setter',
            output='screen',
            parameters=[{
                'initial_x': 1.165,
                'initial_y': -0.880,
                'initial_yaw': 1.54,
                'delay': 5.0,
            }]
        ),
        
        # ==================== WAYPOINT NAVIGATOR ====================
        Node(
            package='manriix_navigation',
            executable='waypoint_navigator.py',
            name='waypoint_navigator',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'wait_mode': wait_mode,
                'wait_time': wait_time,
                'loop_continuous': loop_continuous,
            }]
        ),
        
        # ==================== COMPLETION MESSAGE ====================
        LogInfo(
            msg='\n' + '='*70 + '\n' +
                '[WAYPOINT NAVIGATION] Ready!\n' +
                '\n' +
                'To set waypoints, publish JSON to /waypoint/list:\n' +
                '  ros2 topic pub /waypoint/list std_msgs/String \\\n' +
                '    "{data: \'[{\"x\":1.0,\"y\":0.0},{\"x\":2.0,\"y\":1.0}]\'}" -1\n' +
                '\n' +
                'To start navigation:\n' +
                '  ros2 topic pub /waypoint/control std_msgs/String "{data: start}" -1\n' +
                '\n' +
                'Control commands: start, stop, pause, resume, skip\n' +
                '\n' +
                'Monitor status:\n' +
                '  ros2 topic echo /waypoint/status\n' +
                '='*70
        ),
    ])
