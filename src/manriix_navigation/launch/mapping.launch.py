#!/usr/bin/env python3
"""
Manriix Mapping Launch File
Pure SLAM mapping mode - drive robot manually to build map

USAGE:
  Terminal 1: ros2 launch manriix_bringup robot_nav.launch.py hardware_mode:=real
  Terminal 2: ros2 launch manriix_navigation mapping.launch.py use_rviz:=true
  Terminal 3: ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -p save_map_timeout:=10000

NOTE:
  Costmaps are NOT launched here — they are for navigation, not mapping.
  Your slam_nav.launch.py handles costmaps via nav2_bringup's navigation_launch.py
  which includes its own lifecycle_manager with autostart: true.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_pkg = get_package_share_directory('manriix_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT',
                               '[{severity}] [{name}]: {message}'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                nav_pkg, 'config', 'slam_toolbox_config.yaml'),
            description='Full path to the SLAM Toolbox parameters file'),

        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Whether to start RViz'),

        LogInfo(msg='\n' + '=' * 60 + '\n' +
                '[MANRIIX] MAPPING MODE\n' +
                '  - Drive robot manually to build map\n' +
                '  - Save map: ros2 run nav2_map_server map_saver_cli \\\n' +
                '      -f ~/maps/venue_name \\\n' +
                '      --ros-args -p save_map_timeout:=10000\n' +
                '=' * 60),

        # ==================== SLAM TOOLBOX ====================
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
        ),

        # ==================== RVIZ ====================
        # Uses Nav2's default RViz config which has Map, TF, LaserScan,
        # costmaps, and navigation displays all pre-configured.
        # Reference: https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/
        # "ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'rviz', 'nav2_default_view.rviz')
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),

        # ==================== POSE PUBLISHER ====================
        Node(
            package='manriix_mission',
            executable='pose_publisher',
            name='pose_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])