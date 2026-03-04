#!/usr/bin/env python3
"""
Manriix Localization + Navigation Launch File
Load saved map and navigate using AMCL localization + full Nav2 stack

Usage:
  ros2 launch manriix_navigation localization.launch.py map:=/path/to/map.yaml
  ros2 launch manriix_navigation localization.launch.py map:=/path/to/map.yaml use_3d_perception:=true
  ros2 launch manriix_navigation localization.launch.py use_rviz:=true rviz_config:=/path/to/custom.rviz
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_pkg = get_package_share_directory('manriix_navigation')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_params_3d_file = LaunchConfiguration('nav2_params_3d_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_3d_perception = LaunchConfiguration('use_3d_perception')
    autostart = LaunchConfiguration('autostart')
    rviz_config = LaunchConfiguration('rviz_config')

    # Default RViz configs
    rviz_config_2d = os.path.join(nav_pkg, 'rviz', 'nav2.rviz')
    rviz_config_3d = os.path.join(nav_pkg, 'rviz', 'nav2_3d.rviz')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT',
                               '[{severity}] [{name}]: {message}'),

        # ==================== ARGUMENTS ====================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav_pkg, 'maps', 'map.yaml'),
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav_pkg, 'config', 'nav2', 'nav2_params.yaml'),
            description='Full path to Nav2 parameters file (2D mode)'),

        DeclareLaunchArgument(
            'nav2_params_3d_file',
            default_value=os.path.join(nav_pkg, 'config', 'nav2', 'nav2_params_3d_full.yaml'),
            description='Full path to Nav2 parameters file (3D mode with STVL)'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to start RViz'),

        DeclareLaunchArgument(
            'use_3d_perception',
            default_value='false',
            description='Enable 3D perception with STVL'),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the navigation stack'),

        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Override RViz config file path. '
                        'If empty, uses nav2.rviz (2D) or nav2_3d.rviz (3D)'),

        # ==================== INFO MESSAGES ====================
        LogInfo(
            condition=UnlessCondition(use_3d_perception),
            msg='\n' + '=' * 60 + '\n'
                '[MANRIIX] LOCALIZATION MODE - 2D\n'
                '  - Using saved map with AMCL localization\n'
                '  - RPLidar obstacle detection only\n'
                '  - RViz config: nav2.rviz\n'
                '=' * 60
        ),
        LogInfo(
            condition=IfCondition(use_3d_perception),
            msg='\n' + '=' * 60 + '\n'
                '[MANRIIX] LOCALIZATION MODE - 3D\n'
                '  - Using saved map with AMCL localization\n'
                '  - RPLidar + ZED STVL enabled\n'
                '  - RViz config: nav2_3d.rviz\n'
                '=' * 60
        ),

        # ==================== MAP SERVER ====================
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_yaml_file}
            ],
        ),

        # ==================== AMCL LOCALIZATION ====================
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                nav2_params_file,
                {'use_sim_time': use_sim_time}
            ],
        ),

        # ==================== LIFECYCLE MANAGER FOR LOCALIZATION ====================
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': ['map_server', 'amcl']}
            ],
        ),

        # ==================== NAV2 STACK (2D MODE) ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
                'autostart': 'true',
            }.items(),
            condition=UnlessCondition(use_3d_perception)
        ),

        # ==================== NAV2 STACK (3D MODE) ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_3d_file,
                'autostart': 'true',
            }.items(),
            condition=IfCondition(use_3d_perception)
        ),

        # ==================== RVIZ - CUSTOM CONFIG (when parent passes rviz_config) ====================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(
                PythonExpression([
                    "'", use_rviz, "' == 'true' and '",
                    rviz_config, "' != ''"
                ])
            )
        ),

        # ==================== RVIZ - 2D DEFAULT (no override, 2D mode) ====================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_2d],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(
                PythonExpression([
                    "'", use_rviz, "' == 'true' and '",
                    rviz_config, "' == '' and '",
                    use_3d_perception, "' == 'false'"
                ])
            )
        ),

        # ==================== RVIZ - 3D DEFAULT (no override, 3D mode) ====================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_3d],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(
                PythonExpression([
                    "'", use_rviz, "' == 'true' and '",
                    rviz_config, "' == '' and '",
                    use_3d_perception, "' == 'true'"
                ])
            )
        ),

        # ==================== POSE PUBLISHER ====================
        Node(
            package='manriix_mission',
            executable='pose_publisher',
            name='pose_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])