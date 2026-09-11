#!/usr/bin/env python3
"""
Manriix Photography System Launch

Launches perception + mission nodes ONLY.
Camera node (direct_zed_detection_node) is launched by robot_nav.launch.py with camera_mode:=full

Usage:
  Terminal 1: ros2 launch manriix_bringup robot_nav.launch.py hardware_mode:=real camera_mode:=full
  Terminal 2: ros2 launch manriix_perception complete_photography_robot.launch.py

File: /home/hype/manriix2_ws/src/manriix_perception/launch/complete_photography_robot.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    LogInfo,
    GroupAction
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():

    # Get package directories
    perception_pkg = get_package_share_directory('manriix_perception')
    nav_pkg = get_package_share_directory('manriix_navigation')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_web_viz = LaunchConfiguration('enable_web_viz')
    enable_recovery = LaunchConfiguration('enable_recovery')
    # enable_autonomous_navigation = LaunchConfiguration('enable_autonomous_navigation')

    # Config files
    photographer_params = os.path.join(perception_pkg, 'config', 'photographer_system_params.yaml')

    return LaunchDescription([
        # ==================== ENVIRONMENT ====================
        SetEnvironmentVariable('DISPLAY', ':0'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),

        # ==================== ARGUMENTS ====================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz for visualization'),

        DeclareLaunchArgument(
            'enable_web_viz',
            default_value='true',
            description='Enable Flask web visualization (requires ROSBridge)'),

        DeclareLaunchArgument(
            'enable_recovery',
            default_value='true',
            description='Enable recovery manager for fault tolerance'),

        DeclareLaunchArgument(
            'enable_autonomous_navigation',
            default_value='false',
            description='SAFETY: Enable autonomous navigation (robot will move to POIs). Set false for testing.'),

        # ==================== INFO MESSAGE ====================
        LogInfo(
            msg='\n' + '='*70 + '\n' +
                '[MANRIIX PHOTOGRAPHY] Perception + Mission System Starting\n' +
                '\n' +
                '  PREREQUISITE: robot_nav.launch.py must be running with:\n' +
                '    camera_mode:=full\n' +
                '\n' +
                '  This launch provides:\n' +
                '    ✓ GPU-accelerated human clustering\n' +
                '    ✓ POI management + Mission planning\n' +
                '    ✓ Photography intelligence\n' +
                '    ✓ Recovery manager\n' +
                '\n' +
                '  Camera node (direct_zed_detection_node) is managed by robot_nav.launch.py\n' +
                '\n' +
                '  ⚠️  AUTONOMOUS NAVIGATION: CHECK PARAMETER\n' +
                '      enable_autonomous_navigation:=false (SAFE - robot won\'t move)\n' +
                '      enable_autonomous_navigation:=true  (robot will navigate to POIs)\n' +
                '='*70
        ),

        # ==================== NAV2 + SLAM (3D MODE) ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg, 'launch', 'slam_nav.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_3d_perception': 'true',
                'use_rviz': use_rviz,
            }.items()
        ),

        # ==================== CLUSTERING ====================
        Node(
            package='manriix_perception',
            executable='human_clustering_node.py',
            name='human_clustering_node',
            output='screen',
            parameters=[
                photographer_params,
                {'use_sim_time': use_sim_time}
            ],
            respawn=True,
            respawn_delay=2.0
        ),

        # ==================== POI MANAGEMENT ====================
        Node(
            package='manriix_perception',
            executable='poi_manager.py',
            name='poi_manager',
            output='screen',
            parameters=[
                photographer_params,
                {'use_sim_time': use_sim_time}
            ],
            respawn=True,
            respawn_delay=2.0
        ),

        # ==================== MISSION CONTROL ====================
        Node(
            package='manriix_perception',
            executable='mission_controller.py',
            name='mission_controller',
            output='screen',
            parameters=[
                photographer_params,
                {
                    'use_sim_time': use_sim_time,
                    'enable_autonomous_navigation': LaunchConfiguration('enable_autonomous_navigation')
                }
            ],
            respawn=True,
            respawn_delay=2.0
        ),

        # ==================== PHOTO INTELLIGENCE ====================
        Node(
            package='manriix_perception',
            executable='photo_intelligence_system.py',
            name='photo_intelligence_system',
            output='screen',
            parameters=[
                photographer_params,
                {'use_sim_time': use_sim_time}
            ],
            respawn=True,
            respawn_delay=2.0
        ),

        # ==================== RECOVERY MANAGER ====================
        # Node(
        #     package='manriix_perception',
        #     executable='recovery_manager.py',
        #     name='recovery_manager',
        #     output='screen',
        #     parameters=[
        #         photographer_params,
        #         {'use_sim_time': use_sim_time}
        #     ],
        #     condition=IfCondition(enable_recovery),
        #     respawn=True,
        #     respawn_delay=2.0
        # ),

        Node(
            package='manriix_perception',
            executable='recovery_manager.py',
            name='recovery_manager',
            output='screen',
            parameters=[
                photographer_params,
                {
                    'use_sim_time': use_sim_time,
                    'enable_autonomous_navigation': LaunchConfiguration('enable_autonomous_navigation'),
                }
            ],
            condition=IfCondition(LaunchConfiguration('enable_recovery')),
            respawn=True,
            respawn_delay=2.0
        ),

        # ==================== WEB VISUALIZATION (OPTIONAL) ====================
        # Node(
        #     package='manriix_perception',
        #     executable='remote_viz.py',
        #     name='remote_viz',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     condition=IfCondition(enable_web_viz)
        # ),

        Node(
            package='manriix_perception',
            executable='remote_viz.py',
            name='remote_viz',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('enable_web_viz')),
            respawn=True,
            respawn_delay=2.0
        ),

        # ==================== COMPLETION MESSAGE ====================
        LogInfo(
            msg='\n' + '='*70 + '\n' +
                '[MANRIIX PHOTOGRAPHY] All nodes launched\n' +
                '\n' +
                'Monitor topics:\n' +
                '  ros2 topic echo /human_clustering/humans\n' +
                '  ros2 topic echo /mission/status\n' +
                '  ros2 topic echo /mission/pois\n' +
                '\n' +
                'Web interface (if enabled): http://localhost:5002\n' +
                '='*70
        ),
    ])