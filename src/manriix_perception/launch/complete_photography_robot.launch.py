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
    GroupAction,
    TimerAction,
    ExecuteProcess
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
    use_nav2 = LaunchConfiguration('use_nav2')
    use_clustering = LaunchConfiguration('use_clustering')
    use_pis = LaunchConfiguration('use_pis')
    use_poi_manager = LaunchConfiguration('use_poi_manager')
    use_cpp_bt = LaunchConfiguration('use_cpp_bt')

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

        # DeclareLaunchArgument(
        #     'enable_recovery',
        #     default_value='true',
        #     description='Enable recovery manager for fault tolerance'),

        DeclareLaunchArgument(
            'enable_autonomous_navigation',
            default_value='false',
            description='SAFETY: Enable autonomous navigation (robot will move to POIs). Set false for testing.'),

        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Launch Nav2 + SLAM stack'),

        DeclareLaunchArgument(
            'use_clustering',
            default_value='true',
            description='Launch human clustering node'),

        DeclareLaunchArgument(
            'use_pis',
            default_value='true',
            description='Launch photo intelligence system'),

        DeclareLaunchArgument(
            'use_poi_manager',
            default_value='true',
            description='Launch POI manager node'),

        DeclareLaunchArgument(
            'use_cpp_bt',
            default_value='false',
            description='Use C++ BT mission controller instead of Python'),

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
            }.items(),
            condition=IfCondition(use_nav2)
        ),

        # ==================== LIFECYCLE TRANSITIONS ====================
        # Direct ros2 lifecycle set calls — no bond heartbeat, no cascade failures.
        # Each node gets 2s between configure and activate to settle.
        # Order: photo_intelligence_system → poi_manager → mission_controller
        #        → recovery_manager
        # Timing starts at t=72s (7s after last node started at t=65s).

        TimerAction(period=72.0, actions=[
            LogInfo(msg='[t=72s] Configuring photo_intelligence_system...'),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set',
                     '/photo_intelligence_system', 'configure'],
                output='screen'),
        ]),
        TimerAction(period=74.0, actions=[
            LogInfo(msg='[t=74s] Activating photo_intelligence_system...'),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set',
                     '/photo_intelligence_system', 'activate'],
                output='screen'),
        ]),
        TimerAction(period=76.0, actions=[
            LogInfo(msg='[t=76s] Configuring poi_manager...'),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set',
                     '/poi_manager', 'configure'],
                output='screen'),
        ]),
        TimerAction(period=78.0, actions=[
            LogInfo(msg='[t=78s] Activating poi_manager...'),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set',
                     '/poi_manager', 'activate'],
                output='screen'),
        ]),
        TimerAction(period=80.0, actions=[
            LogInfo(msg='[t=80s] Configuring mission_controller...'),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set',
                     '/mission_controller', 'configure'],
                output='screen'),
        ]),
        TimerAction(period=82.0, actions=[
            LogInfo(msg='[t=82s] Activating mission_controller...'),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set',
                     '/mission_controller', 'activate'],
                output='screen'),
        ]),
        # TimerAction(period=84.0, actions=[
        #     LogInfo(msg='[t=84s] Configuring recovery_manager...'),
        #     ExecuteProcess(
        #         cmd=['ros2', 'lifecycle', 'set',
        #              '/recovery_manager', 'configure'],
        #         output='screen'),
        # ]),
        # TimerAction(period=86.0, actions=[
        #     LogInfo(msg='[t=86s] Activating recovery_manager...'),
        #     ExecuteProcess(
        #         cmd=['ros2', 'lifecycle', 'set',
        #              '/recovery_manager', 'activate'],
        #         output='screen'),
        # ]),
        # ==================== CLUSTERING (t=5s) ====================
        # Cameras already running from robot_nav T1 — just needs TF tree to settle
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='[t=5s] Starting human_clustering_node...'),
                Node(
                    package='manriix_perception',
                    executable='human_clustering_node.py',
                    name='human_clustering_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        photographer_params,
                        {'use_sim_time': use_sim_time}
                    ],
                    respawn=False,
                    condition=IfCondition(use_clustering),
                ),
            ]
        ),

        # ==================== PHOTO INTELLIGENCE (t=5s) ====================
        # No external dependencies at startup — starts alongside clustering
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='[t=5s] Starting photo_intelligence_system...'),
                Node(
                    package='manriix_perception',
                    executable='photo_intelligence_system.py',
                    name='photo_intelligence_system',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        photographer_params,
                        {'use_sim_time': use_sim_time}
                    ],
                    respawn=False,
                    condition=IfCondition(use_pis),
                ),
            ]
        ),

        # ==================== POI MANAGEMENT (t=30s) ====================
        # SLAM needs ~57s to publish /map — starting at 30s reduces wasted
        # update cycles against None map. Node handles None map gracefully.
        TimerAction(
            period=30.0,
            actions=[
                LogInfo(msg='[t=30s] Starting poi_manager...'),
                Node(
                    package='manriix_perception',
                    executable='poi_manager.py',
                    name='poi_manager',
                    emulate_tty=True,
                    output='screen',
                    parameters=[
                        photographer_params,
                        {'use_sim_time': use_sim_time}
                    ],
                    respawn=False,
                    condition=IfCondition(use_poi_manager),
                ),
            ]
        ),

        # ==================== MISSION CONTROL (t=50s) ====================
        # Nav2 BT server ready at ~61s — starting at 50s gives wait_for_server(15s)
        # enough time to connect cleanly without the current 10s fail-and-retry.
        # use_cpp_bt:=true  → C++ BT mission controller (mission_controller_cpp)
        # use_cpp_bt:=false → Python BT mission controller (mission_controller.py)
        TimerAction(
            period=50.0,
            actions=[
                LogInfo(msg='[t=50s] Starting mission_controller (Python BT)...'),
                Node(
                    package='manriix_perception',
                    executable='mission_controller.py',
                    name='mission_controller',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        photographer_params,
                        {
                            'use_sim_time': use_sim_time,
                            'enable_autonomous_navigation': LaunchConfiguration('enable_autonomous_navigation'),
                        }
                    ],
                    respawn=False,
                    condition=IfCondition(
                        PythonExpression(["'", use_cpp_bt, "' == 'false'"])),
                ),
                LogInfo(msg='[t=50s] Starting mission_controller_cpp (C++ BT)...'),
                Node(
                    package='manriix_perception',
                    executable='mission_controller_cpp',
                    name='mission_controller_cpp',
                    output='screen',
                    respawn=False,
                    condition=IfCondition(use_cpp_bt),
                ),
            ]
        ),

        # ==================== RECOVERY MANAGER (t=60s) ====================
        # Starts after everything else is stable — needs clusters, Nav2 and mission status
        # TimerAction(
        #     period=60.0,
        #     actions=[
        #         LogInfo(msg='[t=60s] Starting recovery_manager...'),
        #         Node(
        #             package='manriix_perception',
        #             executable='recovery_manager.py',
        #             name='recovery_manager',
        #             output='screen',
        #             emulate_tty=True,
        #             parameters=[
        #                 photographer_params,
        #                 {
        #                     'use_sim_time': use_sim_time,
        #                     'enable_autonomous_navigation': LaunchConfiguration('enable_autonomous_navigation'),
        #                 }
        #             ],
        #             condition=IfCondition(LaunchConfiguration('enable_recovery')),
        #             respawn=False,
        #             # respawn_delay=5.0
        #         ),
        #     ]
        # ),

        # ==================== WEB VISUALIZATION (t=65s) ====================
        # Last — visualization only, no mission-critical dependency
        TimerAction(
            period=65.0,
            actions=[
                Node(
                    package='manriix_perception',
                    executable='remote_viz.py',
                    name='remote_viz',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{'use_sim_time': use_sim_time}],
                    condition=IfCondition(LaunchConfiguration('enable_web_viz')),
                    respawn=False,
                    # respawn_delay=5.0
                ),
            ]
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