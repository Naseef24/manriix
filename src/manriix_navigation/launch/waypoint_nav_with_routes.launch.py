#!/usr/bin/env python3
"""
Waypoint Navigation Launch File with Route Server + Keepout Zones
Launches localization + Nav2 + Route Server + keepout zones + waypoint navigator

Route Server uses ComputeRoute + FollowPath (Architecture 1) for graph-based navigation.
Keepout zones prevent the robot from entering restricted areas.

Localization modes (all verified by LIDAR scan matching):
  home:     Load saved home position from YAML + verify (DEFAULT, commercial deployment)
  manual:   Wait for operator to set pose via RViz (testing/first-time setup)
  fixed:    Use launch parameter pose + verify (known positions via CLI)
  verify:   Continuous localization quality monitoring

First-time setup:
  1. Launch with localize_mode:=manual use_rviz:=true
  2. Set pose in RViz, drive robot to desired home spot
  3. Calibrate: ros2 topic pub /home/calibrate std_msgs/String "{data: save}" -1
  4. Mark floor with tape at robot's wheels
  5. From now on, launch with default (localize_mode:=home)

Usage:
  # Default: home mode (after calibration)
  ros2 launch manriix_navigation waypoint_nav_with_routes.launch.py use_rviz:=true

  # First-time setup / testing
  ros2 launch manriix_navigation waypoint_nav_with_routes.launch.py \
    localize_mode:=manual use_rviz:=true

  # Custom keepout mask
  ros2 launch manriix_navigation waypoint_nav_with_routes.launch.py \
    keepout_mask:=/path/to/venue_keepout.yaml
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    route_graph = LaunchConfiguration('route_graph')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    keepout_mask = LaunchConfiguration('keepout_mask')

    # Localization configurations
    # localize_mode = LaunchConfiguration('localize_mode')
    # initial_x = LaunchConfiguration('initial_x')
    # initial_y = LaunchConfiguration('initial_y')
    # initial_yaw = LaunchConfiguration('initial_yaw')
    # scan_match_threshold = LaunchConfiguration('scan_match_threshold')
    # verify_interval = LaunchConfiguration('verify_interval')

    # Default paths
    default_map = os.path.join(nav_pkg, 'maps', 'office5.yaml')
    default_graph = os.path.join(nav_pkg, 'graphs', 'office5.geojson')
    default_nav2_params = os.path.join(
        nav_pkg, 'config', 'nav2', 'nav2_params_wedding_venue.yaml')
    default_rviz_config = os.path.join(
        nav_pkg, 'rviz', 'nav2_route_navigation.rviz')
    default_keepout_mask = os.path.join(
        nav_pkg, 'maps', 'keepout_zones', 'office5_keepout_mask.yaml')

    # Behavior Tree XMLs
    route_bt_xml = os.path.join(
        nav_pkg, 'behavior_trees', 'navigate_route_with_first_mile.xml')
    freespace_bt_xml = os.path.join(
        nav_pkg, 'behavior_trees', 'navigate_freespace_safety.xml')

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
            description='Launch RViz with route navigation config'
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

        DeclareLaunchArgument(
            'route_graph',
            default_value=default_graph,
            description='Path to route graph GeoJSON file'
        ),

        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params,
            description='Full path to Nav2 parameters YAML file'
        ),

        DeclareLaunchArgument(
            'keepout_mask',
            default_value=default_keepout_mask,
            description='Full path to keepout mask YAML file'
        ),

        # # Localization arguments
        # DeclareLaunchArgument(
        #     'localize_mode',
        #     default_value='home',
        #     description='Localization mode: home, manual, fixed, verify'
        # ),

        # DeclareLaunchArgument(
        #     'initial_x', default_value='0.0',
        #     description='Initial X for fixed/assisted mode'
        # ),
        # DeclareLaunchArgument(
        #     'initial_y', default_value='0.0',
        #     description='Initial Y for fixed/assisted mode'
        # ),
        # DeclareLaunchArgument(
        #     'initial_yaw', default_value='0.0',
        #     description='Initial yaw for fixed/assisted mode'
        # ),
        # DeclareLaunchArgument(
        #     'scan_match_threshold', default_value='0.30',
        #     description='Min LIDAR-to-map match score (0.0-1.0)'
        # ),
        # DeclareLaunchArgument(
        #     'verify_interval', default_value='3.0',
        #     description='How often to re-verify localization (seconds)'
        # ),

        # ==================== INFO ====================
        LogInfo(
            msg='\n' + '=' * 70 + '\n'
                '[ROUTE NAVIGATION] Starting...\n'
                '  • Localization with pre-built map\n'
                '  • Nav2 navigation stack\n'
                '  • Keepout zones (restricted areas)\n'
                '  • Route Server (structured graph navigation)\n'
                '  • AMCL Localization (set initial pose in RViz)\n'
                '  • Waypoint navigator (hybrid route + freespace)\n'
                '\n'
                '  ARCHITECTURE:\n'
                '    WP1: Freespace BT (first-mile, gets robot onto graph)\n'
                '    WP2+: Route BT (ComputeRoute → SmoothPath → FollowPath)\n'
                '    On failure: Auto-fallback to Freespace BT\n'
                '\n'
                '  NOTE: Make sure robot.launch.py is running with:\n'
                '        launch_cameras:=true (for 3D obstacle avoidance)\n'
                '=' * 70
        ),

        # ==================== LOCALIZATION + NAV2 ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml,
                'nav2_params_file': nav2_params_file,
                'use_rviz': use_rviz,
                'rviz_config': default_rviz_config,
            }.items()
        ),

        # ==================== KEEPOUT ZONE SERVERS ====================
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_filter_mask_server',
            output='screen',
            parameters=[
                nav2_params_file,
                {'yaml_filename': keepout_mask},
            ],
        ),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_costmap_filter_info_server',
            output='screen',
            parameters=[nav2_params_file],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_keepout_zone',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'keepout_filter_mask_server',
                    'keepout_costmap_filter_info_server',
                ]
            }],
        ),

        # # ==================== MAP MANAGER ====================
        # Node(
        #     package='manriix_navigation',
        #     executable='map_manager.py',
        #     name='map_manager',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #         'maps_directory': os.path.join(nav_pkg, 'maps')
        #     }]
        # ),

        # # ==================== HOME POSITION MANAGER ====================
        # Node(
        #     package='manriix_navigation',
        #     executable='home_manager.py',
        #     name='home_position_manager',
        #     output='screen',
        #     parameters=[{
        #         'map_name': 'office1',
        #     }]
        # ),

        # # ==================== AUTO-LOCALIZATION ====================
        # Node(
        #     package='manriix_navigation',
        #     executable='auto_localize.py',
        #     name='auto_localize',
        #     output='screen',
        #     parameters=[{
        #         'mode': localize_mode,
        #         'initial_x': initial_x,
        #         'initial_y': initial_y,
        #         'initial_yaw': initial_yaw,
        #         'startup_delay': 5.0,
        #         'scan_match_threshold': scan_match_threshold,
        #         'verify_interval': verify_interval,
        #     }]
        # ),

        # ==================== ROUTE SERVER ====================
        Node(
            package='nav2_route',
            executable='route_server',
            name='route_server',
            output='screen',
            parameters=[
                nav2_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'graph_filepath': route_graph,
                }
            ]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='route_lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['route_server']
            }],
        ),

        # ==================== WAYPOINT NAVIGATOR ====================
        # Hybrid navigation: first-mile freespace + route graph
        # All navigation goes through NavigateToPose with dynamic BT selection
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
                'route_graph': route_graph,
                # BT XMLs for dynamic switching per-waypoint
                'route_bt_xml': route_bt_xml,
                'freespace_bt_xml': freespace_bt_xml,
                # Robot considered "on graph" within this distance of a node
                'on_graph_distance': 1.5,
            }]
        ),

        # ==================== COMPLETION MESSAGE ====================
        LogInfo(
            msg='\n' + '=' * 70 + '\n'
                '[ROUTE NAVIGATION] All nodes launched!\n'
                '\n'
                'Home Position:\n'
                '  Calibrate: ros2 topic pub /home/calibrate std_msgs/String "{data: save}" -1\n'
                '  Return:    ros2 topic pub /home/return std_msgs/String "{data: go}" -1\n'
                '  Monitor:   ros2 topic echo /localization/status\n'
                '\n'
                'Set waypoints (node IDs):\n'
                '  ros2 topic pub /waypoint/list std_msgs/String \\\n'
                '    "{data: \'[{\"node_id\":0},{\"node_id\":1},{\"node_id\":5}]\'}" -1\n'
                '\n'
                'Start navigation:\n'
                '  ros2 topic pub /waypoint/control std_msgs/String "{data: start}" -1\n'
                '\n'
                'Control: start, stop, pause, resume, skip\n'
                'Monitor: ros2 topic echo /waypoint/status\n'
                '=' * 70
        ),
    ])