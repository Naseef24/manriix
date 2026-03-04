#!/usr/bin/env python3
"""
Direct Photographer System Launch File
=======================================
Launches the complete photographer robot system using DIRECT ZED SDK detection
instead of zed_ros2_wrapper for significantly improved latency (~2ms vs 100-800ms).

Components:
1. Direct ZED Detection Node - Multi-camera perception using pyzed.sl SDK
2. Human Clustering Node - Processes human detections into clusters
3. POI Manager - Point of interest management and position optimization
4. Mission Controller - Sophisticated state machine for mission execution
5. Photo Intelligence System - Dynamic photo timing with crowd analysis
6. Recovery Manager - 8-level escalation recovery system

Usage:
  ros2 launch manriix_navigation direct_photographer_system.launch.py

Optional parameters:
  enable_visualization:=true/false  (default: true)
  enable_recovery:=true/false       (default: true)
  detection_model:=MODEL_NAME       (default: YOLO11)
  config_file:=path_to_config       (default: photographer_system_params.yaml)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('manriix_navigation')

    # Config files
    detection_config = os.path.join(pkg_dir, 'config', 'detection_config.yaml')
    system_params = os.path.join(pkg_dir, 'config', 'photographer_system_params.yaml')

    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================

    declare_enable_visualization = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable visualization components'
    )

    declare_enable_recovery = DeclareLaunchArgument(
        'enable_recovery',
        default_value='true',
        description='Enable recovery manager'
    )

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='photographer_system_params.yaml',
        description='Configuration file name'
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level (DEBUG, INFO, WARN, ERROR)'
    )

    # Direct detection specific arguments
    declare_detection_model = DeclareLaunchArgument(
        'detection_model',
        default_value='YOLO11',
        description='Detection model: YOLO11, ZED_PERSON_FAST, ZED_PERSON_ACCURATE'
    )

    declare_yolo_model_path = DeclareLaunchArgument(
        'yolo_model_path',
        default_value='~/models/yolo11/yolo11s.onnx',
        description='Path to YOLO ONNX model file'
    )

    declare_confidence_threshold = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='50',
        description='Detection confidence threshold (0-100)'
    )

    declare_publish_images = DeclareLaunchArgument(
        'publish_images',
        default_value='true',
        description='Publish camera images with bounding boxes'
    )

    # Camera serial numbers
    declare_camera_front_serial = DeclareLaunchArgument(
        'camera_front_serial',
        default_value='41911351',
        description='Front camera serial number'
    )

    declare_camera_left_serial = DeclareLaunchArgument(
        'camera_left_serial',
        default_value='40487925',
        description='Left camera serial number'
    )

    declare_camera_right_serial = DeclareLaunchArgument(
        'camera_right_serial',
        default_value='46311875',
        description='Right camera serial number (0 = disabled)'
    )

    # Get launch configurations
    enable_visualization = LaunchConfiguration('enable_visualization')
    enable_recovery = LaunchConfiguration('enable_recovery')
    config_file = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')

    # Package paths
    pkg_share = FindPackageShare('manriix_navigation')
    config_path = PathJoinSubstitution([pkg_share, 'config', config_file])

    # ==========================================================================
    # CORE PERCEPTION PIPELINE - DIRECT SDK (LOW LATENCY)
    # ==========================================================================

    # Direct ZED Detection Node - Multi-camera perception using pyzed.sl SDK
    # This replaces zed_fusion_node with direct SDK access for ~50x better latency
    direct_detection_node = Node(
        package='manriix_navigation',
        executable='direct_zed_detection_node',
        name='direct_zed_detection_node',
        output='screen',
        parameters=[
            detection_config,
            system_params,
            {
                'detection_model': LaunchConfiguration('detection_model'),
                'yolo_model_path': LaunchConfiguration('yolo_model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'publish_images': LaunchConfiguration('publish_images'),
                'camera_front_serial': LaunchConfiguration('camera_front_serial'),
                'camera_left_serial': LaunchConfiguration('camera_left_serial'),
                'camera_right_serial': LaunchConfiguration('camera_right_serial'),
                # ZED X optimized settings
                'resolution': 'HD1080',
                'depth_mode': 'NEURAL',
                'fps': 15,
                'draw_bounding_boxes': True,
                'image_scale': 0.5,
                'jpeg_quality': 75,
                'publish_rate': 15.0,
            }
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=5.0
    )

    # ==========================================================================
    # CLUSTERING PIPELINE
    # ==========================================================================

    # Human Clustering Node - Processes human detections into clusters
    human_clustering_node = Node(
        package='manriix_navigation',
        executable='human_clustering_node.py',
        name='human_clustering_node',
        parameters=[config_path],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=3.0
    )

    # ==========================================================================
    # INTELLIGENCE COMPONENTS
    # ==========================================================================

    # POI Manager - Position optimization and POI management
    poi_manager_node = Node(
        package='manriix_navigation',
        executable='poi_manager.py',
        name='poi_manager',
        parameters=[{
            'personal_space_radius': 1.2,
            'social_space_radius': 2.5,
            'photo_distance_min': 2.0,
            'photo_distance_max': 8.0,
            'photo_distance_optimal': 3.5,
            'rule_of_thirds_weight': 0.3,
            'symmetry_weight': 0.2,
            'formation_weight': 0.25,
            'accessibility_weight': 0.25,
            'poi_update_rate': 2.0,
            'position_stability_time': 3.0
        }],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=3.0
    )

    # Photo Intelligence System - Dynamic photo timing
    photo_intelligence_node = Node(
        package='manriix_navigation',
        executable='photo_intelligence_system.py',
        name='photo_intelligence_system',
        parameters=[{
            'min_photo_duration': 30.0,
            'max_photo_duration': 120.0,
            'base_duration': 60.0,
            'evaluation_time': 3.0,
            'cooldown_time': 10.0,
            'stability_threshold': 2.0
        }],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=3.0
    )

    # Mission Controller - State machine and mission management
    mission_controller_node = Node(
        package='manriix_navigation',
        executable='mission_controller.py',
        name='mission_controller',
        parameters=[{
            'evaluation_time': 2.0,
            'stability_wait_time': 3.0,
            'arrival_threshold': 1.0,
            'transition_speed_factor': 0.7,
            'target_switch_threshold': 0.15,
            'max_evaluation_time': 10.0,
            'stability_threshold': 0.05,
            'replanning_interval': 5.0
        }],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=3.0
    )

    # Recovery Manager - 8-level escalation system
    recovery_manager_node = Node(
        package='manriix_navigation',
        executable='recovery_manager.py',
        name='recovery_manager',
        parameters=[{
            'no_data_timeout': 30.0,
            'stale_data_timeout': 5.0,
            'max_recovery_attempts': 3,
            'escalation_timeout': 60.0,
            'system_health_check_interval': 5.0,
            'safe_parking_position': [0.0, 0.0]
        }],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(enable_recovery)
    )

    # ==========================================================================
    # SUPPORTING COMPONENTS
    # ==========================================================================

    # Object Costmap Bridge - Navigation integration
    costmap_bridge_node = Node(
        package='manriix_navigation',
        executable='object_costmap_bridge.py',
        name='object_costmap_bridge',
        parameters=[config_path],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=3.0
    )

    # Remote Visualizer - Web interface using ROSBridge
    remote_viz_node = Node(
        package='manriix_navigation',
        executable='remote_viz.py',
        name='remote_viz',
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        condition=IfCondition(enable_visualization)
    )

    # ==========================================================================
    # LAUNCH DESCRIPTION
    # ==========================================================================

    return LaunchDescription([
        # Launch arguments
        declare_enable_visualization,
        declare_enable_recovery,
        declare_config_file,
        declare_log_level,
        declare_detection_model,
        declare_yolo_model_path,
        declare_confidence_threshold,
        declare_publish_images,
        declare_camera_front_serial,
        declare_camera_left_serial,
        declare_camera_right_serial,

        # Core perception - DIRECT SDK (replaces zed_fusion_node)
        direct_detection_node,

        # Clustering pipeline
        human_clustering_node,

        # Intelligence components
        poi_manager_node,
        photo_intelligence_node,
        mission_controller_node,
        recovery_manager_node,

        # Supporting components
        costmap_bridge_node,
        remote_viz_node,
    ])


"""
==========================================================================
KEY DIFFERENCES FROM photographer_system.launch.py
==========================================================================

1. DETECTION NODE:
   - OLD: zed_fusion_node.py (uses zed_ros2_wrapper, 100-800ms latency)
   - NEW: direct_zed_detection_node (uses pyzed.sl SDK, ~2ms latency)

2. CAMERA ACCESS:
   - OLD: Subscribes to /zed/zed_node/* ROS topics
   - NEW: Opens cameras directly via serial number

3. IMAGE PUBLISHING:
   - OLD: Separate processes may add latency
   - NEW: Pre-draws bounding boxes before publishing, optimized pipeline

4. TOPIC COMPATIBILITY:
   - Both publish to /human_clustering/humans (HumansList message)
   - Downstream nodes (clustering, POI, mission) work unchanged

==========================================================================
USAGE EXAMPLES
==========================================================================

# Basic usage (default YOLO11 model):
ros2 launch manriix_navigation direct_photographer_system.launch.py

# With ZED built-in detection:
ros2 launch manriix_navigation direct_photographer_system.launch.py detection_model:=ZED_PERSON_ACCURATE

# Disable web visualization:
ros2 launch manriix_navigation direct_photographer_system.launch.py enable_visualization:=false

# Debug mode:
ros2 launch manriix_navigation direct_photographer_system.launch.py log_level:=DEBUG

# Custom camera setup (e.g., only two cameras):
ros2 launch manriix_navigation direct_photographer_system.launch.py camera_right_serial:=0

==========================================================================
"""
