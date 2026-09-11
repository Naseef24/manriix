#!/usr/bin/env python3
"""
================================================================================
Manriix Visualization Include — Production-safe Foxglove bridge

Purpose
-------
Expose a curated, low-bandwidth remote monitoring view without allowing
Foxglove/network load to interfere with cuVSLAM, nvblox, Nav2, or safety.

Camera monitoring topics are produced by direct_zed_detection_node_modified.py:
  /zedx_front/zed_node/left/image_rect_color/compressed
  /zedx_left/zed_node/left/image_rect_color/compressed
  /zedx_right/zed_node/left/image_rect_color/compressed

The ZED driver publishes these as low-rate (~3 Hz), resized JPEGs using
BEST_EFFORT + KEEP_LAST(1). Raw images, depth images, dense point clouds,
and broad nvblox wildcards are NOT exposed in the production whitelist.

File:
  ~/manriix2_ws/src/manriix_bringup/launch/include/visualization_include.launch.py
================================================================================
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_navigation = get_package_share_directory('manriix_navigation')

    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='True',
        description='Launch foxglove_bridge.',
    )

    foxglove_layout_arg = DeclareLaunchArgument(
        'foxglove_layout',
        default_value='manriix_navigation',
        choices=['manriix_navigation', 'manriix_perception', 'manriix_debug'],
        description='Foxglove layout to suggest to the user.',
    )

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='WebSocket port for foxglove_bridge.',
    )

    use_foxglove_whitelist_arg = DeclareLaunchArgument(
        'use_foxglove_whitelist',
        default_value='True',
        description=(
            'True = production-safe curated topics. '
            'False = expose all topics for short local debugging only.'
        ),
    )

    enable_remote_camera_monitor_arg = DeclareLaunchArgument(
        'enable_remote_camera_monitor',
        default_value='False',
        description=(
            'Launch the isolated remote JPEG compressor. Default False so '
            'camera monitoring can never block the critical robot bring-up.'
        ),
    )

    remote_camera_monitor_delay_arg = DeclareLaunchArgument(
        'remote_camera_monitor_delay',
        default_value='20.0',
        description=(
            'Seconds to wait after launch before starting the low-priority '
            'remote camera compressor.'
        ),
    )

    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_layout = LaunchConfiguration('foxglove_layout')
    foxglove_port = LaunchConfiguration('foxglove_port')
    use_foxglove_whitelist = LaunchConfiguration('use_foxglove_whitelist')
    enable_remote_camera_monitor = LaunchConfiguration('enable_remote_camera_monitor')
    remote_camera_monitor_delay = LaunchConfiguration('remote_camera_monitor_delay')

    # Production whitelist: avoid raw camera/depth streams, dense clouds,
    # nvblox mesh/ESDF debug streams, and broad /nvblox_node/.* exposure.
    whitelisted_topics = [
        # TF + commands
        '/tf',
        '/tf_static',
        '/clock',
        '/cmd_vel',
        '/cmd_vel_safe',
        '/cmd_vel_smoothed',

        # Nav2 essentials
        '/local_costmap/costmap',
        '/local_costmap/costmap_updates',
        '/local_costmap/published_footprint',
        '/global_costmap/costmap',
        '/global_costmap/costmap_updates',
        '/plan',
        '/plan_smoothed',
        '/received_global_plan',
        '/goal_pose',
        '/initialpose',

        # Robot state / localization
        '/odom',
        '/visual_slam/tracking/odometry',
        '/visual_slam/tracking/vo_pose',
        '/joint_states',

        # Nvblox: lightweight operator products only.
        '/nvblox_node/combined_occupancy_grid',
        '/nvblox_node/dynamic_occupancy_layer',

        # 2D lidar + collision monitor
        '/scan',
        '/polygon_stop',
        '/polygon_slowdown',
        '/collision_monitor_state',

        # LOW-RATE compressed operator camera feeds only.
        '/zedx_front/zed_node/left/image_rect_color/compressed',
        '/zedx_left/zed_node/left/image_rect_color/compressed',
        '/zedx_right/zed_node/left/image_rect_color/compressed',

        # Lightweight perception outputs / markers.
        '/manriix/classification_markers',
        '/manriix/cluster_markers',
        '/manriix/cluster_tracks',
        '/manriix/cluster_tracks_classified',
        '/manriix/cluster_tracks_fused',
        '/manriix/fusion_markers',
        '/manriix/motion_markers',
        '/manriix/objects',
        '/manriix/person_boxes',
        '/manriix/person_markers',
        '/manriix/camera_fov',

        # Zone + diagnostics
        '/zone_manager/.*',
        '/diagnostics',
        '/rosout',
        '/parameter_events',
    ]

    common_parameters = {
        'port': foxglove_port,
        'address': '0.0.0.0',
        'tls': False,
        'param_whitelist': ['.*'],
        'service_whitelist': ['.*'],
        'client_topic_whitelist': ['.*'],

        # Keep queues shallow. Remote visualization is allowed to drop data.
        'min_qos_depth': 1,
        'max_qos_depth': 2,

        # Bound each client connection. If a client/network cannot keep up,
        # Foxglove drops messages instead of accumulating seconds of stale data.
        'send_buffer_limit': 4000000,

        # WebSocket compression costs CPU and is unnecessary for JPEG camera data.
        'use_compression': False,

        'capabilities': [
            'clientPublish',
            'parameters',
            'parametersSubscribe',
            'services',
            'connectionGraph',
            'assets',
        ],
    }

    foxglove_bridge_whitelisted = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            **common_parameters,
            'topic_whitelist': whitelisted_topics,
        }],
        condition=IfCondition(use_foxglove_whitelist),
    )

    # Debug-only bridge. Never use this for routine remote operation because
    # raw images, dense clouds, nvblox mesh, etc. may saturate CPU/network.
    foxglove_bridge_unfiltered = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[common_parameters],
        condition=UnlessCondition(use_foxglove_whitelist),
    )

    remote_camera_compressor = Node(
        package='manriix_perception',
        executable='remote_camera_compressor_node.py',
        name='remote_camera_compressor',
        output='screen',

        # Monitoring is intentionally lower priority than navigation/perception.
        prefix='nice -n 15',

        parameters=[{
            'jpeg_quality': 55,
            'output_rate_hz': 3.0,
        }],
    )

    # Start monitoring only after the critical perception/navigation stack has
    # had time to initialize. The compressor remains a separate low-priority
    # process and can be disabled independently of Foxglove.
    delayed_remote_camera_compressor = TimerAction(
        period=remote_camera_monitor_delay,
        actions=[remote_camera_compressor],
        condition=IfCondition(enable_remote_camera_monitor),
    )

    layout_path = PathJoinSubstitution([
        pkg_navigation,
        'config',
        'foxglove_layouts',
        foxglove_layout,
    ])

    return LaunchDescription([
        use_foxglove_arg,
        foxglove_layout_arg,
        foxglove_port_arg,
        use_foxglove_whitelist_arg,
        enable_remote_camera_monitor_arg,
        remote_camera_monitor_delay_arg,

        LogInfo(
            msg=[
                '\n', '=' * 70, '\n',
                '[VISUALIZATION] Foxglove Bridge\n',
                '  Enabled:        ', use_foxglove, '\n',
                '  Port:           ', foxglove_port, '\n',
                '  Safe whitelist: ', use_foxglove_whitelist, '\n',
                '  Remote cameras:  ', enable_remote_camera_monitor, ' (delayed ', remote_camera_monitor_delay, ' s)\n',
                '  Layout file:    ', layout_path, '.json\n',
                '  Remote URL:     ws://<jetson-ip>:', foxglove_port, '\n',
                '=' * 70,
            ],
            condition=IfCondition(use_foxglove),
        ),

        # Grouping by use_foxglove is unnecessary because this include itself
        # is only useful when enabled; conditions below prevent duplicate bridge.
        delayed_remote_camera_compressor,
        foxglove_bridge_whitelisted,
        foxglove_bridge_unfiltered,
    ])