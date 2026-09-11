#!/usr/bin/env python3
"""
================================================================================
Manriix cuVSLAM + nvblox + Nav2 — NITROS-enabled launch (STAGGERED)

Architecture:
  - Single shared ComposableNodeContainer "nvblox_container" (NITROS-capable)
  - Components loaded into the container via LoadComposableNodes at staggered
    times, NOT all at once. This prevents the multi-camera Argus race that
    causes "INVALID FUNCTION CALL" / "Corrupted frames" on the last-to-open
    camera.
  - Staggered loading verified safe by mirror of working
    robot_nav_wrapper.launch.py timing pattern (5/11/17s for 3 separate
    containers — here we use 2/9/16s relative to container start).

Load timeline:
  t=0s   container starts (empty)
  t=2s   zedx_front  (NEURAL_LIGHT depth + IMU)
  t=9s   zedx_left   (gray only, no depth)
  t=16s  zedx_right  (gray only, no depth)
  t=24s  visual_slam (cuVSLAM — needs camera topics live)
  t=26s  nvblox_node (needs depth topic + tf from cuVSLAM)
  t=35s  Nav2 stack  (needs nvblox costmap + tf chain)

Required environment (set by SetEnvironmentVariable below):
  DISPLAY=:0
    ZED SDK NEURAL depth modes need GPU-backed X display for EGL alloc.
    Without this, Argus crashes at ScopedNvGlsiEglImageRef::reset().
    Verified fix per Stereolabs forum.
  XAUTHORITY=/run/user/1000/gdm/Xauthority
    X11 auth for connecting to display :0 owned by gdm.

Prerequisites (in another terminal):
  ros2 launch manriix_bringup robot_nav_nvblox.launch.py hardware_mode:=real

Plugin entry points (verified via ament_index):
  - stereolabs::ZedCamera                           → libzed_camera_component.so
  - nvidia::isaac_ros::visual_slam::VisualSlamNode  → libvisual_slam_node.so
  - nvblox::NvbloxNode                              → libnvblox_ros_lib.so

File: /home/hype/manriix2_ws/src/manriix_navigation/launch/cuvslam_for_nvblox.launch.py
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
    SetEnvironmentVariable, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


# ── GXF library path — required by isaac_ros_visual_slam and nvblox ──
GXF_LIB_PATH = (
    '/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/serialization:'
    '/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/std:'
    '/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/cuda:'
    '/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/logger:'
    '/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/ipc/grpc'
)

CONTAINER_NAME = 'nvblox_container'
CONTAINER_FULL_NAME = '/' + CONTAINER_NAME   # used as target_container


def generate_launch_description():
    # ── Package directories ─────────────────────────────────────────────────
    pkg_bringup         = get_package_share_directory('manriix_bringup')
    pkg_navigation      = get_package_share_directory('manriix_navigation')
    pkg_nav2_bringup    = get_package_share_directory('nav2_bringup')
    pkg_nvblox_examples = get_package_share_directory('nvblox_examples_bringup')

    # ── ZED config paths ────────────────────────────────────────────────────
    zed_front_yaml = os.path.join(pkg_bringup, 'config', 'zedx_front_depth.yaml')
    zed_left_yaml  = os.path.join(pkg_bringup, 'config', 'zedx_left_nodepth.yaml')
    zed_right_yaml = os.path.join(pkg_bringup, 'config', 'zedx_right_nodepth.yaml')

    # ── nvblox config paths (NVIDIA's base + ZED specialization + manriix overrides) ──
    nvblox_base_yaml    = os.path.join(pkg_nvblox_examples, 'config', 'nvblox',
                                       'nvblox_base.yaml')
    nvblox_zed_yaml     = os.path.join(pkg_nvblox_examples, 'config', 'nvblox',
                                       'specializations', 'nvblox_zed.yaml')
    nvblox_manriix_yaml = os.path.join(pkg_navigation, 'config', 'nvblox',
                                       'nvblox_manriix_dynamic.yaml')

    # ── Nav2 config ─────────────────────────────────────────────────────────
    nav2_params_nvblox = os.path.join(pkg_navigation, 'config', 'nav2',
                                      'nav2_params_nvblox.yaml')

    ld = LaunchDescription()

    # ── Environment ─────────────────────────────────────────────────────────
    ld.add_action(SetEnvironmentVariable('DISPLAY', ':0'))
    ld.add_action(SetEnvironmentVariable('XAUTHORITY', '/run/user/1000/gdm/Xauthority'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'))
    ld.add_action(SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'))
    ld.add_action(SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=[GXF_LIB_PATH + ':', os.environ.get('LD_LIBRARY_PATH', '')]
    ))

    # ── Launch arguments ────────────────────────────────────────────────────
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'use_nav2', default_value='true',
        description='Launch Nav2 stack. Disable to test cuVSLAM+nvblox alone.'))
    ld.add_action(DeclareLaunchArgument(
        'use_foxglove', default_value='true',
        description='Launch foxglove_bridge for remote viz.'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_nav2     = LaunchConfiguration('use_nav2')
    use_foxglove = LaunchConfiguration('use_foxglove')

    ld.add_action(LogInfo(msg='\n' + '=' * 70 + '\n'
        '[MANRIIX cuVSLAM + nvblox + Nav2 — NITROS stack, STAGGERED load]\n'
        '  Container: nvblox_container (single, shared, NITROS-enabled)\n'
        '  Load timeline:\n'
        '    t=0s   container starts empty\n'
        '    t=2s   zedx_front  (NEURAL_LIGHT depth + IMU)\n'
        '    t=9s   zedx_left   (gray only)\n'
        '    t=16s  zedx_right  (gray only)\n'
        '    t=24s  visual_slam (cuVSLAM, 3-stereo VIO)\n'
        '    t=26s  nvblox_node (depth integration + ESDF)\n'
        '    t=35s  Nav2        (needs nvblox costmap + tf)\n'
        '  Separate:\n'
        '    foxglove_bridge (ws://<jetson-ip>:8765)\n' +
        '=' * 70))

    # ────────────────────────────────────────────────────────────────────────
    # STEP 1 — Start empty container immediately
    # ────────────────────────────────────────────────────────────────────────
    nvblox_container = ComposableNodeContainer(
        name=CONTAINER_NAME,
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    ld.add_action(nvblox_container)

    # ────────────────────────────────────────────────────────────────────────
    # STEP 2 — Build the 5 ComposableNode descriptors
    # ────────────────────────────────────────────────────────────────────────

    zedx_front_node = ComposableNode(
        package='zed_components',
        namespace='zedx_front',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=[
            zed_front_yaml,
            {
                'debug.disable_nitros': False,
                'use_sim_time': use_sim_time,
            },
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    zedx_left_node = ComposableNode(
        package='zed_components',
        namespace='zedx_left',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=[
            zed_left_yaml,
            {
                'debug.disable_nitros': False,
                'use_sim_time': use_sim_time,
            },
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    zedx_right_node = ComposableNode(
        package='zed_components',
        namespace='zedx_right',
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=[
            zed_right_yaml,
            {
                'debug.disable_nitros': False,
                'use_sim_time': use_sim_time,
            },
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # ─── cuVSLAM (3 stereo pairs = 6 image streams) ────────────────────────
    camera_optical_frames = [
        'zedx_front_left_camera_frame_optical',
        'zedx_front_right_camera_frame_optical',
        'zedx_left_left_camera_frame_optical',
        'zedx_left_right_camera_frame_optical',
        'zedx_right_left_camera_frame_optical',
        'zedx_right_right_camera_frame_optical',
    ]

    visual_slam_node = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame':   'base_footprint',
            'odom_frame':   'odom',
            'map_frame':    'map',
            'imu_frame':    'zedx_front_imu_link',
            'camera_optical_frames': camera_optical_frames,
            'num_cameras':   6,
            'multicam_mode': 0,
            'enable_imu_fusion': True,
            'gyro_noise_density':  0.000244,
            'gyro_random_walk':    0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk':   0.003,
            'calibration_frequency': 200.0,
            'enable_localization_n_mapping': True,
            'enable_slam_visualization': True,
            'enable_observations_view': True,
            'enable_landmarks_view': True,
            'publish_map_to_odom_tf': True,
            'publish_odom_to_base_tf': True,
            'invert_map_to_odom_tf': False,
            'invert_odom_to_base_tf': False,
            'rectified_images': True,
            'enable_image_denoising': False,
            'image_jitter_threshold_ms': 100.0,
            'image_qos': 'SENSOR_DATA',
            'imu_qos':   'SENSOR_DATA',
            'enable_debug_mode': False,
            'verbosity': 0,
        }],
        remappings=[
            ('visual_slam/image_0',       '/zedx_front/zed_node/left/gray/rect/image'),
            ('visual_slam/camera_info_0', '/zedx_front/zed_node/left/gray/rect/camera_info'),
            ('visual_slam/image_1',       '/zedx_front/zed_node/right/gray/rect/image'),
            ('visual_slam/camera_info_1', '/zedx_front/zed_node/right/gray/rect/camera_info'),
            ('visual_slam/image_2',       '/zedx_left/zed_node/left/gray/rect/image'),
            ('visual_slam/camera_info_2', '/zedx_left/zed_node/left/gray/rect/camera_info'),
            ('visual_slam/image_3',       '/zedx_left/zed_node/right/gray/rect/image'),
            ('visual_slam/camera_info_3', '/zedx_left/zed_node/right/gray/rect/camera_info'),
            ('visual_slam/image_4',       '/zedx_right/zed_node/left/gray/rect/image'),
            ('visual_slam/camera_info_4', '/zedx_right/zed_node/left/gray/rect/camera_info'),
            ('visual_slam/image_5',       '/zedx_right/zed_node/right/gray/rect/image'),
            ('visual_slam/camera_info_5', '/zedx_right/zed_node/right/gray/rect/camera_info'),
            ('visual_slam/imu',           '/zedx_front/zed_node/imu/data'),
        ],
        extra_arguments=[{'use_intra_process_comms': False}]  # NITROS handles its own IPC,
    )

    # ─── nvblox (FRONT camera depth only — B1 mode) ────────────────────────
    nvblox_node = ComposableNode(
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        name='nvblox_node',
        parameters=[
            nvblox_base_yaml,
            nvblox_zed_yaml,
            nvblox_manriix_yaml,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('camera_0/depth/image',       '/zedx_front/zed_node/depth/depth_registered'),
            ('camera_0/depth/camera_info', '/zedx_front/zed_node/depth/camera_info'),
            ('camera_0/color/image',       '/zedx_front/zed_node/left/color/rect/image'),
            ('camera_0/color/camera_info', '/zedx_front/zed_node/left/color/rect/camera_info'),
        ],
        extra_arguments=[{'use_intra_process_comms': False}]  # NITROS handles its own IPC,
    )

    # ────────────────────────────────────────────────────────────────────────
    # STEP 3 — Load components into the container, staggered
    # ────────────────────────────────────────────────────────────────────────

    # t=2s — Front (gives container time to spin up its LoadNode service)
    ld.add_action(TimerAction(period=2.0, actions=[
        LogInfo(msg='[LOAD t=2s] zedx_front (NEURAL_LIGHT depth + IMU)'),
        LoadComposableNodes(
            target_container=CONTAINER_FULL_NAME,
            composable_node_descriptions=[zedx_front_node],
        ),
    ]))

    # t=9s — Left
    ld.add_action(TimerAction(period=9.0, actions=[
        LogInfo(msg='[LOAD t=9s] zedx_left (gray only)'),
        LoadComposableNodes(
            target_container=CONTAINER_FULL_NAME,
            composable_node_descriptions=[zedx_left_node],
        ),
    ]))

    # t=16s — Right
    ld.add_action(TimerAction(period=16.0, actions=[
        LogInfo(msg='[LOAD t=16s] zedx_right (gray only)'),
        LoadComposableNodes(
            target_container=CONTAINER_FULL_NAME,
            composable_node_descriptions=[zedx_right_node],
        ),
    ]))

    # t=24s — cuVSLAM (after all 3 cameras streaming)
    ld.add_action(TimerAction(period=24.0, actions=[
        LogInfo(msg='[LOAD t=24s] visual_slam (cuVSLAM, 3-stereo VIO)'),
        LoadComposableNodes(
            target_container=CONTAINER_FULL_NAME,
            composable_node_descriptions=[visual_slam_node],
        ),
    ]))

    # t=26s — nvblox
    ld.add_action(TimerAction(period=26.0, actions=[
        LogInfo(msg='[LOAD t=26s] nvblox_node (depth integration + ESDF)'),
        LoadComposableNodes(
            target_container=CONTAINER_FULL_NAME,
            composable_node_descriptions=[nvblox_node],
        ),
    ]))

    # ────────────────────────────────────────────────────────────────────────
    # STEP 4 — Nav2 (separate process, delayed until stack is up)
    # ────────────────────────────────────────────────────────────────────────
    ld.add_action(TimerAction(period=35.0, actions=[
        LogInfo(msg='\n' + '=' * 60 + '\n'
            '[LOAD t=35s] Nav2 stack\n'
            '  Config:  ' + nav2_params_nvblox + '\n'
            '  Map:     /nvblox_node/static_map_slice\n'
            '  Odom:    /visual_slam/tracking/odometry\n' +
            '=' * 60),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file':  nav2_params_nvblox,
                'autostart':    'true',
            }.items(),
            condition=IfCondition(use_nav2),
        ),
    ]))

    # ────────────────────────────────────────────────────────────────────────
    # STEP 5 — Foxglove bridge (separate process)
    # ────────────────────────────────────────────────────────────────────────
    ld.add_action(Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': ['.*'],
            'param_whitelist': ['.*'],
            'service_whitelist': ['.*'],
            'client_topic_whitelist': ['.*'],
            'min_qos_depth': 1,
            'max_qos_depth': 10,
            'num_threads': 0,
            'send_buffer_limit': 10000000,
            'use_sim_time': use_sim_time,
            'capabilities': [
                'clientPublish',
                'parameters',
                'parametersSubscribe',
                'services',
                'connectionGraph',
                'assets',
            ],
        }],
        condition=IfCondition(use_foxglove),
    ))

    return ld
