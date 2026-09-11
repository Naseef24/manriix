#!/usr/bin/env python3
"""
================================================================================
Manriix cuVSLAM Launch File

Spawns Isaac ROS cuVSLAM (isaac_ros_visual_slam) configured for the 3× ZED X
camera rig. Expects the three zed_wrapper instances to already be running
(brought up by robot_nav_wrapper.launch.py).

Required Isaac ROS apt packages
-------------------------------
  ros-humble-isaac-ros-visual-slam            (the cuVSLAM node + GXF graph)
  ros-humble-isaac-ros-visual-slam-interfaces (Action/Service .msg + .srv types)
  ros-humble-isaac-ros-nitros                 (zero-copy GPU transport core)
  ros-humble-isaac-ros-managed-nitros         (managed NITROS pub/sub helpers)
  ros-humble-isaac-ros-nitros-image-type      (Image NITROS type bridge)
  ros-humble-isaac-ros-nitros-camera-info-type (CameraInfo NITROS bridge)
  ros-humble-isaac-ros-nitros-imu-type        (IMU NITROS bridge)
  ros-humble-isaac-ros-common                 (CMake helpers, GXF runtime)
  ros-humble-isaac-ros-gxf                    (NVIDIA GXF runtime libraries)

Install in one line:
  sudo apt install -y \\
      ros-humble-isaac-ros-visual-slam \\
      ros-humble-isaac-ros-visual-slam-interfaces \\
      ros-humble-isaac-ros-nitros \\
      ros-humble-isaac-ros-managed-nitros \\
      ros-humble-isaac-ros-nitros-image-type \\
      ros-humble-isaac-ros-nitros-camera-info-type \\
      ros-humble-isaac-ros-nitros-imu-type \\
      ros-humble-isaac-ros-common \\
      ros-humble-isaac-ros-gxf

Multi-camera configuration
--------------------------
cuVSLAM treats the camera rig as N stereo pairs. For Manriix we feed 3 pairs:

  Pair 0 (primary): zedx_front  — left+right rect + IMU + camera_info
  Pair 1:           zedx_left   — left+right rect + camera_info
  Pair 2:           zedx_right  — left+right rect + camera_info

The parameter `camera_optical_frames` is a flat list of 2N optical frame
strings in the order [left_0, right_0, left_1, right_1, ...]. cuVSLAM
reads the static transforms from base_frame to each of these via tf2.

Subscribed image topics are remapped via the `remappings=` table below.
cuVSLAM uses rectified GRAY images, not color (feature tracking only needs
intensity). The zed_wrapper publishes both *_rect_color and *_rect_gray
from the same SDK call, so no extra cost.

Usage
-----
After robot_nav_wrapper.launch.py is up and 3 camera topics confirmed
publishing, launch this on top:

  ros2 launch manriix_navigation cuvslam.launch.py

Verify with:
  ros2 topic hz /visual_slam/tracking/odometry
  ros2 topic echo /visual_slam/status

tf2 conflict warning
--------------------
cuVSLAM publishes map->odom and odom->base_link by default. SLAM Toolbox
also publishes map->odom. To avoid double-broadcast you MUST disable one
of them.

Default in this file: cuVSLAM publishes ONLY odom->base (publish_map_tf:
false, publish_odom_tf: true). SLAM Toolbox is left as the sole owner of
map->odom. Change with the publish_*_tf arguments below if you want the
opposite arrangement.

File: /home/hype/manriix2_ws/src/manriix_navigation/launch/cuvslam.launch.py
================================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ────────────────────────────────────────────────────────────────────────────
# GXF runtime library path — mandatory for Isaac ROS visual_slam.
#
# The visual_slam executable loads GXF extensions (.so files) at runtime
# from these paths. They're not in the system's default library search
# path, so without this LD_LIBRARY_PATH setting the node dies with:
#   "libgxf_serialization.so: cannot open shared object file"
# before it can even parse a launch parameter.
#
# Order doesn't matter; we list all directories that contain GXF .so files
# shipped with isaac_ros_gxf 3.2.x.
# ────────────────────────────────────────────────────────────────────────────
GXF_LIB_PATH = (
    '/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/serialization:'
    '/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/std:'
    '/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/cuda:'
    '/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/logger:'
    '/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/ipc/grpc'
)


def generate_launch_description():
    ld = LaunchDescription()

    # ── Environment ─────────────────────────────────────────────────────────
    ld.add_action(SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'))
    ld.add_action(SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'))

    # GXF library path — must be set BEFORE the visual_slam node spawns.
    # We prepend (not replace) so any existing LD_LIBRARY_PATH from the
    # parent shell (CUDA libs, ZED SDK libs, etc.) is preserved.
    ld.add_action(SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=[GXF_LIB_PATH + ':', os.environ.get('LD_LIBRARY_PATH', '')]
    ))

    # ── Launch arguments ────────────────────────────────────────────────────
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'))

    ld.add_action(DeclareLaunchArgument(
        'enable_imu_fusion', default_value='true',
        description='Fuse the ZED X front camera IMU into cuVSLAM'))

    ld.add_action(DeclareLaunchArgument(
        'enable_localization_n_mapping', default_value='true',
        description='cuVSLAM internal: build map while tracking'))

    ld.add_action(DeclareLaunchArgument(
        'publish_map_tf', default_value='false',
        description='cuVSLAM publishes map->odom. Leave FALSE if SLAM '
                    'Toolbox is owning that transform.'))

    ld.add_action(DeclareLaunchArgument(
        'publish_odom_tf', default_value='true',
        description='cuVSLAM publishes odom->base_link (replaces wheel odom)'))

    ld.add_action(DeclareLaunchArgument(
        'base_frame', default_value='base_footprint',
        description='The robot base frame cuVSLAM reports pose for. '
                    'Must match the URDF root frame and what SLAM Toolbox / '
                    'Nav2 / EKF expect, otherwise tf2 will report '
                    '"unconnected trees".'))

    ld.add_action(DeclareLaunchArgument(
        'odom_frame', default_value='odom',
        description='Odometry frame name'))

    ld.add_action(DeclareLaunchArgument(
        'map_frame', default_value='map',
        description='Map frame name'))

    ld.add_action(DeclareLaunchArgument(
        'imu_frame', default_value='zedx_front_imu_link',
        description='Frame ID of the IMU used for inertial fusion'))

    ld.add_action(DeclareLaunchArgument(
        'rectified_images', default_value='true',
        description='Input images are already rectified (ZED wrapper does it)'))

    ld.add_action(DeclareLaunchArgument(
        'image_qos', default_value='SENSOR_DATA',
        description='QoS for image inputs. SENSOR_DATA matches zed_wrapper '
                    'image_rect_gray topics'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_imu_fusion = LaunchConfiguration('enable_imu_fusion')
    enable_localization_n_mapping = LaunchConfiguration('enable_localization_n_mapping')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    rectified_images = LaunchConfiguration('rectified_images')
    image_qos = LaunchConfiguration('image_qos')

    ld.add_action(LogInfo(msg='\n' + '=' * 64 + '\n'
        '[MANRIIX cuVSLAM] Launching Isaac ROS Visual SLAM\n'
        '  Cameras: 3 ZED X (front primary + left + right)\n'
        '  IMU:     front camera (400 Hz hardware-timestamped)\n'
        '  TF:      cuVSLAM owns odom->base_link only\n'
        '           (SLAM Toolbox keeps map->odom)\n' +
        '=' * 64))

    # ── camera_optical_frames ───────────────────────────────────────────────
    # Order matters: [left0, right0, left1, right1, left2, right2]
    # These frame names must exist in your URDF tf tree.
    # The convention here matches your existing direct_zed_detection_node
    # ('zedx_*_left_camera_frame_optical'). If your URDF uses a different
    # naming pattern, edit this list to match.
    camera_optical_frames = [
        'zedx_front_left_camera_frame_optical',
        'zedx_front_right_camera_frame_optical',
        'zedx_left_left_camera_frame_optical',
        'zedx_left_right_camera_frame_optical',
        'zedx_right_left_camera_frame_optical',
        'zedx_right_right_camera_frame_optical',
    ]

    # ── Topic remappings ────────────────────────────────────────────────────
    # cuVSLAM expects topics in a fixed pattern:
    #   visual_slam/image_{i}        — N image streams
    #   visual_slam/camera_info_{i}  — matching CameraInfo for each
    #   visual_slam/imu              — single IMU input
    # Index order MUST match camera_optical_frames above.
    #
    # IMPORTANT: SDK 5.3 wrapper topic paths are:
    #   /<ns>/zed_node/{left|right}/gray/rect/image
    #   /<ns>/zed_node/{left|right}/gray/rect/camera_info
    # NOT /image_rect_gray as in older wrapper versions.
    remappings = [
        # Pair 0 — FRONT (primary, contributes IMU)
        ('visual_slam/image_0',       '/zedx_front/zed_node/left/gray/rect/image'),
        ('visual_slam/camera_info_0', '/zedx_front/zed_node/left/gray/rect/camera_info'),
        ('visual_slam/image_1',       '/zedx_front/zed_node/right/gray/rect/image'),
        ('visual_slam/camera_info_1', '/zedx_front/zed_node/right/gray/rect/camera_info'),

        # Pair 1 — LEFT side camera (wider visual coverage)
        ('visual_slam/image_2',       '/zedx_left/zed_node/left/gray/rect/image'),
        ('visual_slam/camera_info_2', '/zedx_left/zed_node/left/gray/rect/camera_info'),
        ('visual_slam/image_3',       '/zedx_left/zed_node/right/gray/rect/image'),
        ('visual_slam/camera_info_3', '/zedx_left/zed_node/right/gray/rect/camera_info'),

        # Pair 2 — RIGHT side camera
        ('visual_slam/image_4',       '/zedx_right/zed_node/left/gray/rect/image'),
        ('visual_slam/camera_info_4', '/zedx_right/zed_node/left/gray/rect/camera_info'),
        ('visual_slam/image_5',       '/zedx_right/zed_node/right/gray/rect/image'),
        ('visual_slam/camera_info_5', '/zedx_right/zed_node/right/gray/rect/camera_info'),

        # IMU — front camera only (cuVSLAM fuses one IMU)
        ('visual_slam/imu',           '/zedx_front/zed_node/imu/data'),
    ]

    # ── Visual SLAM Node ────────────────────────────────────────────────────
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            # ── Frames ─────────────────────────────────────────────────────
            'base_frame': base_frame,
            'odom_frame': odom_frame,
            'map_frame': map_frame,
            'imu_frame': imu_frame,
            'camera_optical_frames': camera_optical_frames,

            # ── Camera count ───────────────────────────────────────────────
            # 6 image streams = 3 stereo pairs (front, left, right)
            # multicam_mode lets cuVSLAM treat them as independent pairs
            # rather than one giant rig.
            'num_cameras': 6,
            'multicam_mode': 0,

            # ── IMU (front camera only) ────────────────────────────────────
            'enable_imu_fusion': enable_imu_fusion,
            # ZED X IMU noise model — verify against datasheet for production
            'gyro_noise_density': 0.000244,
            'gyro_random_walk':   0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk':   0.003,

            # ── SLAM behaviour ─────────────────────────────────────────────
            'enable_localization_n_mapping': enable_localization_n_mapping,
            'enable_slam_visualization': True,
            'enable_observations_view': True,
            'enable_landmarks_view': True,

            # ── TF broadcasting ───────────────────────────────────────────
            # publish_map_tf default = false → SLAM Toolbox keeps map→odom
            # publish_odom_tf default = true → cuVSLAM owns odom→base_link
            'publish_map_to_odom_tf': publish_map_tf,
            'publish_odom_to_base_tf': publish_odom_tf,
            'invert_map_to_odom_tf': False,
            'invert_odom_to_base_tf': False,

            # ── Image processing ──────────────────────────────────────────
            'rectified_images': rectified_images,
            'enable_image_denoising': False,
            'image_jitter_threshold_ms': 100.0,
            'image_qos': image_qos,
            'imu_qos': 'SENSOR_DATA',

            # ── Diagnostics ───────────────────────────────────────────────
            'enable_debug_mode': False,
            'verbosity': 0,
        }],
        remappings=remappings,
        # No respawn — if cuVSLAM crashes we want to see why, not silently
        # restart with potentially the same fault.
    )

    ld.add_action(visual_slam_node)

    return ld