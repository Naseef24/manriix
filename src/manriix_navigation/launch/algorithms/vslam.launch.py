#!/usr/bin/env python3
"""
================================================================================
Manriix Algorithm — cuVSLAM

Purpose
-------
Load Isaac ROS Visual SLAM into /manriix_container.

Recommended production configuration
------------------------------------
Use the front ZED X stereo camera only:

    vslam_cameras:=zedx_front

This creates two camera inputs:

    image_0 / camera_info_0 -> front left rectified grayscale
    image_1 / camera_info_1 -> front right rectified grayscale

The front ZED IMU is fused with the stereo pair. The same front camera may
simultaneously provide colour and depth to nvblox.

The launch remains camera-list driven so a controlled multi-camera experiment
can still be run later. For normal navigation, perceptor_include.launch.py
should resolve:

    enabled_cameras = zedx_front
    vslam_cameras = zedx_front
    nvblox_cameras = zedx_front

File:
  ~/manriix2_ws/src/manriix_navigation/launch/algorithms/vslam.launch.py
================================================================================
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


CONTAINER_NAME = "/manriix_container"
VSLAM_LOAD_TIME = 24.0


def zed_topic(camera_name: str, side: str, info: bool = False) -> str:
    """Return a raw rectified grayscale image or CameraInfo topic."""
    suffix = "camera_info" if info else "image"
    return f"/{camera_name}/zed_node/{side}/gray/rect/{suffix}"


def build_remappings_and_frames(cameras):
    """
    Build cuVSLAM remappings and optical-frame names.

    Every physical ZED contributes two image slots:
      slot 2*i     -> left image
      slot 2*i + 1 -> right image
    """
    remappings = [
        ("visual_slam/imu", "/zedx_front/zed_node/imu/data"),
    ]
    optical_frames = []

    for index, camera_name in enumerate(cameras):
        left_slot = 2 * index
        right_slot = left_slot + 1

        remappings.extend(
            [
                (
                    f"visual_slam/image_{left_slot}",
                    zed_topic(camera_name, "left"),
                ),
                (
                    f"visual_slam/camera_info_{left_slot}",
                    zed_topic(camera_name, "left", info=True),
                ),
                (
                    f"visual_slam/image_{right_slot}",
                    zed_topic(camera_name, "right"),
                ),
                (
                    f"visual_slam/camera_info_{right_slot}",
                    zed_topic(camera_name, "right", info=True),
                ),
            ]
        )

        optical_frames.extend(
            [
                f"{camera_name}_left_camera_frame_optical",
                f"{camera_name}_right_camera_frame_optical",
            ]
        )

    return remappings, optical_frames


def build_vslam_node(context, *args, **kwargs):
    """Construct cuVSLAM from the camera list resolved by the orchestrator."""
    del args, kwargs

    camera_string = LaunchConfiguration("vslam_cameras").perform(context)
    enable_wheel_odometry = (
        LaunchConfiguration("enable_wheel_odometry")
        .perform(context)
        .lower()
        == "true"
    )

    if not camera_string.strip():
        return [
            LogInfo(msg="[VSLAM] No cameras configured. Skipping cuVSLAM."),
        ]

    cameras = [
        item.strip()
        for item in camera_string.split(",")
        if item.strip()
    ]

    num_cameras = 2 * len(cameras)
    remappings, optical_frames = build_remappings_and_frames(cameras)

    # For front-only this resolves to two images, so both members of the
    # physical stereo pair are required for every tracking update.
    min_num_images = num_cameras

    publish_odom_to_base_tf = not enable_wheel_odometry

    visual_slam_node = ComposableNode(
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        name="visual_slam",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),

                # Frames
                "base_frame": "base_footprint",
                "odom_frame": "odom",
                "map_frame": "map",
                "imu_frame": "zedx_front_imu_link",
                "camera_optical_frames": optical_frames,

                # Camera synchronization
                "num_cameras": num_cameras,
                "min_num_images": min_num_images,
                "multicam_mode": 0,
                "sync_matching_threshold_ms": 5.0,

                # IMU fusion
                "enable_imu_fusion": True,
                "gyro_noise_density": 0.000244,
                "gyro_random_walk": 0.000019393,
                "accel_noise_density": 0.001862,
                "accel_random_walk": 0.003,
                # The direct driver has measured approximately 40–50 Hz.
                "calibration_frequency": 50.0,

                # SLAM
                "enable_localization_n_mapping": True,
                "enable_slam_visualization": True,
                "enable_observations_view": True,
                "enable_landmarks_view": True,

                # Planar wheeled robot constraints
                "enable_ground_constraint_in_odometry": True,
                "enable_ground_constraint_in_slam": True,

                # TF ownership
                "publish_map_to_odom_tf": True,
                "publish_odom_to_base_tf": publish_odom_to_base_tf,
                "invert_map_to_odom_tf": False,
                "invert_odom_to_base_tf": False,

                # Images and QoS
                "rectified_images": True,
                "enable_image_denoising": False,
                # This is a warning threshold, not a synchronization window.
                # 250 ms accommodates occasional Python-driver scheduling gaps
                # while still exposing significant stalls.
                "image_jitter_threshold_ms": 250.0,
                "image_qos": "SENSOR_DATA",
                "imu_qos": "SENSOR_DATA",

                # Debug
                "enable_debug_mode": False,
                "verbosity": 0,
            }
        ],
        remappings=remappings,
        extra_arguments=[{"use_intra_process_comms": False}],
    )

    return [
        TimerAction(
            period=VSLAM_LOAD_TIME,
            actions=[
                LogInfo(
                    msg=[
                        "\n",
                        "=" * 70,
                        "\n[VSLAM LOAD t=",
                        str(VSLAM_LOAD_TIME),
                        "s]\n",
                        "  Cameras:           ",
                        camera_string,
                        "\n  Image slots:       ",
                        str(num_cameras),
                        "\n  Minimum images:    ",
                        str(min_num_images),
                        "\n  Sync threshold:    5.0 ms\n",
                        "  Ground constraint: True\n",
                        "  Wheel odometry:    ",
                        str(enable_wheel_odometry),
                        "\n  Publishes odom→base_footprint TF: ",
                        str(publish_odom_to_base_tf),
                        "\n",
                        "=" * 70,
                    ]
                ),
                LoadComposableNodes(
                    target_container=CONTAINER_NAME,
                    composable_node_descriptions=[visual_slam_node],
                ),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vslam_cameras",
                default_value="zedx_front",
                description=(
                    "Comma-separated cameras supplied to cuVSLAM. "
                    "Use zedx_front for the recommended production setup."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
            ),
            DeclareLaunchArgument(
                "enable_wheel_odometry",
                default_value="False",
                description=(
                    "When True, wheel odometry owns odom→base_footprint. "
                    "When False, cuVSLAM publishes that transform."
                ),
            ),
            OpaqueFunction(function=build_vslam_node),
        ]
    )





# ==========================================
# #!/usr/bin/env python3
# """
# ================================================================================
# Manriix Algorithm — cuVSLAM (multi-camera visual SLAM)

# Modeled on:
#   isaac_ros_perceptor_bringup/launch/algorithms/vslam.launch.py

# Purpose: load the Isaac ROS Visual SLAM node into the shared composable
# container, configured for multi-camera operation.

# Currently loads at t=24s (after all cameras are streaming). This timing
# mirrors the production cuvslam_for_nvblox.launch.py.

# Camera slots:
#   Each camera contributes 2 image slots (left + right). For N cameras,
#   num_cameras = 2*N image streams.

#   vslam_cameras='zedx_front'                          → 2 slots (slots 0,1)
#   vslam_cameras='zedx_front,zedx_left,zedx_right'     → 6 slots (slots 0–5)

# Topic remappings:
#   visual_slam/image_{2i}        → /<camera_i>/zed_node/left/gray/rect/image
#   visual_slam/camera_info_{2i}  → /<camera_i>/zed_node/left/gray/rect/camera_info
#   visual_slam/image_{2i+1}      → /<camera_i>/zed_node/right/gray/rect/image
#   visual_slam/camera_info_{2i+1}→ /<camera_i>/zed_node/right/gray/rect/camera_info

# Improvements over previous cuvslam_for_nvblox.launch.py (Carter-inspired):
#   - enable_ground_constraint_in_odometry: True  ← wheeled robot benefit
#   - enable_ground_constraint_in_slam: True      ← wheeled robot benefit
#   - enable_wheel_odometry flag: when True, cuVSLAM stops publishing
#     odom->base_footprint TF (wheel odom owns it). Default False = cuVSLAM
#     owns TF (matches current behavior).

# IMU: only zedx_front has IMU enabled, so vslam IMU is hardwired to its topic.

# File: ~/manriix2_ws/src/manriix_navigation/launch/algorithms/vslam.launch.py
# ================================================================================
# """

# import os

# from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     LogInfo,
#     OpaqueFunction,
#     TimerAction,
# )
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import LoadComposableNodes
# from launch_ros.descriptions import ComposableNode


# CONTAINER_NAME = '/manriix_container'

# # t=24s — matches cuvslam_for_nvblox.launch.py (after cameras streaming)
# VSLAM_LOAD_TIME = 24.0


# # ZED-X topic naming pattern (this is your current setup)
# def zed_topic(camera_name, side, info=False):
#     """Generate ZED-X gray rect image or camera_info topic name."""
#     suffix = 'camera_info' if info else 'image'
#     return f'/{camera_name}/zed_node/{side}/gray/rect/{suffix}'


# def build_remappings_and_frames(cameras):
#     """
#     For each camera in the list, generate:
#       - 4 remappings (left+right image + 2 camera_info)
#       - 2 optical frame names (left + right)

#     Returns (remappings, camera_optical_frames).
#     """
#     remappings = [
#         # IMU (only front camera has it)
#         ('visual_slam/imu', '/zedx_front/zed_node/imu/data'),
#     ]
#     camera_optical_frames = []

#     for i, name in enumerate(cameras):
#         # Image slots
#         remappings.append(
#             (f'visual_slam/image_{2*i}',       zed_topic(name, 'left')))
#         remappings.append(
#             (f'visual_slam/camera_info_{2*i}', zed_topic(name, 'left', info=True)))
#         remappings.append(
#             (f'visual_slam/image_{2*i+1}',       zed_topic(name, 'right')))
#         remappings.append(
#             (f'visual_slam/camera_info_{2*i+1}', zed_topic(name, 'right', info=True)))

#         # Optical frames (used by VSLAM for TF lookups)
#         camera_optical_frames.append(f'{name}_left_camera_frame_optical')
#         camera_optical_frames.append(f'{name}_right_camera_frame_optical')

#     return remappings, camera_optical_frames


# def build_vslam_node(context, *args, **kwargs):
#     """
#     OpaqueFunction: build the cuVSLAM ComposableNode based on the
#     enabled camera list (resolved at launch time).
#     """
#     vslam_cameras_str = LaunchConfiguration(
#         'vslam_cameras').perform(context)
#     enable_wheel_odom = LaunchConfiguration(
#         'enable_wheel_odometry').perform(context).lower() == 'true'

#     if not vslam_cameras_str.strip():
#         return [
#             LogInfo(msg='[VSLAM] No cameras configured for VSLAM. Skipping.'),
#         ]

#     cameras = [c.strip() for c in vslam_cameras_str.split(',') if c.strip()]
#     num_cameras = 2 * len(cameras)

#     remappings, optical_frames = build_remappings_and_frames(cameras)

#     # Carter pattern: when wheel odom is used, cuVSLAM stops publishing
#     # odom->base_footprint TF. Wheel odom (ros2_control diff_drive) owns it.
#     publish_odom_to_base_tf = not enable_wheel_odom

#     visual_slam_node = ComposableNode(
#         package='isaac_ros_visual_slam',
#         plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
#         name='visual_slam',
#         parameters=[{
#             'use_sim_time': LaunchConfiguration('use_sim_time'),

#             # Frames
#             'base_frame':   'base_footprint',
#             'odom_frame':   'odom',
#             'map_frame':    'map',
#             'imu_frame':    'zedx_front_imu_link',
#             'camera_optical_frames': optical_frames,

#             # Camera count
#             'num_cameras':   num_cameras,
#             'min_num_images': num_cameras,
#             'multicam_mode': 0,

#             # IMU fusion
#             'enable_imu_fusion':      True,
#             'gyro_noise_density':     0.000244,
#             'gyro_random_walk':       0.000019393,
#             'accel_noise_density':    0.001862,
#             'accel_random_walk':      0.003,
#             'calibration_frequency':  50.0, #200

#             # SLAM
#             'enable_localization_n_mapping': True,
#             'enable_slam_visualization':     True,
#             'enable_observations_view':      True,
#             'enable_landmarks_view':         True,

#             # Carter improvement: ground constraint (wheeled robot)
#             'enable_ground_constraint_in_odometry': True,
#             'enable_ground_constraint_in_slam':     True,

#             # TF publishing
#             'publish_map_to_odom_tf':  True,
#             'publish_odom_to_base_tf': publish_odom_to_base_tf,
#             'invert_map_to_odom_tf':   False,
#             'invert_odom_to_base_tf':  False,

#             # Image handling
#             'rectified_images':           True,   # ZED publishes pre-rectified
#             'enable_image_denoising':     False,
#             'image_jitter_threshold_ms':  200.0,  #100.0
#             'image_qos':                  'SENSOR_DATA',
#             'imu_qos':                    'SENSOR_DATA',

#             # Debug
#             'enable_debug_mode': False,
#             'verbosity':         0,
#         }],
#         remappings=remappings,
#         # NITROS handles IPC internally; ROS2-level IPC must be False
#         extra_arguments=[{'use_intra_process_comms': False}],
#     )

#     return [
#         TimerAction(
#             period=VSLAM_LOAD_TIME,
#             actions=[
#                 LogInfo(msg=[
#                     '\n', '=' * 70, '\n',
#                     '[VSLAM LOAD t=', str(VSLAM_LOAD_TIME), 's] cuVSLAM\n',
#                     '  Cameras:           ', vslam_cameras_str, '\n',
#                     '  Image slots:       ', str(num_cameras), '\n',
#                     '  Ground constraint: True\n',
#                     '  Wheel odometry:    ', str(enable_wheel_odom), '\n',
#                     '  Publishes odom→base_footprint TF: ',
#                     str(publish_odom_to_base_tf), '\n',
#                     '=' * 70,
#                 ]),
#                 LoadComposableNodes(
#                     target_container=CONTAINER_NAME,
#                     composable_node_descriptions=[visual_slam_node],
#                 ),
#             ],
#         ),
#     ]


# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'vslam_cameras',
#             default_value='zedx_front,zedx_left,zedx_right',
#             description='Comma-separated cameras to feed cuVSLAM.',
#         ),
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='False',
#         ),
#         DeclareLaunchArgument(
#             'enable_wheel_odometry',
#             default_value='False',
#             description=(
#                 'If True, cuVSLAM stops publishing odom→base_footprint TF '
#                 '(wheel odom owns it). If False, cuVSLAM owns TF chain.'
#             ),
#         ),

#         OpaqueFunction(function=build_vslam_node),
#     ])