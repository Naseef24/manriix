#!/usr/bin/env python3
"""
================================================================================
Manriix Algorithm — nvblox 3D Reconstruction

Modeled on:
  isaac_ros_perceptor_bringup/launch/algorithms/nvblox.launch.py

Purpose: load the nvblox node into the shared composable container.
Supports multiple cameras via the camera_N remapping pattern.

YAML stack (loaded in order — later files override earlier):
  1. nvblox_examples_bringup/config/nvblox/nvblox_base.yaml         (NVIDIA defaults)
  2. nvblox_examples_bringup/config/nvblox/specializations/nvblox_zed.yaml (ZED-specific)
  3. manriix_navigation/config/nvblox/nvblox_manriix_dynamic.yaml   (Manriix overrides)
  4. {'num_cameras': N}  (inline, set from camera list)

Multi-camera pattern (Carter):
  For each camera_i in nvblox_cameras list, generate 4 remappings:
    camera_{i}/depth/image        → /<name>/zed_node/depth/depth_registered
    camera_{i}/depth/camera_info  → /<name>/zed_node/depth/camera_info
    camera_{i}/color/image        → /<name>/zed_node/left/color/rect/image
    camera_{i}/color/camera_info  → /<name>/zed_node/left/color/rect/camera_info

  Currently default: nvblox_cameras='zedx_front' (single camera, matches
  current production). Adding more cameras to the list is the future
  enhancement path (Phase 2+ continuation).

Load timing: t=26s (after cuVSLAM has started at t=24s).

File: ~/manriix2_ws/src/manriix_navigation/launch/algorithms/nvblox.launch.py
================================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
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


CONTAINER_NAME = '/manriix_container'

# t=26s — matches cuvslam_for_nvblox.launch.py (after cuVSLAM is up)
NVBLOX_LOAD_TIME = 26.0


# -----------------------------------------------------------------------
# ZED depth/color topic naming (per ZED-X wrapper conventions)
# -----------------------------------------------------------------------
def zed_nvblox_topics(camera_name):
    """Return the 4 topics nvblox needs for one ZED-X camera."""
    base = f'/{camera_name}/zed_node'
    return {
        'depth_image':       f'{base}/depth/depth_registered',
        'depth_camera_info': f'{base}/depth/camera_info',
        'color_image':       f'{base}/left/color/rect/image',
        'color_camera_info': f'{base}/left/color/rect/camera_info',
    }


def build_nvblox_remappings(cameras):
    """
    Build the (internal_name, external_topic) remappings for nvblox.
    Carter pattern: camera_0, camera_1, ... regardless of camera identity.
    """
    remappings = []
    for i, name in enumerate(cameras):
        topics = zed_nvblox_topics(name)
        remappings.append(
            (f'camera_{i}/depth/image',       topics['depth_image']))
        remappings.append(
            (f'camera_{i}/depth/camera_info', topics['depth_camera_info']))
        remappings.append(
            (f'camera_{i}/color/image',       topics['color_image']))
        remappings.append(
            (f'camera_{i}/color/camera_info', topics['color_camera_info']))
    return remappings


def build_nvblox_node(context, *args, **kwargs):
    """
    OpaqueFunction: build the nvblox ComposableNode after resolving
    the nvblox_cameras list.
    """
    nvblox_cameras_str = LaunchConfiguration(
        'nvblox_cameras').perform(context)

    if not nvblox_cameras_str.strip():
        return [
            LogInfo(msg='[NVBLOX] No cameras configured for nvblox. Skipping.'),
        ]

    cameras = [c.strip() for c in nvblox_cameras_str.split(',') if c.strip()]
    num_cameras = len(cameras)

    # -----------------------------------------------------------------
    # YAML config stack
    # -----------------------------------------------------------------
    pkg_nvblox_examples = get_package_share_directory('nvblox_examples_bringup')
    pkg_navigation = get_package_share_directory('manriix_navigation')

    nvblox_base_yaml    = os.path.join(
        pkg_nvblox_examples, 'config', 'nvblox', 'nvblox_base.yaml')
    nvblox_zed_yaml     = os.path.join(
        pkg_nvblox_examples, 'config', 'nvblox',
        'specializations', 'nvblox_zed.yaml')
    nvblox_manriix_yaml = os.path.join(
        pkg_navigation, 'config', 'nvblox', 'nvblox_manriix_dynamic.yaml')

    # -----------------------------------------------------------------
    # Build the nvblox composable node
    # -----------------------------------------------------------------
    remappings = build_nvblox_remappings(cameras)

    nvblox_node = ComposableNode(
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        name='nvblox_node',
        parameters=[
            nvblox_base_yaml,
            nvblox_zed_yaml,
            nvblox_manriix_yaml,
            {
                'num_cameras':  num_cameras,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings=remappings,
        # NITROS handles IPC; ROS2 IPC=False mandatory
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    return [
        TimerAction(
            period=NVBLOX_LOAD_TIME,
            actions=[
                LogInfo(msg=[
                    '\n', '=' * 70, '\n',
                    '[NVBLOX LOAD t=', str(NVBLOX_LOAD_TIME), 's] nvblox_node\n',
                    '  Cameras feeding nvblox: ', nvblox_cameras_str, '\n',
                    '  num_cameras:            ', str(num_cameras), '\n',
                    '  YAML stack:\n',
                    '    1. ', nvblox_base_yaml, '\n',
                    '    2. ', nvblox_zed_yaml, '\n',
                    '    3. ', nvblox_manriix_yaml, '\n',
                    '=' * 70,
                ]),
                LoadComposableNodes(
                    target_container=CONTAINER_NAME,
                    composable_node_descriptions=[nvblox_node],
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'nvblox_cameras',
            default_value='zedx_front',
            description='Comma-separated cameras to feed nvblox. '
                        'Current production: zedx_front only. '
                        'Future: zedx_front,zedx_left,zedx_right (multi-cam).',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
        ),

        OpaqueFunction(function=build_nvblox_node),
    ])