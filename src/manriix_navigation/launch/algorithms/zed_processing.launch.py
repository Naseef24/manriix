#!/usr/bin/env python3
"""
================================================================================
Manriix Algorithm — ZED-X Camera Processing

Modeled on:
  isaac_ros_perceptor_bringup/launch/algorithms/hawks_processing.launch.py

Purpose: launch ZED-X stereo cameras with staggered timing to avoid
Argus race conditions when opening multiple cameras simultaneously
in the same composable container.

Each camera is loaded into the shared composable container as a
separate ComposableNode descriptor. The staggered TimerAction
intervals (2s / 9s / 16s) are verified-working values from your
existing production launch (cuvslam_for_nvblox.launch.py).

YAML configs per camera (must exist in manriix_bringup/config/):
  - zedx_front_depth.yaml   (depth ON, IMU on — feeds nvblox + cuVSLAM)
  - zedx_left_nodepth.yaml  (depth OFF — feeds cuVSLAM only)
  - zedx_right_nodepth.yaml (depth OFF — feeds cuVSLAM only)

Container:
  - Target: /manriix_container (created in navigation.launch.py)

CRITICAL Jetson + ZED quirks (from project memory):
  - ZED NEURAL depth modes REQUIRE a GPU-backed X display.
  - Without DISPLAY=:0, Argus crashes at ScopedNvGlsiEglImageRef::reset().
  - This is set in navigation.launch.py (DISPLAY=:0, XAUTHORITY=...).

File: ~/manriix2_ws/src/manriix_navigation/launch/algorithms/zed_processing.launch.py
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

# Verified-working staggered load times (mirrors cuvslam_for_nvblox.launch.py)
CAMERA_LOAD_TIMING = {
    'zedx_front': 2.0,
    'zedx_left':  9.0,
    'zedx_right': 16.0,
}


def build_zed_composable_node(camera_name, config_yaml_path):
    """Build a ComposableNode descriptor for a single ZED-X camera."""
    return ComposableNode(
        package='zed_components',
        namespace=camera_name,
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=[
            config_yaml_path,
            {
                'debug.disable_nitros': False,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )


def stage_cameras(context, *args, **kwargs):
    """
    OpaqueFunction: parse the enabled_cameras list, then create staggered
    TimerAction loaders for each enabled camera into the shared container.
    """
    enabled_cameras_str = LaunchConfiguration(
        'enabled_cameras').perform(context)

    # Empty string check (in case no cameras enabled)
    if not enabled_cameras_str.strip():
        return [
            LogInfo(msg='[ZED] No cameras enabled. Nothing to launch.'),
        ]

    enabled_cameras = [c.strip() for c in enabled_cameras_str.split(',')
                       if c.strip()]

    pkg_bringup = get_package_share_directory('manriix_bringup')

    # Map camera name -> YAML config path
    config_yaml_map = {
        'zedx_front': os.path.join(pkg_bringup, 'config', 'zedx_front_depth.yaml'),
        'zedx_left':  os.path.join(pkg_bringup, 'config', 'zedx_left_nodepth.yaml'),
        'zedx_right': os.path.join(pkg_bringup, 'config', 'zedx_right_nodepth.yaml'),
    }

    actions = []
    actions.append(LogInfo(msg=[
        '\n', '=' * 70, '\n',
        '[ZED PROCESSING] Loading ', str(len(enabled_cameras)),
        ' camera(s) into ', CONTAINER_NAME, '\n',
        '=' * 70,
    ]))

    for camera_name in enabled_cameras:
        if camera_name not in config_yaml_map:
            actions.append(LogInfo(
                msg=f'[ZED] WARNING: unknown camera "{camera_name}". Skipping.'
            ))
            continue

        config_path = config_yaml_map[camera_name]
        load_time = CAMERA_LOAD_TIMING.get(camera_name, 5.0)

        zed_node = build_zed_composable_node(camera_name, config_path)

        actions.append(TimerAction(
            period=load_time,
            actions=[
                LogInfo(msg=[
                    '[ZED LOAD t=', str(load_time), 's] ', camera_name,
                    '   config: ', config_path,
                ]),
                LoadComposableNodes(
                    target_container=CONTAINER_NAME,
                    composable_node_descriptions=[zed_node],
                ),
            ],
        ))

    return actions


def generate_launch_description():
    # -----------------------------------------------------------------
    # Arguments
    # -----------------------------------------------------------------
    enabled_cameras_arg = DeclareLaunchArgument(
        'enabled_cameras',
        default_value='zedx_front,zedx_left,zedx_right',
        description='Comma-separated list of ZED cameras to enable.',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation time.',
    )

    return LaunchDescription([
        enabled_cameras_arg,
        use_sim_time_arg,

        # Stage the cameras (built by OpaqueFunction so we can parse the list)
        OpaqueFunction(function=stage_cameras),
    ])