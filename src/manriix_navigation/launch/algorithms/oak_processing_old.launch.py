#!/usr/bin/env python3
"""
================================================================================
Manriix Algorithm — OAK-D Pro W Processing + Ground Segmentation
Carter-style sibling to nvblox.launch.py and vslam.launch.py.

Purpose: bring up the OAK-D Pro W camera, QoS bridge, AND the PCL RANSAC
ground segmentation node. Output topics:
  /oak/points              (PointCloud2, BEST_EFFORT,  ~22 Hz)  raw
  /oak/points_reliable     (PointCloud2, RELIABLE,     ~22 Hz)  raw post-QoS-bridge
  /oak/points_obstacles    (PointCloud2, RELIABLE,     ~22 Hz)  RANSAC non-ground
  /oak/points_ground       (PointCloud2, RELIABLE,     ~22 Hz)  RANSAC ground (debug)

The /oak/points_obstacles topic is consumed by the OAK STVL voxel layer in
Nav2 (oak_voxel_layer in nav2_params_carter.yaml — local + global costmaps +
collision_monitor).

Chain:
  Includes oak_pointcloud_bridged.launch.py from manriix_description:
    - OAK driver via oak_xyz_only.launch.py (custom — avoids upstream OpenCV
      remap crash in image_proc::Rectify)
    - QoS bridge: /oak/points (BEST_EFFORT) → /oak/points_reliable (RELIABLE)
  Then spawns oak_ground_remover_node directly at t=14s.

Frame info:
  /oak/points* clouds publish in: oak_rgb_camera_optical_frame
  (NOT oak_d_pro_w_* — the depthai driver's stock naming. URDF / TF should
   resolve this via the camera_as_part_of_a_robot pattern or static TF.)

Load timing:
  t=0   OAK driver starts (via oak_xyz_only.launch.py)
  t=8   qos_bridge starts
  t=14  oak_ground_remover_node starts

File: ~/manriix2_ws/src/manriix_navigation/launch/algorithms/oak_processing.launch.py
================================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('manriix_description')

    # -----------------------------------------------------------------
    # Path to OAK + qos_bridge launch (owned by manriix_description)
    # -----------------------------------------------------------------
    oak_launch_path = os.path.join(
        pkg_description, 'launch', 'oak_pointcloud_bridged.launch.py'
    )

    # -----------------------------------------------------------------
    # Arguments
    # -----------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
    )

    # -----------------------------------------------------------------
    # Step 1: Include OAK driver + qos_bridge (starts immediately)
    # -----------------------------------------------------------------
    oak_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oak_launch_path),
    )

    # -----------------------------------------------------------------
    # Step 2: Start oak_ground_remover_node at t=14s
    # qos_bridge starts at t=8s inside oak_pointcloud_bridged.launch.py.
    # We wait until t=14s to ensure /oak/points_reliable is alive.
    # -----------------------------------------------------------------
    ground_remover = TimerAction(
        period=14.0,
        actions=[
            LogInfo(msg=[
                '\n', '=' * 70, '\n',
                '[OAK PROCESSING] Starting oak_ground_remover (RANSAC)\n',
                '  Input:           /oak/points_reliable\n',
                '  Output (obs):    /oak/points_obstacles  → STVL\n',
                '  Output (ground): /oak/points_ground     (debug)\n',
                '=' * 70,
            ]),
            Node(
                package='manriix_description',
                executable='oak_ground_remover_node',
                name='oak_ground_remover',
                output='screen',
                parameters=[{
                    'input_topic': '/oak/points_reliable',
                    'obstacles_topic': '/oak/points_obstacles',
                    'ground_topic': '/oak/points_ground',
                    'publish_ground': True,

                    'voxel_leaf_size': 0.05,
                    'plane_distance_threshold': 0.05,
                    'max_iterations': 100,
                    'epsilon_angle_deg': 15.0,
                    'min_inliers': 200,

                    # ---- Range / spatial clipping (optical frame) ----
                    'min_range':           0.20,    # Z min
                    'max_range':           4.00,    # Z max
                    'floor_clip_m':        0.40,    # Y max (below camera)
                    'ceiling_clip_m':      1.50,    # |Y| above camera
                    'obstacle_max_range':  4.00,    # final Z cap on obstacles

                    # ---- Euclidean clustering (TIGHTENED to kill flicker) ----
                    # Diagnostic on empty floor showed 30+ point noise clusters
                    # passing through at 30 Hz. Stricter values needed:
                    'cluster_tolerance':   0.07,    # was 0.10 — tighter grouping
                    'cluster_min_size':    80,      # was 30 — bigger blob required
                    'cluster_max_size':    25000,

                    # Axis along which plane normal must lie.
                    # OAK rgb_camera_optical_frame:
                    #   X=right, Y=down, Z=forward
                    # Floor normal points up = -Y in optical frame.
                    'axis_x': 0.0,
                    'axis_y': -1.0,
                    'axis_z': 0.0,

                    'stats_log_period_sec': 5.0,
                }],
            ),
        ]
    )

    # -----------------------------------------------------------------
    # Assemble
    # -----------------------------------------------------------------
    return LaunchDescription([
        use_sim_time_arg,
        LogInfo(msg=[
            '\n', '=' * 70, '\n',
            '[OAK PROCESSING] Including: ', oak_launch_path, '\n',
            '  Output topics: /oak/points, /oak/points_reliable,\n',
            '                 /oak/points_obstacles, /oak/points_ground\n',
            '  Frame: oak_rgb_camera_optical_frame\n',
            '  Consumed by: oak_voxel_layer in Nav2 (STVL plugin)\n',
            '=' * 70,
        ]),
        oak_launch,
        ground_remover,
    ])