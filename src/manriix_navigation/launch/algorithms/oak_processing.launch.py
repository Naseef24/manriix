#!/usr/bin/env python3
"""
================================================================================
Manriix Algorithm — OAK-D Pro W Processing (dynamic obstacle pipeline)
Carter-style sibling to nvblox.launch.py and vslam.launch.py.

REPLACES the previous payload (oak_pointcloud_bridged + qos_bridge +
oak_ground_remover_node RANSAC). Now brings up the manriix_oak_obstacles
pipeline — an implementation of Eppenberger et al., IROS 2020
("Leveraging Stereo-Camera Data for Real-Time Dynamic Obstacle Detection
and Tracking") adapted to the OAK-D Pro W:

  oak_camera_prod.launch.py:
    depthai_ros_driver::Camera (name=oak_d_pro_w, bare component:
      URDF/RSP owns TF; no rectify) + on-device spatial MobileNet-SSD
    depth_image_proc PointCloudXyz -> /oak_d_pro_w/points

  manriix_pipeline_prod.launch.py (publishes NO TF):
    filtering -> DBSCAN clustering+tracking -> static/dynamic voting
    -> person fusion -> Kalman motion estimation -> occupancy grid

Output topics (consumers):
  /manriix/obstacle_cloud   PointCloud2, base_footprint frame, RELIABLE
      -> oak_stvl_layer marking source (Nav2 local costmap)
      -> collision_monitor pointcloud_oak
  /oak_d_pro_w/points       raw RGB-aligned cloud, optical frame
      -> oak_stvl_layer clearing source (frustum decay)
  /manriix/objects          ClusterTrackArray: classified+tracked objects
      (static / dynamic / person / uncertain, with velocities)
  /manriix/grid/*           OccupancyGrid layers (viz / debug)

TF requirements (provided elsewhere):
  cuVSLAM: map -> odom -> base_footprint
  RSP/URDF: base_footprint -> ... -> oak_d_pro_w_rgb_camera_optical_frame

USB note: the driver component here owns the OAK device end-to-end.
Nothing else in the bringup may open it (the old oak_pointcloud_bridged
path must stay retired).

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


def generate_launch_description():
    pkg_oak = get_package_share_directory('manriix_oak_obstacles')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
    )

    # Step 1: camera + on-device NN + pointcloud (owns the OAK USB device)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_oak, 'launch', 'oak_camera.launch.py')),
    )

    # Step 2: perception pipeline. Delayed a few seconds so the driver
    # has enumerated the device and topics exist before subscribers spin
    # up (mirrors the old file's staged timing, much shorter since our
    # nodes tolerate missing topics gracefully).
    pipeline_launch = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg=[
                '\n', '=' * 70, '\n',
                '[OAK PROCESSING] Starting manriix dynamic obstacle pipeline\n',
                '  Marking  -> /manriix/obstacle_cloud (base_footprint)\n',
                '  Clearing -> /oak_d_pro_w/points (optical, STVL frustum)\n',
                '  Objects  -> /manriix/objects\n',
                '=' * 70,
            ]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_oak, 'launch',
                                 'manriix_pipeline.launch.py')),
            ),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        LogInfo(msg=[
            '\n', '=' * 70, '\n',
            '[OAK PROCESSING] manriix_oak_obstacles pipeline '
            '(Eppenberger et al. IROS 2020)\n',
            '  Driver: depthai Camera component only — URDF owns camera TF\n',
            '  Camera name/frames: oak_d_pro_w_* (matches URDF)\n',
            '=' * 70,
        ]),
        camera_launch,
        pipeline_launch,
    ])