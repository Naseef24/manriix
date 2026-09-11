#!/usr/bin/env python3
"""
================================================================================
oak_xyz_only.launch.py

Loads ONLY the depthai_ros_driver Camera component (no RSP, no URDF).
The main robot bringup is the single source of URDF/TF — see
  ~/manriix2_ws/src/manriix_description/urdf/manriix2.urdf.xacro
where <xacro:depthai_camera camera_name="oak_d_pro_w" .../> declares
the OAK frames.

What this launch does:
  - Spawns the OAK container (oak_d_pro_w_container)
  - Loads depthai_ros_driver::Camera component (pcl.yaml params)
  - Loads depth_image_proc::PointCloudXyzNode into the same container
  - Publishes /oak/points (BEST_EFFORT)

What this launch does NOT do:
  - NOT include camera.launch.py (would spawn a duplicate RSP)
  - NOT include urdf_launch.py (would spawn a duplicate RSP)
  - NOT load image_proc::Rectify (broken OpenCV remap path on this build)
  - NOT load PointCloudXyzrgbNode (needs rectified RGB we don't produce)

File: ~/manriix2_ws/src/manriix_description/launch/oak_xyz_only.launch.py
================================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # NOTE: We do NOT load depthai_ros_driver/config/pcl.yaml here.
    # That YAML is keyed under '/oak:' namespace, but our node runs as
    # '/oak_d_pro_w', so none of the params would apply.
    # Instead, we set all params inline (single source of truth in this file).

    # OAK device + companion XYZ point-cloud node, both inside the same
    # composable container for intra-process zero-copy.
    oak_container = ComposableNodeContainer(
        name='oak_d_pro_w_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 1) The OAK device driver (depthai_ros_driver::Camera).
            #    Name MUST match camera_name in the URDF macro
            #    (oak_d_pro_w) so topic prefixes & frame names align with TF.
            ComposableNode(
                package='depthai_ros_driver',
                plugin='depthai_ros_driver::Camera',
                name='oak_d_pro_w',
                namespace='',
                parameters=[{
                    # ---- Pipeline ----
                    # RGBD = color + stereo depth, no NN
                    'camera.i_pipeline_type': 'RGBD',
                    'camera.i_nn_type':       'none',

                    # ---- Stereo (depth) ----
                    'stereo.i_align_depth':              True,
                    'stereo.i_board_socket_id':          2,
                    'stereo.i_subpixel':                 True,
                    'stereo.i_lr_check':                 True,
                    'stereo.i_right_rect_publish_topic': True,
                    'stereo.i_right_rect_synced':        False,
                    # Run at 10 Hz (was implicitly defaulting low; 10 is plenty
                    # for navigation at 0.4 m/s and cuts CPU significantly).
                    # Bump to 15 or 20 later if more responsiveness needed.
                    'stereo.i_fps':                      10.0,

                    # ---- RGB ----
                    # Match stereo rate so messages can sync properly
                    'rgb.i_fps':                         10.0,
                }],
            ),
            # 2) Depth -> XYZ point cloud (no color, no rectify).
            #    Publishes /oak/points (downstream consumer convention).
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                namespace='',
                remappings=[
                    ('image_rect',  '/oak_d_pro_w/stereo/image_raw'),
                    ('camera_info', '/oak_d_pro_w/stereo/camera_info'),
                    ('points',      '/oak/points'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        oak_container,
    ])