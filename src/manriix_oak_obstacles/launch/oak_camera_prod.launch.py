"""
oak_camera_prod.launch.py — PRODUCTION camera bringup.

Differences from oak_camera.launch.py (the standalone/dev launch):

1. Loads ONLY the depthai_ros_driver::Camera composable — no
   oak_state_publisher, no rectify_color_node. The robot URDF
   (manriix_description/urdf/manriix2.urdf.xacro) already declares the
   camera via depthai_macro with camera_name='oak_d_pro_w'; the main
   bringup's robot_state_publisher owns ALL camera frames. Loading the
   driver's own RSP would create a second publisher of the same frames.
   (Skipping rectify also avoids the upstream image_proc::Rectify OpenCV
   remap crash the old oak_xyz_only.launch.py existed to dodge.)

2. Camera name = 'oak_d_pro_w' so the frame_ids stamped on the driver's
   data (oak_d_pro_w_rgb_camera_optical_frame, ...) match the URDF
   frames exactly. Topics are namespaced accordingly: /oak_d_pro_w/...

3. Adds a plain PointCloudXyzNode consuming the RGB-aligned depth
   (/oak_d_pro_w/stereo/*) -> /oak_d_pro_w/points.

Params: config/oak_camera_prod_params.yaml (spatial MobileNet NN,
depth aligned to RGB socket, 640x400 stereo).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('manriix_oak_obstacles')
    camera_params = os.path.join(
        pkg_share, 'config', 'oak_camera_prod_params.yaml')

    return LaunchDescription([
        ComposableNodeContainer(
            name='oak_d_pro_w_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            composable_node_descriptions=[
                ComposableNode(
                    package='depthai_ros_driver',
                    plugin='depthai_ros_driver::Camera',
                    name='oak_d_pro_w',
                    parameters=[camera_params],
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='oak_point_cloud_xyz',
                    remappings=[
                        ('image_rect', '/oak_d_pro_w/stereo/image_raw'),
                        ('camera_info', '/oak_d_pro_w/stereo/camera_info'),
                        ('points', '/oak_d_pro_w/points'),
                    ],
                ),
            ],
        ),
    ])