"""
Step 1 launch: static TF chain (map -> base_footprint -> oak-d-base-frame)
plus the point cloud filtering node (Sec III-B of the paper).

Assumes the depthai-ros driver (which publishes /oak/points and the
oak-d-base-frame -> oak -> ... -> oak_*_optical_frame chain) is already
running via its own launch file -- start that separately first.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('manriix_oak_obstacles')
    params_file = os.path.join(pkg_share, 'config', 'pipeline_params.yaml')

    return LaunchDescription([

        # map -> base_footprint (identity; standalone testing global frame)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'base_footprint',
            ],
            output='screen',
            name='static_tf_map_base',
        ),

        # base_footprint -> oak-d-base-frame
        # (depthai-ros driver publishes oak-d-base-frame -> oak -> ... ->
        #  oak_right_camera_optical_frame itself; do NOT duplicate that.)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', '0.400', '--y', '-0.0375', '--z', '0.270',
                '--roll', '0.0', '--pitch', '-0.08', '--yaw', '0.0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'oak-d-base-frame',
            ],
            output='screen',
            name='static_tf_base_oak_base',
        ),

        Node(
            package='manriix_oak_obstacles',
            executable='pointcloud_filter_node',
            name='pointcloud_filter_node',
            output='screen',
            parameters=[params_file],
        ),
    ])