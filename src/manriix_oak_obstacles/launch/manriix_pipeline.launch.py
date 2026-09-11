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
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('manriix_oak_obstacles')
    params_file = os.path.join(pkg_share, 'config', 'pipeline_params.yaml')

    return LaunchDescription([

        # Cap OpenMP threads (Open3D, numpy/BLAS, scipy) for all nodes in
        # this launch. Without this, Open3D's parallel kernels in the
        # filter node will consume every core on the Jetson.
        SetEnvironmentVariable('OMP_NUM_THREADS', '3'),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '2'),

        # odom -> base_footprint (identity; robot is stationary in tests).
        # REP-105: Nav2 owns/expects map -> odom (provided by the nav2
        # test launch, or later by real localization); publishing
        # map -> base_footprint directly here would give base_footprint
        # two parents and break TF.
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
        #         '--x', '0.0', '--y', '0.0', '--z', '0.0',
        #         '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
        #         '--frame-id', 'odom',
        #         '--child-frame-id', 'base_footprint',
        #     ],
        #     output='screen',
        #     name='static_tf_odom_base',
        # ),

        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
        #         '--x', '0.0', '--y', '0.0', '--z', '0.5',
        #         '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
        #         '--frame-id', 'base_footprint',
        #         '--child-frame-id', 'base_link',
        #     ],
        #     output='screen',
        #     name='static_tf_odom_base',
        # ),

        # # map -> odom (identity) so the pipeline also runs standalone
        # # WITHOUT Nav2. NOTE: comment this block out when launching the
        # # nav2 test launch, which publishes map -> odom itself.
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
        #         '--x', '0.0', '--y', '0.0', '--z', '0.0',
        #         '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
        #         '--frame-id', 'map',
        #         '--child-frame-id', 'odom',
        #     ],
        #     output='screen',
        #     name='static_tf_map_odom',
        # ),

        # base_footprint -> oak-d-base-frame
        # (depthai-ros driver publishes oak-d-base-frame -> oak -> ... ->
        #  oak_right_camera_optical_frame itself; do NOT duplicate that.)
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
        #         '--x', '0.400', '--y', '-0.0375', '--z', '0.0',
        #         # '--roll', '0.0', '--pitch', '-0.08', '--yaw', '0.0',
        #         '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
        #         '--frame-id', 'base_link',
        #         '--child-frame-id', 'oak-d-base-frame',
        #     ],
        #     output='screen',
        #     name='static_tf_base_oak_base',
        # ),

        Node(
            package='manriix_oak_obstacles',
            executable='pointcloud_filter_node',
            name='pointcloud_filter_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='manriix_oak_obstacles',
            executable='cluster_tracking_node',
            name='cluster_tracking_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='manriix_oak_obstacles',
            executable='static_dynamic_classifier_node',
            name='static_dynamic_classifier_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='manriix_oak_obstacles',
            executable='people_tracker_node',
            name='people_tracker_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='manriix_oak_obstacles',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='manriix_oak_obstacles',
            executable='motion_estimation_node',
            name='motion_estimation_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='manriix_oak_obstacles',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
