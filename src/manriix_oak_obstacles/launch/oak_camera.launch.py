#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('manriix_oak_obstacles')
    camera_params = os.path.join(pkg_share, 'config', 'oak_camera_params.yaml')

    name = LaunchConfiguration('name')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='oak'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('params_file', default_value=camera_params),
        DeclareLaunchArgument('use_sim_time', default_value='False'),

        ComposableNodeContainer(
            name=[name, '_container'],
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            composable_node_descriptions=[],
        ),

        LoadComposableNodes(
            target_container='oak_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depthai_ros_driver',
                    plugin='depthai_ros_driver::Camera',
                    name='oak',
                    namespace='',
                    parameters=[
                        params_file,
                        {'use_sim_time': use_sim_time},
                    ],
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz',
                    namespace='',
                    remappings=[
                        ('image_rect', '/oak/stereo/image_raw'),
                        ('camera_info', '/oak/stereo/camera_info'),
                        ('points', '/oak/points'),
                    ],
                ),
            ],
        ),
        
        Node(
            package='manriix_oak_obstacles',
            executable='fov_marker_node',
            name='fov_marker_node',
            output='screen',
            parameters=[{
                'base_frame': 'base_footprint',
                # DEV launch (oak_camera.launch.py): camera_name='oak'
                #   -> camera_frame: 'oak_rgb_camera_optical_frame'
                # PROD launch (oak_camera_prod.launch.py): match whatever
                #   camera_name is actually deployed there, e.g.
                #   'oak_d_pro_w_rgb_camera_optical_frame' if that naming
                #   is used, or 'oak_rgb_camera_optical_frame' if the
                #   dev-style naming was kept (as it currently is on Kyro).
                'camera_frame': 'oak_rgb_camera_optical_frame',
                'horizontal_fov_deg': 127.0,
                'range_m': 3.0,   # keep in sync with pointcloud_filter_node's l_d
                'publish_rate_hz': 5.0,
            }],
        ),
    ])