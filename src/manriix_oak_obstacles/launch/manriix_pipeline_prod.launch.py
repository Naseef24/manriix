"""
manriix_pipeline_prod.launch.py — PRODUCTION pipeline (no TF publishers).

TF ownership in production:
  map -> odom -> base_footprint            : cuVSLAM
  base_footprint -> base_link -> oak_...   : URDF via robot_state_publisher
This launch publishes NOTHING to /tf or /tf_static — deliberately.
Use manriix_pipeline.launch.py (dev) only for bench testing without
the full bringup.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('manriix_oak_obstacles')
    params_file = os.path.join(pkg_share, 'config', 'pipeline_params_prod.yaml')

    node_names = [
        'pointcloud_filter_node',
        'cluster_tracking_node',
        'static_dynamic_classifier_node',
        'people_tracker_node',
        'fusion_node',
        'motion_estimation_node',
        'occupancy_grid_node',
    ]

    return LaunchDescription([
        SetEnvironmentVariable('OMP_NUM_THREADS', '3'),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '2'),
        *[
            Node(
                package='manriix_oak_obstacles',
                executable=name,
                name=name,
                output='screen',
                parameters=[params_file],
            )
            for name in node_names
        ],
    ])