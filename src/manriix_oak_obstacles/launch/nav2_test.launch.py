"""
nav2_test.launch.py

Brings up a mapless Nav2 stack fed by the manriix OAK-D obstacle
pipeline's /manriix/obstacle_cloud.

Launch order:
  1. ros2 launch manriix_oak_obstacles oak_camera.launch.py
  2. ros2 launch manriix_oak_obstacles manriix_pipeline.launch.py
       -> IMPORTANT: comment out the 'static_tf_map_odom' block in that
          launch first (this launch publishes map -> odom instead).
  3. ros2 launch manriix_oak_obstacles nav2_test.launch.py

Then in RViz (Fixed Frame: map): add Map displays for
/global_costmap/costmap and /local_costmap/costmap, and use the
"Nav2 Goal" / "2D Goal Pose" tool to request a plan. The robot is
static (identity odom), so we're validating costmap ingestion and
path planning around static + velocity-swept dynamic costs -- cmd_vel
output is produced but nothing drives.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('manriix_oak_obstacles')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_share, 'config', 'nav2_test_params.yaml')

    return LaunchDescription([

        # map -> odom (identity; no localization in this test).
        # The pipeline launch publishes odom -> base_footprint.
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
            ],
            output='screen',
            name='static_tf_map_odom_nav2',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': nav2_params,
                'autostart': 'true',
            }.items(),
        ),
    ])