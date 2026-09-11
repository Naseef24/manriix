#!/usr/bin/env python3
"""
================================================================================
Manriix Navigation Include — The Workhorse (Carter-style)

Modeled on:
  nova_carter/nova_carter_bringup/launch/include/navigation_include.launch.py

Carter uses isaac_ros_launch_utils.set_parameter() (a service-call wrapper
not available outside Isaac ROS) to inject the plugin list. We can't use it.

We also can't use nav2_common.launch.RewrittenYaml because it serializes
list-valued param_rewrites as a single concatenated string — Nav2 then
rejects `plugins: "obstacle_layerinflation_layer"` because it expected
a string_array.

Workaround: read the source YAML in plain Python, modify the dict in
memory, write it to a temp file, point Nav2 at the temp file.

Workflow:
  1. Resolve enable_* flags from launch context.
  2. Build local/global plugin lists based on flags.
  3. Decide odometry topic (cuVSLAM vs wheel odom).
  4. Read source nav2 params yaml.
  5. Write rewritten copy to /tmp/manriix_nav2_*.yaml.
  6. Expose the temp file path via SetLaunchConfiguration so
     nav2_include.launch.py picks it up.

OAK obstacle integration (replaces old RANSAC + DFKI stack):
  - When enable_oak_obstacles is True, includes oak_obstacle.launch.py
    from the manriix_oak_obstacles package.
  - Appends 'oak_stvl_layer' to local_costmap plugins.
  - The OAK driver and oak_ground_remover_node are NO LONGER LAUNCHED
    from hardware_abstraction_layer_include / perceptor_include because
    the new node owns the OAK USB device end-to-end.

Remote monitoring note:
  Remote camera/Foxglove traffic is intentionally kept out of this workhorse.
  This file continues to own navigation composition only. Camera monitoring is
  handled by the direct ZED driver at low rate and visualization_include via a
  strict whitelist.

File: ~/manriix2_ws/src/manriix_bringup/launch/include/navigation_include.launch.py
================================================================================
"""

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# --------------------------------------------------------------------------
# OpaqueFunction: read source YAML, override plugin lists + odom_topic,
# write to /tmp, expose path to sub-launches.
# --------------------------------------------------------------------------
def build_rewritten_yaml(context, *args, **kwargs):
    enable_2d_lidar_costmap = LaunchConfiguration(
        'enable_2d_lidar_costmap').perform(context).lower() == 'true'
    enable_oak_obstacles = LaunchConfiguration(
        'enable_oak_obstacles').perform(context).lower() == 'true'
    enable_nvblox_costmap = LaunchConfiguration(
        'enable_nvblox_costmap').perform(context).lower() == 'true'
    enable_wheel_odometry = LaunchConfiguration(
        'enable_wheel_odometry').perform(context).lower() == 'true'
    map_yaml_path = LaunchConfiguration('map_yaml_path').perform(context)
    source_path = LaunchConfiguration(
        'navigation_parameters_path').perform(context)

    # -------- Build LOCAL and GLOBAL plugin lists --------
    local_plugins = []
    global_plugins = []

    enable_global_nav = map_yaml_path not in ('None', '', 'none')
    if enable_global_nav:
        global_plugins.append('static_map_layer')
        print(f'[NAV INCLUDE] Global map provided: {map_yaml_path}')

    if enable_nvblox_costmap:
        local_plugins.append('nvblox_layer')
        global_plugins.append('nvblox_layer')
        print('[NAV INCLUDE] Enabling nvblox layer.')

    if enable_2d_lidar_costmap:
        local_plugins.append('obstacle_layer')
        global_plugins.append('obstacle_layer')
        print('[NAV INCLUDE] Enabling 2D LiDAR obstacle layer.')

    if enable_oak_obstacles:
        # OAK obstacle pipeline:
        #   depthai_ros_driver -> /oak/points
        #   oak_ground_remover_node (PCL RANSAC + clustering)
        #     -> /oak/points_obstacles  (marking source)
        #     -> /oak/points_ground     (clearing source via raytrace)
        #   nav2_costmap_2d::ObstacleLayer with both sources consumes them
        #
        # NOTE: We switched OFF DFKI ground_consistency_layer because its
        # probabilistic decay couldn't clear cells fast enough at 30 Hz
        # pipeline rate. Standard ObstacleLayer with mark+clear sources is
        # the same pattern as the LIDAR scan layer and clears instantly.
        # local_plugins.append('oak_stvl_layer')
        # print('[NAV INCLUDE] Enabling OAK ObstacleLayer (mark+clear) — local only.')
        # local_plugins.append('oak_stvl_layer')
        print('[NAV INCLUDE] OAK obstacles enabled via obstacle_layer.oak_objects – no STVL layer.')

    # Inflation last
    local_plugins.append('inflation_layer')
    global_plugins.append('inflation_layer')

    # -------- Odometry topic decision --------
    if enable_wheel_odometry:
        odom_topic = '/odom'
        print(f'[NAV INCLUDE] Odometry source: wheel odom ({odom_topic})')
    else:
        odom_topic = '/visual_slam/tracking/odometry'
        print(f'[NAV INCLUDE] Odometry source: cuVSLAM ({odom_topic})')

    print(f'[NAV INCLUDE] Local costmap plugins:  {local_plugins}')
    print(f'[NAV INCLUDE] Global costmap plugins: {global_plugins}')

    # -------- Read source YAML --------
    with open(source_path, 'r') as f:
        params = yaml.safe_load(f)

    # -------- Override local costmap plugins --------
    try:
        params['local_costmap']['local_costmap']['ros__parameters']['plugins'] = local_plugins
    except KeyError as e:
        print(f'[NAV INCLUDE] WARNING: could not set local_costmap plugins: {e}')

    # -------- Override global costmap plugins --------
    try:
        params['global_costmap']['global_costmap']['ros__parameters']['plugins'] = global_plugins
    except KeyError as e:
        print(f'[NAV INCLUDE] WARNING: could not set global_costmap plugins: {e}')

    # -------- Override odom_topic for the three nodes that use it --------
    for node_name in ('bt_navigator', 'controller_server', 'velocity_smoother'):
        try:
            params[node_name]['ros__parameters']['odom_topic'] = odom_topic
        except KeyError:
            pass

    # -------- Write rewritten YAML to /tmp --------
    fd, rewritten_path = tempfile.mkstemp(
        prefix='manriix_nav2_', suffix='.yaml', text=True)
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)

    return [
        LogInfo(msg=[
            '[NAV INCLUDE] Rewritten nav2 params: ', rewritten_path,
        ]),
        SetLaunchConfiguration(
            'rewritten_navigation_parameters_path', rewritten_path
        ),
    ]


def generate_launch_description():
    pkg_bringup = get_package_share_directory('manriix_bringup')
    pkg_navigation = get_package_share_directory('manriix_navigation')
    pkg_oak_obstacles = get_package_share_directory('manriix_oak_obstacles')

    # ---- Arguments ----
    args = [
        DeclareLaunchArgument('hardware_mode', default_value='real'),
        DeclareLaunchArgument('stereo_camera_configuration',
                              default_value='front_left_right_configuration'),
        DeclareLaunchArgument('enable_navigation', default_value='True'),
        DeclareLaunchArgument('enable_2d_lidar_costmap', default_value='True'),
        DeclareLaunchArgument('enable_oak_obstacles', default_value='True'),
        DeclareLaunchArgument('enable_nvblox_costmap', default_value='True'),
        DeclareLaunchArgument('enable_wheel_odometry', default_value='False'),
        DeclareLaunchArgument('map_yaml_path', default_value='None'),
        DeclareLaunchArgument(
            'navigation_parameters_path',
            default_value=os.path.join(
                pkg_navigation, 'config', 'nav2', 'nav2_params_carter.yaml'),
            description='Source nav2 params YAML (will be rewritten in-memory '
                        'with dynamic plugin list + odom topic).',
        ),
        DeclareLaunchArgument('use_sim_time',default_value='False',description='Use simulation clock for all navigation and perception nodes.',
        ),
    ]

    hardware_mode = LaunchConfiguration('hardware_mode')
    stereo_camera_configuration = LaunchConfiguration('stereo_camera_configuration')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_oak_obstacles = LaunchConfiguration('enable_oak_obstacles')
    enable_nvblox_costmap = LaunchConfiguration('enable_nvblox_costmap')
    enable_wheel_odometry = LaunchConfiguration('enable_wheel_odometry')
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- Sub-launches ----
    # IMPORTANT: hardware_abstraction_layer_include + perceptor_include
    # previously took 'enable_oak_voxel_costmap' to decide whether to
    # launch the OAK driver and oak_ground_remover_node. We pass False
    # to those flags here because the new oak_obstacle_node owns the
    # OAK USB device entirely (depthai opens the device internally).
    # Letting the old paths also try to open the OAK would cause a USB
    # device conflict ("device busy"). See note at bottom of this file.
    hardware_layer_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'include',
                         'hardware_abstraction_layer_include.launch.py')
        ),
        launch_arguments={
            'hardware_mode': hardware_mode,
            # Forward the new flag to old plumbing — they share semantics now:
            # enable_oak_obstacles True == launch OAK driver + ground_remover (PCL RANSAC)
            'enable_oak_voxel_costmap': enable_oak_obstacles,
        }.items(),
    )

    perceptor_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'include',
                         'perceptor_include.launch.py')
        ),
        launch_arguments={
            'stereo_camera_configuration': stereo_camera_configuration,
            'enable_nvblox': enable_nvblox_costmap,
            # Same: route enable_oak_obstacles -> existing downstream flag
            'enable_oak_voxel_costmap': enable_oak_obstacles,
            'enable_wheel_odometry': enable_wheel_odometry,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # OAK edge-based node is DEPRECATED.
    # OAK obstacle detection now happens via:
    #   perceptor_include -> oak_processing.launch.py
    #     -> oak_pointcloud_bridged.launch.py (depthai_ros_driver C++)
    #     -> oak_ground_remover_node (PCL RANSAC + Euclidean clustering)
    # Outputs /oak/points_obstacles consumed by oak_ground_consistency_layer
    # in nav2_params_carter.yaml local_costmap.

    # Nav2 reads the REWRITTEN yaml path, not the source.
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'include',
                         'nav2_include.launch.py')
        ),
        launch_arguments={
            'navigation_parameters_path': LaunchConfiguration(
                'rewritten_navigation_parameters_path'),
            'map_yaml_path': map_yaml_path,
        }.items(),
        condition=IfCondition(enable_navigation),
    )

    return LaunchDescription([
        *args,

        # Step 1: rewrite the yaml with dynamic content
        OpaqueFunction(function=build_rewritten_yaml),

        # Step 2: dispatch sub-launches
        hardware_layer_include,
        perceptor_include,
        nav2_include,
    ])