#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("node_name", default_value="direct_zed_detection_node_modified"),
        DeclareLaunchArgument("mode", default_value="nav"),
        DeclareLaunchArgument("enabled_cameras", default_value="zedx_front"),
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("camera_front_serial", default_value="41911351"),
        DeclareLaunchArgument("camera_left_serial", default_value="40487925"),
        DeclareLaunchArgument("camera_right_serial", default_value="46311875"),
        DeclareLaunchArgument("resolution", default_value="SVGA"),
        DeclareLaunchArgument("front_resolution", default_value=""),
        DeclareLaunchArgument("fps", default_value="15"),
        DeclareLaunchArgument("front_depth_mode", default_value="NEURAL_LIGHT"),
        DeclareLaunchArgument("left_depth_mode", default_value="NONE"),
        DeclareLaunchArgument("right_depth_mode", default_value="NONE"),
        DeclareLaunchArgument("publish_rate", default_value="15.0"),
        DeclareLaunchArgument("publish_pointcloud", default_value="False"),
        DeclareLaunchArgument("pointcloud_downsample", default_value="4"),
        DeclareLaunchArgument("publish_compressed_monitoring", default_value="False"),
        DeclareLaunchArgument("monitoring_publish_divisor", default_value="5"),
        DeclareLaunchArgument("imu_rate_hz", default_value="100.0"),
    ]

    node = Node(
        package="manriix_perception",
        executable="direct_zed_detection_node_modified.py",
        name=LaunchConfiguration("node_name"),
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "mode": LaunchConfiguration("mode"),
            "enabled_cameras": LaunchConfiguration("enabled_cameras"),
            "config_file": LaunchConfiguration("config_file"),
            "camera_front_serial": LaunchConfiguration("camera_front_serial"),
            "camera_left_serial": LaunchConfiguration("camera_left_serial"),
            "camera_right_serial": LaunchConfiguration("camera_right_serial"),
            "resolution": LaunchConfiguration("resolution"),
            "front_resolution": LaunchConfiguration("front_resolution"),
            "fps": LaunchConfiguration("fps"),
            "front_depth_mode": LaunchConfiguration("front_depth_mode"),
            "left_depth_mode": LaunchConfiguration("left_depth_mode"),
            "right_depth_mode": LaunchConfiguration("right_depth_mode"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            "publish_pointcloud": LaunchConfiguration("publish_pointcloud"),
            "pointcloud_downsample": LaunchConfiguration("pointcloud_downsample"),
            "publish_compressed_monitoring": LaunchConfiguration("publish_compressed_monitoring"),
            "monitoring_publish_divisor": LaunchConfiguration("monitoring_publish_divisor"),
            "imu_rate_hz": LaunchConfiguration("imu_rate_hz"),
        }],
    )

    return LaunchDescription(args + [
        LogInfo(msg=[
            "[DIRECT ZED PROCESS] node=", LaunchConfiguration("node_name"),
            " mode=", LaunchConfiguration("mode"),
            " cameras=", LaunchConfiguration("enabled_cameras"),
        ]),
        node,
    ])