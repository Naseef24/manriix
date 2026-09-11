#!/usr/bin/env python3
"""
Launch the direct ZED SDK ROS 2 node that replaces zed_wrapper.

This launch file starts:
  manriix_perception.nodes.direct_zed_detection_node_modified

The node publishes the raw stereo, CameraInfo, depth, colour and IMU topics
expected by the existing vslam.launch.py and nvblox.launch.py files.

Unlike the old zed_processing.launch.py, this is a normal Python ROS node and
is not loaded into /manriix_container. cuVSLAM and nvblox may remain composable
nodes in that shared container and subscribe to these ROS topics.

File:
  ~/manriix2_ws/src/manriix_navigation/launch/algorithms/
  direct_zed_processing.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mode = LaunchConfiguration("mode")
    enabled_cameras = LaunchConfiguration("enabled_cameras")
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="nav",
                description="'nav' or 'full'.",
            ),
            DeclareLaunchArgument(
                "enabled_cameras",
                default_value="front,left,right",
                description=(
                    "Comma-separated direct SDK camera names: "
                    "front,left,right."
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description=(
                    "Optional YAML file containing a top-level direct_sdk key."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
            ),
            DeclareLaunchArgument(
                "camera_front_serial",
                default_value="41911351",
            ),
            DeclareLaunchArgument(
                "camera_left_serial",
                default_value="40487925",
            ),
            DeclareLaunchArgument(
                "camera_right_serial",
                default_value="46311875",
            ),
            DeclareLaunchArgument(
                "resolution",
                default_value="SVGA",
            ),
            DeclareLaunchArgument(
                "front_resolution",
                default_value="",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="15",
            ),
            DeclareLaunchArgument(
                "front_depth_mode",
                default_value="NEURAL_LIGHT",
                description="Front depth for nvblox.",
            ),
            DeclareLaunchArgument(
                "left_depth_mode",
                default_value="NONE",
                description=(
                    "Use NONE for cuVSLAM-only operation. Enable depth if "
                    "left-camera point clouds or nvblox depth are required."
                ),
            ),
            DeclareLaunchArgument(
                "right_depth_mode",
                default_value="NONE",
                description=(
                    "Use NONE for cuVSLAM-only operation. Enable depth if "
                    "right-camera point clouds or nvblox depth are required."
                ),
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="30.0",
                description=(
                    "ROS publication polling rate. New-frame sequence checks "
                    "prevent duplicate frame publication."
                ),
            ),
            DeclareLaunchArgument(
                "publish_pointcloud",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "pointcloud_downsample",
                default_value="4",
            ),
            DeclareLaunchArgument(
                "publish_compressed_monitoring",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "monitoring_publish_divisor",
                default_value="5",
            ),
            DeclareLaunchArgument(
                "imu_rate_hz",
                default_value="100.0",
            ),
            LogInfo(
                msg=[
                    "\n",
                    "=" * 70,
                    "\n[DIRECT ZED PROCESSING]\n",
                    "  mode:             ",
                    mode,
                    "\n  enabled cameras: ",
                    enabled_cameras,
                    "\n  raw cuVSLAM and nvblox topics enabled\n",
                    "=" * 70,
                ]
            ),
            Node(
                package="manriix_perception",
                executable="direct_zed_detection_node_modified.py",
                name="direct_zed_detection_node_modified",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "mode": mode,
                        "enabled_cameras": enabled_cameras,
                        "config_file": config_file,
                        "camera_front_serial": LaunchConfiguration(
                            "camera_front_serial"
                        ),
                        "camera_left_serial": LaunchConfiguration(
                            "camera_left_serial"
                        ),
                        "camera_right_serial": LaunchConfiguration(
                            "camera_right_serial"
                        ),
                        "resolution": LaunchConfiguration("resolution"),
                        "front_resolution": LaunchConfiguration(
                            "front_resolution"
                        ),
                        "fps": LaunchConfiguration("fps"),
                        "front_depth_mode": LaunchConfiguration(
                            "front_depth_mode"
                        ),
                        "left_depth_mode": LaunchConfiguration(
                            "left_depth_mode"
                        ),
                        "right_depth_mode": LaunchConfiguration(
                            "right_depth_mode"
                        ),
                        "publish_rate": LaunchConfiguration(
                            "publish_rate"
                        ),
                        "publish_pointcloud": LaunchConfiguration(
                            "publish_pointcloud"
                        ),
                        "pointcloud_downsample": LaunchConfiguration(
                            "pointcloud_downsample"
                        ),
                        "publish_compressed_monitoring":
                            LaunchConfiguration(
                                "publish_compressed_monitoring"
                            ),
                        "monitoring_publish_divisor":
                            LaunchConfiguration(
                                "monitoring_publish_divisor"
                            ),
                        "imu_rate_hz": LaunchConfiguration(
                            "imu_rate_hz"
                        ),
                    }
                ],
            ),
        ]
    )