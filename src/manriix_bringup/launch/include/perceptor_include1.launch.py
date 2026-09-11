#!/usr/bin/env python3
"""
================================================================================
Manriix Perceptor Include — Perception orchestrator using direct ZED SDK

Purpose: dispatch perception sub-launches based on configuration.

Stereo camera configurations:
  no_cameras
  front_only_vslam
  front_configuration
  front_left_right_configuration

Sub-launches:
  direct_zed_processing.launch.py — direct ZED SDK ROS driver
  vslam.launch.py                 — cuVSLAM
  nvblox.launch.py                — nvblox
  oak_processing_old.launch.py    — OAK driver + QoS bridge
================================================================================
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     IncludeLaunchDescription,
#     LogInfo,
#     OpaqueFunction,
# )
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnShutdown

def resolve_perception_configuration(context, *args, **kwargs):
    """Resolve the selected perception configuration."""
    from launch.actions import SetLaunchConfiguration

    stereo_config = LaunchConfiguration(
        "stereo_camera_configuration"
    ).perform(context)
    enable_nvblox = (
        LaunchConfiguration("enable_nvblox")
        .perform(context)
        .lower()
        == "true"
    )
    enable_oak = (
        LaunchConfiguration("enable_oak_voxel_costmap")
        .perform(context)
        .lower()
        == "true"
    )

    if stereo_config == "no_cameras":
        enabled_cameras = []
        vslam_cameras = []
        nvblox_cameras = []
    elif stereo_config == "front_only_vslam":
        enabled_cameras = ["zedx_front"]
        vslam_cameras = ["zedx_front"]
        nvblox_cameras = []
    elif stereo_config == "front_configuration":
        enabled_cameras = ["zedx_front"]
        vslam_cameras = ["zedx_front"]
        nvblox_cameras = ["zedx_front"] if enable_nvblox else []
    elif stereo_config == "front_left_right_configuration":
        enabled_cameras = ["zedx_front", "zedx_left", "zedx_right"]
        vslam_cameras = ["zedx_front", "zedx_left", "zedx_right"]
        nvblox_cameras = ["zedx_front"] if enable_nvblox else []
    else:
        raise RuntimeError(
            f"Unknown stereo_camera_configuration: {stereo_config}"
        )

    enabled_cameras_str = ",".join(enabled_cameras)
    vslam_cameras_str = ",".join(vslam_cameras)
    nvblox_cameras_str = ",".join(nvblox_cameras)

    return [
        LogInfo(
            msg=[
                "\n",
                "=" * 70,
                "\n[PERCEPTOR INCLUDE] Configuration: ",
                stereo_config,
                "\n  Enabled cameras:      ",
                enabled_cameras_str or "(none)",
                "\n  cuVSLAM cameras:      ",
                vslam_cameras_str or "(none)",
                "\n  nvblox cameras:       ",
                nvblox_cameras_str or "(none)",
                "\n  OAK-D enabled (STVL): ",
                str(enable_oak),
                "\n",
                "=" * 70,
            ]
        ),
        SetLaunchConfiguration(
            "enabled_cameras",
            enabled_cameras_str,
        ),
        SetLaunchConfiguration(
            "vslam_cameras",
            vslam_cameras_str,
        ),
        SetLaunchConfiguration(
            "nvblox_cameras",
            nvblox_cameras_str,
        ),
        SetLaunchConfiguration(
            "enable_cameras_flag",
            "True" if enabled_cameras else "False",
        ),
        SetLaunchConfiguration(
            "enable_vslam_flag",
            "True" if vslam_cameras else "False",
        ),
        SetLaunchConfiguration(
            "enable_nvblox_flag",
            "True" if nvblox_cameras else "False",
        ),
        SetLaunchConfiguration(
            "enable_oak_flag",
            "True" if enable_oak else "False",
        ),
    ]

def stop_remote_oak_service(context, *args, **kwargs):
    """Synchronously stop the OAK stack running on Jetson B."""

    print(
        "[OAK REMOTE] Stopping manriix-oak.service on Kyro..."
    )

    try:
        result = subprocess.run(
            [
                "ssh",
                "manriix-kyro",
                "sudo",
                "/usr/bin/systemctl",
                "stop",
                "manriix-oak.service",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=20,
            check=False,
        )

        if result.returncode == 0:
            print(
                "[OAK REMOTE] manriix-oak.service stopped successfully."
            )
        else:
            print(
                "[OAK REMOTE] WARNING: remote stop returned "
                f"code {result.returncode}"
            )

            if result.stderr:
                print(
                    "[OAK REMOTE] stderr: "
                    f"{result.stderr.strip()}"
                )

    except subprocess.TimeoutExpired:
        print(
            "[OAK REMOTE] ERROR: timeout while stopping "
            "manriix-oak.service."
        )

    except Exception as exc:
        print(
            "[OAK REMOTE] ERROR while stopping remote OAK service: "
            f"{exc}"
        )

    return []

def generate_launch_description():
    pkg_navigation = get_package_share_directory("manriix_navigation")
    pkg_oak_obstacles = get_package_share_directory("manriix_oak_obstacles")

    stereo_camera_configuration_arg = DeclareLaunchArgument(
        "stereo_camera_configuration",
        default_value="front_left_right_configuration",
        choices=[
            "no_cameras",
            "front_only_vslam",
            "front_configuration",
            "front_left_right_configuration",
        ],
        description="Which cameras and algorithms to run.",
    )

    enable_nvblox_arg = DeclareLaunchArgument(
        "enable_nvblox",
        default_value="True",
        description="Run nvblox when the selected configuration supports it.",
    )

    enable_oak_voxel_costmap_arg = DeclareLaunchArgument(
        "enable_oak_voxel_costmap",
        default_value="True",
        description="Run the OAK-D STVL input pipeline.",
    )

    enable_wheel_odometry_arg = DeclareLaunchArgument(
        "enable_wheel_odometry",
        default_value="False",
        description=(
            "When True, wheel odometry owns odom->base_footprint."
        ),
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
    )

    direct_zed_processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_navigation,
                "launch",
                "algorithms",
                "direct_zed_processing.launch.py",
            )
        ),
        launch_arguments={
            "enabled_cameras": LaunchConfiguration("enabled_cameras"),
            "mode": "full",
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(
            LaunchConfiguration("enable_cameras_flag")
        ),
    )

    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_navigation,
                "launch",
                "algorithms",
                "vslam.launch.py",
            )
        ),
        launch_arguments={
            "vslam_cameras": LaunchConfiguration("vslam_cameras"),
            "enable_wheel_odometry": LaunchConfiguration(
                "enable_wheel_odometry"
            ),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(
            LaunchConfiguration("enable_vslam_flag")
        ),
    )

    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_navigation,
                "launch",
                "algorithms",
                "nvblox.launch.py",
            )
        ),
        launch_arguments={
            "nvblox_cameras": LaunchConfiguration("nvblox_cameras"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(
            LaunchConfiguration("enable_nvblox_flag")
        ),
    )

    # oak_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             pkg_navigation,
    #             "launch",
    #             "algorithms",
    #             "oak_processing_old.launch.py",
    #         )
    #     ),
    #     condition=IfCondition(
    #         LaunchConfiguration("enable_oak_flag")
    #     ),
    # )

    # oak_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             pkg_oak_obstacles, 'launch',
    #             'oak_camera.launch.py'  # or a new combined launch
    #         )
    #     ),
    #     condition=IfCondition(LaunchConfiguration('enable_oak_flag')),
    # )

    # oak_pipeline_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             pkg_oak_obstacles, 'launch',
    #             'manriix_pipeline.launch.py'
    #         )
    #     ),
    #     condition=IfCondition(LaunchConfiguration('enable_oak_flag')),
    # )

    # ---------------------------------------------------------
    # Remote OAK-D stack
    #
    # The OAK camera + obstacle pipeline physically run on
    # Jetson B (kyro). Jetson A (hype) only controls the
    # systemd service over SSH.
    # ---------------------------------------------------------

    oak_remote_start = ExecuteProcess(
        cmd=[
            "ssh",
            "manriix-kyro",
            "sudo",
            "/usr/bin/systemctl",
            "start",
            "manriix-oak.service",
        ],
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("enable_oak_flag")
        ),
    )

    # oak_remote_stop = ExecuteProcess(
    #     cmd=[
    #         "ssh",
    #         "manriix-kyro",
    #         "sudo",
    #         "/usr/bin/systemctl",
    #         "stop",
    #         "manriix-oak.service",
    #     ],
    #     output="screen",
    # )

    oak_shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(
                    msg=(
                        "[OAK REMOTE] Main launch shutting down. "
                        "Stopping manriix-oak.service on Kyro..."
                    )
                ),
                OpaqueFunction(
                    function=stop_remote_oak_service
                ),
            ]
        ),
        condition=IfCondition(
            LaunchConfiguration("enable_oak_flag")
        ),
    )

    return LaunchDescription(
        [
            stereo_camera_configuration_arg,
            enable_nvblox_arg,
            enable_oak_voxel_costmap_arg,
            enable_wheel_odometry_arg,
            use_sim_time_arg,
            OpaqueFunction(
                function=resolve_perception_configuration
            ),
            direct_zed_processing,
            vslam_launch,
            nvblox_launch,
            # oak_launch,
            # oak_pipeline_launch,
            oak_remote_start,
            oak_shutdown_handler,
        ]
    )