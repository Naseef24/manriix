#!/usr/bin/env python3

"""
MANRIIX Perceptor Include
=========================

Purpose
-------
Bring up the perception stack used by MANRIIX navigation.

ZED architecture
----------------
The three ZED X cameras are now owned by ONE process:

    multi_zed_capture_node_v2.py

Camera roles:

FRONT ZED X
    Serial: 41911351
    Resolution: SVGA
    FPS: 15
    Depth: NEURAL_LIGHT

    Publishes:
        /zedx_front/zed_node/left/gray/rect/image
        /zedx_front/zed_node/right/gray/rect/image

        /zedx_front/zed_node/left/gray/rect/camera_info
        /zedx_front/zed_node/right/gray/rect/camera_info

        /zedx_front/zed_node/left/color/rect/image
        /zedx_front/zed_node/left/color/rect/camera_info

        /zedx_front/zed_node/depth/depth_registered
        /zedx_front/zed_node/depth/camera_info

        /zedx_front/zed_node/imu/data

        /monitor/zedx_front/image_raw

    Used by:
        cuVSLAM
        nvblox
        shared 2D detector later


LEFT ZED X
    Serial: 40487925
    Resolution: SVGA
    FPS: 15
    Depth: NONE

    Publishes:
        /monitor/zedx_left/image_raw

    Used by:
        shared 2D detector
        lightweight remote monitoring


RIGHT ZED X
    Serial: 46311875
    Resolution: SVGA
    FPS: 15
    Depth: NONE

    Publishes:
        /monitor/zedx_right/image_raw

    Used by:
        shared 2D detector
        lightweight remote monitoring


IMPORTANT
---------
The following OLD architecture is intentionally NOT launched here:

    direct_zed_detection_node_modified.py
    zed_side_capture_supervisor.py
    zed_side_worker_supervisor.py
    delayed LEFT ZED processes
    delayed RIGHT ZED processes
    independent front/left/right ZED SDK processes

All three cameras must remain owned by multi_zed_capture_node_v2.py.


Startup prerequisite
--------------------
Before starting navigation.launch.py, the ZED daemon should be restarted
from a clean state:

    sudo systemctl restart zed_x_daemon.service
    sleep 5

The launch also forces:

    DISPLAY=:0
    XAUTHORITY=/run/user/1000/gdm/Xauthority

because MANRIIX is normally started remotely over SSH.


Perception architecture
-----------------------

    multi_zed_capture_node
              |
              +--> FRONT stereo + IMU
              |         |
              |         +--> cuVSLAM
              |
              +--> FRONT color + depth
              |         |
              |         +--> nvblox
              |
              +--> LEFT monitor image
              |
              +--> RIGHT monitor image


The shared YOLO detector is intentionally NOT launched yet.
It will be added after this camera + cuVSLAM + nvblox architecture
is validated.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)

from launch.conditions import IfCondition

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)

from launch_ros.actions import Node


def generate_launch_description():

    # ================================================================
    # Package paths
    # ================================================================

    pkg_navigation = get_package_share_directory(
        "manriix_navigation"
    )

    # ================================================================
    # Launch arguments
    # ================================================================

    stereo_camera_configuration_arg = DeclareLaunchArgument(
        "stereo_camera_configuration",
        default_value="front_left_right_configuration",
        description=(
            "Stereo camera configuration. "
            "Use front_left_right_configuration for the "
            "three-ZED production configuration."
        ),
    )

    enable_nvblox_arg = DeclareLaunchArgument(
        "enable_nvblox",
        default_value="True",
        description="Enable nvblox perception.",
    )

    enable_oak_obstacles_arg = DeclareLaunchArgument(
        "enable_oak_obstacles",
        default_value="True",
        description=(
            "Enable OAK-D obstacle perception."
        ),
    )

    enable_oak_voxel_costmap_arg = DeclareLaunchArgument(
        "enable_oak_voxel_costmap",
        default_value="True",
        description=(
            "Enable the OAK-D voxel/STVL obstacle pipeline."
        ),
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
    )

    # ------------------------------------------------
    # ZED manager parameters
    # ------------------------------------------------

    zed_fps_arg = DeclareLaunchArgument(
        "zed_fps",
        default_value="15",
    )

    zed_resolution_arg = DeclareLaunchArgument(
        "zed_resolution",
        default_value="SVGA",
    )

    front_zed_depth_mode_arg = DeclareLaunchArgument(
        "front_zed_depth_mode",
        default_value="NEURAL_LIGHT",
        description=(
            "Depth mode used ONLY by the front ZED X. "
            "LEFT and RIGHT always run with depth disabled."
        ),
    )

    zed_monitor_rate_arg = DeclareLaunchArgument(
        "zed_monitor_rate_hz",
        default_value="5.0",
    )

    zed_monitor_width_arg = DeclareLaunchArgument(
        "zed_monitor_width",
        default_value="640",
    )

    zed_imu_rate_arg = DeclareLaunchArgument(
        "zed_imu_rate_hz",
        default_value="100.0",
    )


    # ------------------------------------------------
    # ZED benchmark configuration (T1/T2/T3/T4)
    # ------------------------------------------------

    enable_left_stereo_arg = DeclareLaunchArgument(
        "enable_left_stereo",
        default_value="False",
    )

    enable_right_stereo_arg = DeclareLaunchArgument(
        "enable_right_stereo",
        default_value="False",
    )

    enable_left_depth_arg = DeclareLaunchArgument(
        "enable_left_depth",
        default_value="False",
    )

    enable_right_depth_arg = DeclareLaunchArgument(
        "enable_right_depth",
        default_value="False",
    )

    vslam_cameras_arg = DeclareLaunchArgument(
        "vslam_cameras",
        default_value="zedx_front",
        description=(
            "Comma separated cameras for cuVSLAM. "
            "Examples: "
            "zedx_front "
            "or "
            "zedx_front,zedx_left,zedx_right"
        ),
    )
    # ================================================================
    # Launch configurations
    # ================================================================

    stereo_camera_configuration = LaunchConfiguration(
        "stereo_camera_configuration"
    )

    enable_nvblox = LaunchConfiguration(
        "enable_nvblox"
    )

    enable_oak_obstacles = LaunchConfiguration(
        "enable_oak_obstacles"
    )

    enable_oak_voxel_costmap = LaunchConfiguration(
        "enable_oak_voxel_costmap"
    )

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )

    zed_fps = LaunchConfiguration(
        "zed_fps"
    )

    zed_resolution = LaunchConfiguration(
        "zed_resolution"
    )

    front_zed_depth_mode = LaunchConfiguration(
        "front_zed_depth_mode"
    )

    zed_monitor_rate_hz = LaunchConfiguration(
        "zed_monitor_rate_hz"
    )

    zed_monitor_width = LaunchConfiguration(
        "zed_monitor_width"
    )

    zed_imu_rate_hz = LaunchConfiguration(
        "zed_imu_rate_hz"
    )


    enable_left_stereo = LaunchConfiguration(
        "enable_left_stereo"
    )

    enable_right_stereo = LaunchConfiguration(
        "enable_right_stereo"
    )

    enable_left_depth = LaunchConfiguration(
        "enable_left_depth"
    )

    enable_right_depth = LaunchConfiguration(
        "enable_right_depth"
    )

    vslam_cameras = LaunchConfiguration(
        "vslam_cameras"
    )
    # ================================================================
    # Environment
    # ================================================================
    #
    # MANRIIX is normally launched over SSH.
    #
    # Do not allow ZED/EGL to inherit:
    #
    #     DISPLAY=localhost:10.0
    #
    # from SSH X forwarding.
    #
    # Force the Jetson's local graphical session.
    # ================================================================

    set_display = SetEnvironmentVariable(
        name="DISPLAY",
        value=":0",
    )

    set_xauthority = SetEnvironmentVariable(
        name="XAUTHORITY",
        value="/run/user/1000/gdm/Xauthority",
    )

    # ================================================================
    # Three-ZED camera manager
    # ================================================================
    #
    # ONE PROCESS owns:
    #
    #     FRONT
    #     LEFT
    #     RIGHT
    #
    # This is critical.
    #
    # Do NOT add independent ZED SDK camera workers back here.
    # ================================================================

    multi_zed_capture = Node(
        package="manriix_perception",
        executable="multi_zed_capture_node_v2.py",
        name="multi_zed_capture_node",

        output="screen",
        emulate_tty=True,

        parameters=[
            {
                "use_sim_time": use_sim_time,

                # -----------------------------------------
                # All cameras
                # -----------------------------------------
                "fps": zed_fps,
                "resolution": zed_resolution,

                # -----------------------------------------
                # FRONT only
                # -----------------------------------------
                "front_depth_mode": front_zed_depth_mode,
                "imu_publish_rate_hz": zed_imu_rate_hz,

                # -----------------------------------------
                # Monitoring
                # -----------------------------------------
                "monitor_rate_hz": zed_monitor_rate_hz,
                "monitor_width": zed_monitor_width,

                # -----------------------------------------
                # Benchmark configuration
                # T1:
                #   all False
                #
                # T2:
                #   left/right stereo True
                #
                # T3/T4:
                #   depth switches
                # -----------------------------------------
                "enable_left_stereo": enable_left_stereo,
                "enable_right_stereo": enable_right_stereo,
                "enable_left_depth": enable_left_depth,
                "enable_right_depth": enable_right_depth,
            }
        ],

        condition=IfCondition(
            PythonExpression([
                "'",
                stereo_camera_configuration,
                "' != 'no_cameras'",
            ])
        ),
    )

    shared_three_camera_detector = Node(
        package="manriix_perception",
        executable="shared_three_camera_yolo_detector.py",
        name="shared_three_camera_yolo_detector",
        output="screen",
        emulate_tty=True,

        parameters=[{
            "model_path": "/home/hype/models/yolo11/yolo11n.pt",

            "confidence": 0.25,

            "imgsz": 640,

            # Total inference budget across all three cameras.
            # Approximately 3 FPS detection per camera initially.
            "max_total_inference_hz": 9.0,

            "device": "0",
        }],

        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration(
                    "stereo_camera_configuration"
                ),
                "' == 'front_left_right_configuration'",
            ])
        ),
    )
    # ================================================================
    # cuVSLAM
    # ================================================================
    #
    # FRONT CAMERA ONLY
    #
    # Inputs expected from multi_zed_capture_node:
    #
    # /zedx_front/zed_node/left/gray/rect/image
    # /zedx_front/zed_node/right/gray/rect/image
    #
    # /zedx_front/zed_node/left/gray/rect/camera_info
    # /zedx_front/zed_node/right/gray/rect/camera_info
    #
    # /zedx_front/zed_node/imu/data
    #
    # cuVSLAM remains responsible for:
    #
    #     map -> odom
    #     odom -> base_footprint
    #
    # ================================================================

    visual_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_navigation,
                "launch",
                "algorithms",
                "vslam.launch.py",
            )
        ),

        launch_arguments={
            "vslam_cameras": LaunchConfiguration(
                "vslam_cameras"
            ),

            "enable_wheel_odometry": "False",

            "use_sim_time": use_sim_time,

        }.items(),

        condition=IfCondition(
            PythonExpression([
                "'",
                stereo_camera_configuration,
                "' != 'no_cameras'",
            ])
        ),
    )

    # ================================================================
    # nvblox
    # ================================================================
    #
    # FRONT CAMERA ONLY
    #
    # Inputs:
    #
    # /zedx_front/zed_node/left/color/rect/image
    # /zedx_front/zed_node/left/color/rect/camera_info
    #
    # /zedx_front/zed_node/depth/depth_registered
    # /zedx_front/zed_node/depth/camera_info
    #
    # ================================================================

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
            "nvblox_cameras": "zedx_front",
            "use_sim_time": use_sim_time,
        }.items(),

        condition=IfCondition(
            PythonExpression([
                "'",
                enable_nvblox,
                "' == 'True' and '",
                stereo_camera_configuration,
                "' != 'no_cameras'",
            ])
        ),
    )

    # ================================================================
    # OAK-D
    # ================================================================
    #
    # IMPORTANT:
    #
    # We are not changing the OAK architecture as part of this ZED
    # refactor.
    #
    # The existing OAK service / OAK launch should remain controlled by
    # the rest of the MANRIIX bringup stack.
    #
    # If your current perceptor_include.launch.py already contains a
    # specific OAK IncludeLaunchDescription or SSH/systemd launcher,
    # retain that block here.
    #
    # This file deliberately does NOT attempt to launch a second OAK
    # process because the camera is running on the other Jetson.
    # ================================================================

    oak_enabled_log = LogInfo(
        condition=IfCondition(
            enable_oak_obstacles
        ),
        msg=(
            "[PERCEPTOR] OAK obstacle perception enabled. "
            "OAK hardware remains managed by the existing "
            "MANRIIX OAK service/remote Jetson architecture."
        ),
    )

    # ================================================================
    # Startup information
    # ================================================================

    startup_log = LogInfo(
        msg=[
            "\n",
            "=" * 72,
            "\n",
            "[PERCEPTOR] MANRIIX perception configuration\n",
            "\n",
            "  ZED architecture:\n",
            "    ONE multi_zed_capture_node\n",
            "\n",
            "  FRONT ZED:\n",
            "    Serial:        41911351\n",
            "    FPS:           ",
            zed_fps,
            "\n",
            "    Resolution:    ",
            zed_resolution,
            "\n",
            "    Depth:         ",
            front_zed_depth_mode,
            "\n",
            "    cuVSLAM:       enabled\n",
            "    nvblox source: enabled\n",
            "\n",
            "  LEFT ZED:\n",
            "    Serial:        40487925\n",
            "    Depth:         NONE\n",
            "    Monitor only:  yes\n",
            "\n",
            "  RIGHT ZED:\n",
            "    Serial:        46311875\n",
            "    Depth:         NONE\n",
            "    Monitor only:  yes\n",
            "\n",
            "  Monitor rate:   ",
            zed_monitor_rate_hz,
            " Hz\n",
            "\n",
            "  DISPLAY:        :0\n",
            "  XAUTHORITY:      /run/user/1000/gdm/Xauthority\n",
            "\n",
            "  OLD direct_zed workers: DISABLED\n",
            "  Side supervisor:        DISABLED\n",
            "\n",
            "=" * 72,
        ],
    )

    # ================================================================
    # Assemble
    # ================================================================

    return LaunchDescription([

        # ------------------------------------------------------------
        # Arguments
        # ------------------------------------------------------------

        stereo_camera_configuration_arg,

        enable_nvblox_arg,

        enable_oak_obstacles_arg,

        enable_oak_voxel_costmap_arg,

        use_sim_time_arg,

        zed_fps_arg,

        zed_resolution_arg,

        front_zed_depth_mode_arg,

        zed_monitor_rate_arg,

        zed_monitor_width_arg,

        zed_imu_rate_arg,

        enable_left_stereo_arg,
        enable_right_stereo_arg,
        enable_left_depth_arg,
        enable_right_depth_arg,

        vslam_cameras_arg,
        # ------------------------------------------------------------
        # Environment
        # ------------------------------------------------------------

        set_display,

        set_xauthority,

        # ------------------------------------------------------------
        # Information
        # ------------------------------------------------------------

        startup_log,

        # ------------------------------------------------------------
        # Camera ownership
        # ------------------------------------------------------------

        multi_zed_capture,

        # Shared 2D detection
        shared_three_camera_detector,

        # ------------------------------------------------------------
        # Localization
        # ------------------------------------------------------------

        visual_slam_launch,

        # ------------------------------------------------------------
        # 3D mapping / obstacle perception
        # ------------------------------------------------------------

        nvblox_launch,

        # ------------------------------------------------------------
        # OAK status
        # ------------------------------------------------------------

        oak_enabled_log,
    ])