# =============================================================================
# Manriix Bi3D isolation test — minimum environment for Bi3D
#
# Goal: see if Bi3D-Freespace works WITHOUT the rest of the stack
#       (no cuVSLAM, no nvblox, no Nav2, no controllers, no other ZEDs).
#
# Includes ONLY:
#   - robot_state_publisher (for base_footprint -> camera TF chain via URDF)
#   - 1× ZED X front camera with depth OFF (minimal Argus profile)
#
# Then run Bi3D from a second terminal:
#   ros2 launch manriix_navigation manriix_bi3d_freespace.launch.py
#
# Compare crash behavior vs. full stack to isolate concurrent CUDA pressure
# as the root cause of the convert_to_custom cudaErrorIllegalAddress.
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    SetEnvironmentVariable, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('manriix_description')
    pkg_bringup = get_package_share_directory('manriix_bringup')
    pkg_zed_wrapper = get_package_share_directory('zed_wrapper')

    # Use the MINIMAL front config (depth OFF) — same as production cuVSLAM path
    zed_front_config = os.path.join(pkg_bringup, 'config', 'zedx_front.yaml')

    # ---------- ARGUMENTS ----------
    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode', default_value='real',
        description='Hardware mode',
        choices=['mock', 'gazebo', 'ignition', 'real']
    )
    hardware_mode = LaunchConfiguration('hardware_mode')

    # ---------- URDF / ROBOT_STATE_PUBLISHER ----------
    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_description, 'urdf', 'manriix2.urdf.xacro'),
        ' hardware_mode:=', hardware_mode
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': False,
            'use_tf_static': False,
        }],
    )

    # ---------- ZED FRONT (DEPTH OFF, minimum Argus load) ----------
    zed_front_log = LogInfo(msg=[
        "\n", "=" * 60, "\n",
        "[ISOLATION TEST] Launching ONLY ZED front (depth OFF) + TF\n",
        "  Config:    zedx_front.yaml (depth_mode: NONE)\n",
        "  No cuVSLAM, no nvblox, no Nav2 — pure isolation\n",
        "=" * 60,
    ])

    zed_front_launch = TimerAction(
        period=2.0,
        actions=[
            zed_front_log,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_zed_wrapper, 'launch', 'zed_camera.launch.py')
                ),
                launch_arguments={
                    'camera_model': 'zedx',
                    'camera_name': 'zedx_front',
                    'serial_number': '41911351',
                    'ros_params_override_path': zed_front_config,
                    'publish_tf': 'false',
                    'publish_map_tf': 'false',
                    'publish_imu_tf': 'true',
                    'publish_urdf': 'false',
                    'node_log_type': 'screen',
                }.items(),
            ),
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('DISPLAY', ':0'),
        SetEnvironmentVariable('XAUTHORITY', '/run/user/1000/gdm/Xauthority'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT',
                                '[{severity}] [{name}]: {message}'),
        hardware_mode_arg,
        robot_state_publisher,
        zed_front_launch,
    ])
