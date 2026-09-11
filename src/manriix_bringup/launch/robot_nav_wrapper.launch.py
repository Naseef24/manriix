#!/usr/bin/env python3
"""
================================================================================
Manriix Robot Bringup — ZED ROS2 Wrapper variant

Launches the full robot with 3× ZED X cameras via the official Stereolabs
zed_ros2_wrapper instead of the custom direct_zed_detection_node.

Why this variant exists
-----------------------
The Direct SDK node (direct_zed_detection_node.py) is excellent for our
detection pipeline but does NOT publish what cuVSLAM and other Isaac ROS
stereo VO algorithms need:
  - raw uncompressed left+right rectified images (mono8 or bgr8)
  - rectified CameraInfo with baseline term in P[0,3]
  - hardware-timestamped IMU at >=200Hz

The official wrapper publishes all of those. This launch is the foundation
for the cuVSLAM integration (see cuvslam.launch.py).

What this launch DOES
---------------------
- robot_state_publisher, ros2_control, motor diagnostics — UNCHANGED
- RPLidar — UNCHANGED
- EKF, teleop — UNCHANGED
- 3× zed_wrapper instances (front primary + IMU, left, right) — NEW
- ZMQ bridge — kept (subscribes to wrapper topics now, not direct SDK)
- Optional D455 / OAK-D — UNCHANGED
- Detection pipeline — REMOVED for now (per the request)
- Photo capture MQTT bridge — REMOVED (depends on detection)

Minimal-Argus configuration
---------------------------
Running 3 zed_wrappers at full settings on a single AGX Orin causes
nvargus-daemon to fail capture sessions (NvScf errors). This launch
uses a minimal-Argus profile per camera:
  - depth_mode: NONE (we still get rectified stereo but no depth pipeline)
  - object_detection / body_tracking / pos_tracking: false
  - grab_resolution: SVGA (lower Argus bandwidth)
  - Front camera is the only one with IMU enabled
Side cameras run at the lowest viable settings so the system can boot
all 3 reliably.

File: /home/hype/manriix2_ws/src/manriix_bringup/launch/robot_nav_wrapper.launch.py
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    GroupAction, LogInfo, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    pkg_description = get_package_share_directory('manriix_description')
    pkg_hardware = get_package_share_directory('manriix_hardware')
    pkg_control = get_package_share_directory('manriix_control')
    pkg_rplidar = get_package_share_directory('rplidar_ros')
    pkg_bringup = get_package_share_directory('manriix_bringup')
    pkg_zed_wrapper = get_package_share_directory('zed_wrapper')

    # ==================== LAUNCH ARGUMENTS ====================
    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode',
        default_value='mock',
        description='Hardware mode: mock, gazebo, ignition, real',
        choices=['mock', 'gazebo', 'ignition', 'real']
    )

    use_motor_diagnostics_arg = DeclareLaunchArgument(
        'use_motor_diagnostics',
        default_value='true',
        description='Launch motor diagnostics node (real hardware only)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_rplidar_arg = DeclareLaunchArgument(
        'use_rplidar',
        default_value='true',
        description='Use RPLidar 2D laser scanner'
    )

    use_zedx_arg = DeclareLaunchArgument(
        'use_zedx',
        default_value='true',
        description='Launch 3× ZED X cameras via the official ROS2 wrapper'
    )

    use_d455_arg = DeclareLaunchArgument(
        'use_d455',
        default_value='false',
        description='Launch RealSense D455 camera (rear low-level obstacle detection)'
    )

    use_oak_arg = DeclareLaunchArgument(
        'use_oak',
        default_value='false',
        description='Launch OAK-D Pro W camera (front low-level obstacle detection)'
    )

    launch_teleop_arg = DeclareLaunchArgument(
        'launch_teleop',
        default_value='true',
        description='Launch teleop'
    )

    launch_ekf_arg = DeclareLaunchArgument(
        'launch_ekf',
        default_value='true',
        description='Launch EKF localization'
    )

    launch_zmq_bridge_arg = DeclareLaunchArgument(
        'launch_zmq_bridge',
        default_value='false',
        description='Launch ZMQ bridge for AI photo capture system'
    )

    # ZED wrapper config selection. We provide minimal-Argus configs for
    # the three cameras under manriix_bringup/config/. They override the
    # defaults shipped with zed_wrapper.
    zed_front_config_arg = DeclareLaunchArgument(
        'zed_front_config',
        default_value=os.path.join(pkg_bringup, 'config', 'zedx_front.yaml'),
        description='YAML config for front ZED X (primary, with IMU)'
    )
    zed_left_config_arg = DeclareLaunchArgument(
        'zed_left_config',
        default_value=os.path.join(pkg_bringup, 'config', 'zedx_left.yaml'),
        description='YAML config for left ZED X (secondary)'
    )
    zed_right_config_arg = DeclareLaunchArgument(
        'zed_right_config',
        default_value=os.path.join(pkg_bringup, 'config', 'zedx_right.yaml'),
        description='YAML config for right ZED X (secondary)'
    )

    # Get launch configurations
    hardware_mode = LaunchConfiguration('hardware_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_motor_diagnostics = LaunchConfiguration('use_motor_diagnostics')

    use_rplidar = LaunchConfiguration('use_rplidar')
    use_zedx = LaunchConfiguration('use_zedx')
    use_oak = LaunchConfiguration('use_oak')
    use_d455 = LaunchConfiguration('use_d455')

    launch_teleop = LaunchConfiguration('launch_teleop')
    launch_ekf = LaunchConfiguration('launch_ekf')

    launch_zmq_bridge = LaunchConfiguration('launch_zmq_bridge')
    zed_front_config = LaunchConfiguration('zed_front_config')
    zed_left_config = LaunchConfiguration('zed_left_config')
    zed_right_config = LaunchConfiguration('zed_right_config')

    # ==================== ROBOT DESCRIPTION ====================
    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_description, 'urdf', 'manriix2.urdf.xacro'),
        ' hardware_mode:=', hardware_mode
    ])

    controller_config = os.path.join(pkg_hardware, 'config', 'manriix_controllers.yaml')

    # ==================== CORE NODES ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': use_sim_time,
            'use_tf_static': False,
        }],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(robot_description_content, value_type=str)},
            controller_config
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            )
        ]
    )

    controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['manriix_4ws_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    # ==================== MOTOR DIAGNOSTICS (REAL HARDWARE ONLY) ====================
    motor_diagnostics = Node(
        package='manriix_hardware',
        executable='motor_diagnostics_node.py',
        name='motor_diagnostics',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", hardware_mode, "' == 'real' and '",
            use_motor_diagnostics, "' == 'true'"
        ]))
    )

    # ==================== SENSORS (RPLidar) ====================
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'rplidar_a2m12_launch.py')
        ),
        condition=IfCondition(use_rplidar)
    )

    oak_d_pro_w_node = TimerAction(
        period=45.0,
        actions=[
            GroupAction(
                condition=IfCondition(use_oak),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('depthai_ros_driver'),
                                'launch', 'camera_as_part_of_a_robot.launch.py'
                            )
                        ),
                        launch_arguments={
                            'name': 'oak_d_pro_w',
                            'parent_frame': 'oak_d_pro_w_camera_frame',
                            'params_file': os.path.join(
                                get_package_share_directory('manriix_description'),
                                'config', 'oak_d_pro_w_nav.yaml'
                            ),
                            'rectify_rgb': 'false',
                            'publish_tf_from_calibration': 'false',
                            'imu_from_descr': 'true',
                        }.items(),
                    )
                ]
            )
        ]
    )

    # ==================== ZED X CAMERAS (3x) — OFFICIAL ROS2 WRAPPER ====================
    # Cameras are launched in their own namespaces so all topics are prefixed:
    #   /zedx_front/zed_node/...
    #   /zedx_left/zed_node/...
    #   /zedx_right/zed_node/...
    #
    # IMPORTANT staggered timing: nvargus-daemon takes a few seconds per
    # capture session. We stagger the three wrapper launches by 6 seconds
    # each to give Argus time to settle before opening the next camera.
    # If Argus errors still appear, increase these intervals to 10s.

    zed_front_log = LogInfo(msg=[
        "\n", "=" * 60, "\n",
        "[ZED X — FRONT] Launching zed_wrapper (primary, with IMU)\n",
        "  Serial:     41911351\n",
        "  Namespace:  zedx_front\n",
        "  Config:     ", zed_front_config, "\n",
        "  Topics will appear under /zedx_front/zed_node/...\n",
        "=" * 60,
    ])

    zed_front_launch = TimerAction(
        period=5.0,
        actions=[
            GroupAction(
                condition=IfCondition(use_zedx),
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
        ]
    )

    zed_left_log = LogInfo(msg=[
        "\n", "=" * 60, "\n",
        "[ZED X — LEFT] Launching zed_wrapper (secondary, no IMU)\n",
        "  Serial:     40487925\n",
        "  Namespace:  zedx_left\n",
        "=" * 60,
    ])

    zed_left_launch = TimerAction(
        period=11.0,
        actions=[
            GroupAction(
                condition=IfCondition(use_zedx),
                actions=[
                    zed_left_log,
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_zed_wrapper, 'launch', 'zed_camera.launch.py')
                        ),
                        launch_arguments={
                            'camera_model': 'zedx',
                            'camera_name': 'zedx_left',
                            'serial_number': '40487925',
                            'ros_params_override_path': zed_left_config,
                            'publish_tf': 'false',
                            'publish_map_tf': 'false',
                            'publish_imu_tf': 'false',
                            'publish_urdf': 'false',
                            'node_log_type': 'screen',
                        }.items(),
                    ),
                ]
            )
        ]
    )

    zed_right_log = LogInfo(msg=[
        "\n", "=" * 60, "\n",
        "[ZED X — RIGHT] Launching zed_wrapper (secondary, no IMU)\n",
        "  Serial:     46311875\n",
        "  Namespace:  zedx_right\n",
        "=" * 60,
    ])

    zed_right_launch = TimerAction(
        period=17.0,
        actions=[
            GroupAction(
                condition=IfCondition(use_zedx),
                actions=[
                    zed_right_log,
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_zed_wrapper, 'launch', 'zed_camera.launch.py')
                        ),
                        launch_arguments={
                            'camera_model': 'zedx',
                            'camera_name': 'zedx_right',
                            # 'namespace': 'zedx_right',
                            'serial_number': '46311875',
                            'ros_params_override_path': zed_right_config,
                            'publish_tf': 'false',
                            'publish_map_tf': 'false',
                            'publish_imu_tf': 'false',
                            'publish_urdf': 'false',
                            'node_log_type': 'screen',
                        }.items(),
                    ),
                ]
            )
        ]
    )

    # ==================== ZMQ BRIDGE (for AI Photo Capture System) ====================
    # Now consumes wrapper topics instead of the Direct SDK node's output.
    # If your zed_zmq_bridge_node.py subscribes to topic NAMES that already
    # match (/zedx_*/zed_node/left/image_rect_color/compressed and ...
    # /point_cloud/cloud_registered and .../imu/data), no code change is needed.
    # Launched well after the three cameras are up.
    zmq_bridge_node = TimerAction(
        period=25.0,
        actions=[
            GroupAction(
                condition=IfCondition(
                    PythonExpression([
                        "'", use_zedx, "' == 'true' and '",
                        launch_zmq_bridge, "' == 'true'"
                    ])
                ),
                actions=[
                    LogInfo(msg="[ZMQ Bridge] Starting camera image bridge to IPC"),
                    Node(
                        package='manriix_perception',
                        executable='zed_zmq_bridge_node.py',
                        name='zed_zmq_bridge',
                        output='screen',
                        parameters=[{
                            'ipc_path': '/tmp/zedx_images.ipc',
                            'cameras': ['front', 'left', 'right'],
                            'queue_size': 1,
                            'publish_stats': True,
                            'stats_interval': 10.0,
                            'sync_timeout': 0.05,
                            'buffer_size': 10,
                            'use_sim_time': use_sim_time,
                        }],
                        respawn=True,
                        respawn_delay=5.0,
                    ),
                ]
            )
        ]
    )

    # ==================== TELEOP (OPTIONAL) ====================
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'teleop.launch.py')
        ),
        condition=IfCondition(launch_teleop)
    )

    # ==================== EKF LOCALIZATION (OPTIONAL) ====================
    ekf_config = os.path.join(pkg_control, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_ekf)
    )

    # ==================== D455 (UNCHANGED) ====================
    d455_config = os.path.join(
        get_package_share_directory('manriix_description'), 'config', 'd455_config.yaml'
    )

    d455_node = TimerAction(
        period=5.0,
        actions=[
            GroupAction(
                condition=IfCondition(use_d455),
                actions=[
                    Node(
                        package='realsense2_camera',
                        executable='realsense2_camera_node',
                        name='camera',
                        namespace='camera',
                        output='screen',
                        parameters=[d455_config],
                        respawn=True,
                        respawn_delay=5.0,
                    )
                ]
            )
        ]
    )

    # ==================== RETURN LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        SetEnvironmentVariable('DISPLAY', ':0'),
        SetEnvironmentVariable('XAUTHORITY', '/run/user/1000/gdm/Xauthority'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),

        # Arguments
        hardware_mode_arg,
        use_sim_time_arg,
        use_motor_diagnostics_arg,

        use_rplidar_arg,
        use_zedx_arg,
        use_oak_arg,
        use_d455_arg,

        launch_teleop_arg,
        launch_ekf_arg,

        launch_zmq_bridge_arg,
        zed_front_config_arg,
        zed_left_config_arg,
        zed_right_config_arg,

        # Core nodes
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        controller_spawner,

        # Motor diagnostics
        motor_diagnostics,

        # Teleop
        teleop_launch,
        # EKF
        ekf_node,

        # Sensors (RPLidar)
        rplidar_launch,

        # ZED X Cameras (3x) via official ROS2 wrapper, staggered
        zed_front_launch,
        zed_left_launch,
        zed_right_launch,

        # OAK-D Pro W
        oak_d_pro_w_node,
        # D455 RealSense
        d455_node,

        # ZMQ Bridge (optional)
        zmq_bridge_node,
    ])