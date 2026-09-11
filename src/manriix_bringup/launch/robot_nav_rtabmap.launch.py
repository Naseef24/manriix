#!/usr/bin/env python3
"""
================================================================================
Manriix Robot Bringup — STEP 3 (RTAB-Map + OAK-D)
================================================================================

Brings up the robot for RTAB-Map based SLAM using OAK-D Pro W as mapping sensor
and external odometry from EKF (wheel + ZED IMU fusion).

Differences from robot_nav.launch.py:
  - OAK-D Pro W launched with RTAB-Map-tuned config (RGBD pipeline)
  - use_oak default = true (OAK-D is essential for this stack)
  - All other components identical to robot_nav.launch.py

Architecture:
  Wheel encoders + ZED front IMU → EKF → /odometry/filtered → RTAB-Map (motion)
  OAK-D RGB + depth → RTAB-Map (mapping + loop closure)
  RTAB-Map → /map → Nav2

  ZED X cameras free for direct_zed_detection_node (point clouds + IMU + detection)

Launch sequence:
  Terminal 1: ros2 launch manriix_bringup robot_nav_rtabmap.launch.py \\
              hardware_mode:=real launch_ekf:=true

  Terminal 2: ros2 launch manriix_navigation rtabmap_nav.launch.py

File: /home/hype/manriix2_ws/src/manriix_bringup/launch/robot_nav_rtabmap.launch.py
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
    pkg_hardware    = get_package_share_directory('manriix_hardware')
    pkg_control     = get_package_share_directory('manriix_control')
    pkg_rplidar     = get_package_share_directory('rplidar_ros')
    pkg_bringup     = get_package_share_directory('manriix_bringup')

    # ==================== LAUNCH ARGUMENTS ====================
    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode',
        default_value='mock',
        description='Hardware mode: mock, gazebo, ignition, real',
        choices=['mock', 'gazebo', 'ignition', 'real']
    )

    use_motor_diagnostics_arg = DeclareLaunchArgument(
        'use_motor_diagnostics', default_value='true',
        description='Launch motor diagnostics node (real hardware only)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )

    use_rplidar_arg = DeclareLaunchArgument(
        'use_rplidar', default_value='true',
        description='Use RPLidar 2D laser scanner'
    )

    use_zedx_arg = DeclareLaunchArgument(
        'use_zedx', default_value='true',
        description='Launch ZED X cameras (3x) via Direct SDK'
    )

    # OAK-D enabled by default for step 3 (it's the SLAM sensor here)
    use_oak_arg = DeclareLaunchArgument(
        'use_oak', default_value='true',
        description='Launch OAK-D Pro W camera (RTAB-Map RGBD sensor in step 3)'
    )

    use_d455_arg = DeclareLaunchArgument(
        'use_d455', default_value='false',
        description='Launch RealSense D455 camera'
    )

    launch_teleop_arg = DeclareLaunchArgument(
        'launch_teleop', default_value='true',
        description='Launch teleop'
    )

    # EKF essential for step 3 — RTAB-Map uses /odometry/filtered as external odom
    launch_ekf_arg = DeclareLaunchArgument(
        'launch_ekf', default_value='true',
        description='Launch EKF localization (required for RTAB-Map external odom)'
    )

    launch_zmq_bridge_arg = DeclareLaunchArgument(
        'launch_zmq_bridge', default_value='true',
        description='Launch ZMQ bridge for AI photo capture system'
    )

    camera_mode_arg = DeclareLaunchArgument(
        'camera_mode', default_value='nav',
        description='ZED camera mode: nav (point clouds + IMU) or full (+ YOLO)',
        choices=['nav', 'full']
    )

    camera_resolution_arg = DeclareLaunchArgument(
        'camera_resolution', default_value='SVGA',
        description='ZED camera resolution: SVGA, HD1080, HD1200',
        choices=['SVGA', 'HD1080', 'HD1200']
    )

    camera_depth_mode_arg = DeclareLaunchArgument(
        'camera_depth_mode', default_value='NEURAL_LIGHT',
        description='ZED depth mode',
        choices=['NEURAL_LIGHT', 'NEURAL', 'NEURAL_PLUS']
    )

    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps', default_value='15',
        description='ZED camera FPS'
    )

    # Get launch configurations
    hardware_mode         = LaunchConfiguration('hardware_mode')
    use_sim_time          = LaunchConfiguration('use_sim_time')
    use_motor_diagnostics = LaunchConfiguration('use_motor_diagnostics')
    use_rplidar           = LaunchConfiguration('use_rplidar')
    use_zedx              = LaunchConfiguration('use_zedx')
    use_oak               = LaunchConfiguration('use_oak')
    use_d455              = LaunchConfiguration('use_d455')
    launch_teleop         = LaunchConfiguration('launch_teleop')
    launch_ekf            = LaunchConfiguration('launch_ekf')
    launch_zmq_bridge     = LaunchConfiguration('launch_zmq_bridge')
    camera_mode           = LaunchConfiguration('camera_mode')
    camera_resolution     = LaunchConfiguration('camera_resolution')
    camera_depth_mode     = LaunchConfiguration('camera_depth_mode')
    camera_fps            = LaunchConfiguration('camera_fps')

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
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )]
    )

    controller_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['manriix_4ws_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    # ==================== MOTOR DIAGNOSTICS ====================
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

    # ==================== RPLIDAR ====================
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'rplidar_a2m12_launch.py')
        ),
        condition=IfCondition(use_rplidar)
    )

    # ==================== OAK-D Pro W (STEP 3 — RTAB-Map sensor) ====================
    # Uses RTAB-Map-tuned config: RGBD pipeline, NN disabled, IMU enabled
    # URDF position: x=0.40, y=-0.0375, z=-0.27 from base_link, pitch=-0.08
    oak_d_pro_w_node = TimerAction(
        period=45.0,
        actions=[GroupAction(
            condition=IfCondition(use_oak),
            actions=[
                LogInfo(msg=[
                    "\n",
                    "=" * 60, "\n",
                    "[OAK-D] Launching with RTAB-Map RGBD config\n",
                    "  Topics:\n",
                    "    /oak_d_pro_w/rgb/image_rect           (color)\n",
                    "    /oak_d_pro_w/rgb/camera_info\n",
                    "    /oak_d_pro_w/stereo/image_raw         (depth, aligned)\n",
                    "    /oak_d_pro_w/stereo/camera_info\n",
                    "    /oak_d_pro_w/imu/data                 (200Hz, not fused)\n",
                    "    /oak_d_pro_w/points                   (point cloud)\n",
                    "=" * 60,
                ]),
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
                            pkg_bringup, 'config', 'oak_d_pro_w_rtabmap.yaml'
                        ),
                        'rectify_rgb': 'false',     # RGB already rectified in RGBD pipeline
                        'publish_tf_from_calibration': 'false',  # URDF owns TF
                        'imu_from_descr': 'true',
                    }.items(),
                )
            ]
        )]
    )

    # ==================== ZED X CAMERAS (Direct SDK — unchanged from step 1/2) ====================
    direct_zed_node = TimerAction(
        period=8.0,
        actions=[GroupAction(
            condition=IfCondition(use_zedx),
            actions=[
                LogInfo(msg=[
                    "\n",
                    "=" * 60, "\n",
                    "[ZED X] Launching 3 cameras via Direct SDK\n",
                    "  Mode: ", camera_mode, "\n",
                    "  Resolution: ", camera_resolution, "\n",
                    "  Depth: ", camera_depth_mode, "\n",
                    "  FPS: ", camera_fps, "\n",
                    "  Role in step 3: Detection + EKF IMU source (NOT SLAM)\n",
                    "=" * 60,
                ]),
                Node(
                    package='manriix_perception',
                    executable='direct_zed_detection_node.py',
                    name='direct_zed_detection_node',
                    output='screen',
                    parameters=[{
                        'mode': camera_mode,
                        'camera_front_serial': 41911351,
                        'camera_left_serial':  40487925,
                        'camera_right_serial': 46311875,
                        'resolution': camera_resolution,
                        'depth_mode': camera_depth_mode,
                        'fps': camera_fps,
                        'front_resolution': '',
                        'front_depth_mode': '',
                        'publish_rate': 15.0,
                        'pointcloud_downsample': 4,
                        'confidence_threshold': 30,
                        'detection_model': 'YOLO11',
                        'yolo_model_path': '/home/hype/models/yolo11/yolo11m.onnx',
                        'publish_images': True,
                        'draw_bounding_boxes': True,
                        'image_scale': 0.5,
                        'jpeg_quality': 75,
                        'use_sim_time': use_sim_time,
                    }],
                    respawn=True,
                    respawn_delay=5.0,
                ),
            ]
        )]
    )

    # ==================== ZMQ BRIDGE ====================
    zmq_bridge_node = TimerAction(
        period=10.0,
        actions=[GroupAction(
            condition=IfCondition(PythonExpression([
                "'", use_zedx, "' == 'true' and '",
                launch_zmq_bridge, "' == 'true'"
            ])),
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
        )]
    )

    # ==================== PHOTO CAPTURE BRIDGE (full mode only) ====================
    photo_capture_bridge_node = TimerAction(
        period=15.0,
        actions=[GroupAction(
            condition=IfCondition(PythonExpression([
                "'", use_zedx, "' == 'true' and '",
                camera_mode, "' == 'full'"
            ])),
            actions=[
                LogInfo(msg="[Photo Capture Bridge] Starting MQTT bridge for detection results"),
                Node(
                    package='manriix_perception',
                    executable='photo_capture_bridge_node.py',
                    name='photo_capture_bridge',
                    output='screen',
                    parameters=[{
                        'mqtt_enabled': True,
                        'mqtt_host': 'localhost',
                        'mqtt_port': 1883,
                        'mqtt_topic': '/photo_capture/tracking_results',
                        'publish_objects': True,
                        'mqtt_objects_topic': '/photo_capture/objects_tracking',
                        'sync_enabled': True,
                        'sync_window': 0.2,
                        'cameras': ['front', 'left', 'right'],
                        'use_sim_time': use_sim_time,
                    }],
                    respawn=True,
                    respawn_delay=5.0,
                ),
            ]
        )]
    )

    # ==================== TELEOP ====================
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'teleop.launch.py')
        ),
        condition=IfCondition(launch_teleop)
    )

    # ==================== EKF (REQUIRED for step 3) ====================
    # Publishes /odometry/filtered → consumed by RTAB-Map as external odometry
    # Publishes tf: odom → base_footprint (cuVSLAM NOT used in step 3)
    #
    # CONFIG: ekf_rtabmap.yaml fuses:
    #   - Wheel odometry (manriix_4ws_controller)
    #   - ZED front IMU (high, chest-level)
    #   - OAK-D Pro W IMU (low, knee-level)
    #
    # Started AFTER OAK-D is up (period=50s) so EKF can find /oak_d_pro_w/imu/data
    # immediately on launch. EKF without all configured inputs will warn but operate
    # with what's available — restart EKF (or robot_nav) if inputs come up out of order.
    ekf_config = os.path.join(pkg_control, 'config', 'ekf_rtabmap.yaml')
    ekf_node = TimerAction(
        period=50.0,
        actions=[
            GroupAction(
                condition=IfCondition(launch_ekf),
                actions=[
                    LogInfo(msg="[EKF] Starting filter — wheel + ZED IMU + OAK-D IMU"),
                    Node(
                        package='robot_localization',
                        executable='ekf_node',
                        name='ekf_filter_node',
                        output='screen',
                        parameters=[
                            ekf_config,
                            {'use_sim_time': use_sim_time},
                        ],
                    ),
                ]
            )
        ]
    )

    # ==================== D455 (optional, unchanged) ====================
    d455_config = os.path.join(
        get_package_share_directory('manriix_description'), 'config', 'd455_config.yaml'
    )
    d455_node = TimerAction(
        period=5.0,
        actions=[GroupAction(
            condition=IfCondition(use_d455),
            actions=[Node(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                name='camera',
                namespace='camera',
                output='screen',
                parameters=[d455_config],
                respawn=True,
                respawn_delay=5.0,
            )]
        )]
    )

    # ==================== RETURN LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        SetEnvironmentVariable('DISPLAY', ':0'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT',
                               '[{severity}] [{name}]: {message}'),

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
        camera_mode_arg,
        camera_resolution_arg,
        camera_depth_mode_arg,
        camera_fps_arg,

        # Core nodes
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        controller_spawner,

        # Hardware
        motor_diagnostics,
        teleop_launch,
        ekf_node,
        rplidar_launch,

        # Cameras
        direct_zed_node,
        oak_d_pro_w_node,
        d455_node,

        # Bridges
        zmq_bridge_node,
        photo_capture_bridge_node,
    ])