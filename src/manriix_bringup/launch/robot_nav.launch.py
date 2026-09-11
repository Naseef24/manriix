#!/usr/bin/env python3
"""
================================================================================
Manriix Robot Bringup - 

(Direct ZED SDK)
Launches full robot with 3× ZED X cameras using Direct SDK (no ZED wrapper).
- All cameras managed by single direct_zed_detection_node
- Point clouds published for STVL (Nav2 3D obstacle avoidance)
- IMU published for EKF sensor fusion (front camera only)
- Optional YOLO detection (mode=full)

Modes:
  nav:  Point clouds + IMU only (lightweight, for navigation)
  full: Point clouds + IMU + YOLO detection + images (for photography)

File: /home/hype/manriix2_ws/src/manriix_bringup/launch/robot_nav.launch.py
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
    # pkg_teleop = get_package_share_directory('manriix_rc_teleop') 

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
        description='Launch ZED X cameras (3x) via Direct SDK'
    )

    use_d455_arg = DeclareLaunchArgument(
        'use_d455',
        default_value='false',
        description='Launch RealSense D455 camera (rear low levelobstacle detection)'
    )

    use_oak_arg = DeclareLaunchArgument(
        'use_oak',
        default_value='false',
        description='Launch OAK-D Pro W camera (front low level obstacle detection)'
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
        default_value='true',
        description='Launch ZMQ bridge for AI photo capture system'
    )

    camera_mode_arg = DeclareLaunchArgument(
        'camera_mode',
        default_value='nav',
        description='Camera mode: nav (point clouds + IMU) or full (+ YOLO detection)',
        choices=['nav', 'full']
    )

    camera_resolution_arg = DeclareLaunchArgument(
        'camera_resolution',
        default_value='SVGA',
        description='Camera resolution: SVGA, HD1080, HD1200',
        choices=['SVGA', 'HD1080', 'HD1200']
    )

    camera_depth_mode_arg = DeclareLaunchArgument(
        'camera_depth_mode',
        default_value='NEURAL_LIGHT',
        description='Depth mode: NEURAL_LIGHT, NEURAL, NEURAL_PLUS',
        choices=['NEURAL_LIGHT', 'NEURAL', 'NEURAL_PLUS']
    )

    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='15',
        description='Camera FPS'
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
    camera_mode = LaunchConfiguration('camera_mode')
    camera_resolution = LaunchConfiguration('camera_resolution')
    camera_depth_mode = LaunchConfiguration('camera_depth_mode')
    camera_fps = LaunchConfiguration('camera_fps')

    # ==================== ROBOT DESCRIPTION ====================
    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_description, 'urdf', 'manriix3.urdf.xacro'),
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

    # manriix_rc_teleop = Node(
    #     package='manriix_rc_teleop',
    #     executable='manriix_rc_teleop',
    #     name='manriix_rc_teleop',
    #     output='screen',
    #     # condition=IfCondition(PythonExpression(["'", hardware_mode, "' == 'real'"]))
    # )

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

    # ==================== ZED X CAMERAS (3x) - DIRECT SDK ====================
    # Single node manages all 3 cameras
    # Publishes: point clouds (STVL), IMU (EKF), optionally detection + images
    direct_zed_node = TimerAction(
        period=8.0,
        actions=[
            GroupAction(
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
                        "  Topics:\n",
                        "    /zedx_front/zed_node/point_cloud/cloud_registered\n",
                        "    /zedx_left/zed_node/point_cloud/cloud_registered\n",
                        "    /zedx_right/zed_node/point_cloud/cloud_registered\n",
                        "    /zedx_front/zed_node/imu/data\n",
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
                            'camera_left_serial': 40487925,
                            'camera_right_serial': 46311875,
                            'resolution': camera_resolution,
                            'depth_mode': camera_depth_mode,
                            'fps': camera_fps,
                            'front_resolution': '',
                            'front_depth_mode': '',
                            'publish_rate': 15.0,
                            'pointcloud_downsample': 4,
                            'confidence_threshold': 30,#50,
                            'detection_model': 'YOLO11',
                            'yolo_model_path': '/home/hype/models/yolo11/yolo11m.onnx',#'/home/hype/models/yolo11/yolo11s.onnx',
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
            )
        ]
    )

    # ==================== ZMQ BRIDGE (for AI Photo Capture System) ====================
    # Forwards camera images from ROS topics to ZeroMQ IPC
    # Launches 10s after cameras to ensure topics are available
    zmq_bridge_node = TimerAction(
        period=10.0,
        actions=[
            GroupAction(
                condition=IfCondition(
                    PythonExpression([
                        "'", use_zedx, "' == 'true' and '",
                        launch_zmq_bridge, "' == 'true'"
                    ])
                ),
                actions=[
                    LogInfo(msg="[ZMQ Bridge] Starting camera image bridge to IPC (RGB + Depth + CameraInfo)"),
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
                            'sync_timeout': 0.05,      # 50ms max time diff for RGB+depth sync
                            'buffer_size': 10,         # Max frames buffered per stream
                            'use_sim_time': use_sim_time,
                        }],
                        respawn=True,
                        respawn_delay=5.0,
                    ),
                ]
            )
        ]
    )

    # ==================== PHOTO CAPTURE BRIDGE (MQTT) ====================
    # Forwards detection results to AI photo capture system via MQTT
    # Launches 15s after cameras to ensure detection topics are available
    photo_capture_bridge_node = TimerAction(
        period=15.0,
        actions=[
            GroupAction(
                condition=IfCondition(
                    PythonExpression([
                        "'", use_zedx, "' == 'true' and '",
                        camera_mode, "' == 'full'"
                    ])
                ),
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

    # ==================== DEPTH IMAGE → LASERSCAN (D455) ====================
    depth_to_scan_config = os.path.join(
        get_package_share_directory('manriix_description'), 'config', 'depth_to_scan.yaml')

    depth_timestamp_fix_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='manriix_bringup',
                executable='depth_timestamp_fix_node.py',
                name='depth_timestamp_fix',
                output='screen',
            )
        ]
    )
    
    depth_to_scan_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='depthimage_to_laserscan',
                executable='depthimage_to_laserscan_node',
                name='depthimage_to_laserscan',
                output='screen',
                parameters=[depth_to_scan_config, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('depth', '/camera/depth/image_rect_raw_fixed'),
                    ('depth_camera_info', '/camera/depth/camera_info_fixed'),
                    ('scan', '/scan_rear'),
                ],
            )
        ]
    )

    # ==================== LASER SCAN MERGER (RPLidar + D455) ====================
    scan_merger_config = os.path.join(
        get_package_share_directory('manriix_description'), 'config', 'scan_merger.yaml')
    
    scan_merger_node = TimerAction(
        period=10.0,
        actions=[
            ComposableNodeContainer(
                name='scan_merger_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='dual_laser_merger',
                        plugin='merger_node::MergerNode',
                        name='dual_laser_merger',
                        parameters=[scan_merger_config],
                        remappings=[
                            ('merged', '/scan_combined'),
                        ],
                    )
                ],
                output='screen',
            )
        ]
    )    

    # ==================== RETURN LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        SetEnvironmentVariable('DISPLAY', ':0'),
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
        camera_mode_arg,
        camera_resolution_arg,
        camera_depth_mode_arg,
        camera_fps_arg,

        # Core nodes
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        controller_spawner,

        # Motor diagnostics (real hardware only)
        motor_diagnostics,
        # manriix_rc_teleop,
        # Teleop
        teleop_launch,
        # EKF
        ekf_node,

        # Sensors (RPLidar)
        rplidar_launch,
        # ZED X Cameras (3x) - Direct SDK (replaces ZED wrapper)
        direct_zed_node,
        # OAK-D Pro W — low-front obstacle detection
        oak_d_pro_w_node,
        # D455 RealSense (rear obstacle detection)
        d455_node,

        # ZMQ Bridge (camera images to IPC for AI photo capture)
        zmq_bridge_node,
        # Photo Capture Bridge (MQTT) — only in full camera mode
        photo_capture_bridge_node,
        
        # Depth image to laserscan (D455 → /scan_rear)
        # depth_to_scan_node,

        # Laser scan merger (RPLidar + D455 → /scan_combined)
        # scan_merger_node,  

        # D455 timestamp fix (republish with ROS system time)
        # depth_timestamp_fix_node,

        # Node(
        #     package='foxglove_bridge',
        #     executable='foxglove_bridge',
        #     name='foxglove_bridge',
        #     parameters=[{
        #         'port': 8765,
        #         'address': '0.0.0.0',
        #         'send_buffer_limit': 10000000,
        #         'num_threads': 2,
        #     }],
        # ),

    ])
