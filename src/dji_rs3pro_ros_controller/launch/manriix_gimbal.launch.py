#!/usr/bin/env python3

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
# from ament_index_python.packages import get_package_share_directory
# import os

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('dji_rs3pro_ros_controller')
    config_file = os.path.join(pkg_share, 'config', 'control_gimbal.yaml')
    
    # Start CAN Receiver
    can_receiver = Node(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver_node',
        parameters=[{'interface': 'can3'}],
        output='screen'
    )
    
    # Start CAN Sender
    can_sender = Node(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='socket_can_sender_node',
        parameters=[{'interface': 'can3'}],
        output='screen'
    )
    
    # Activate nodes after 2 seconds
    activate_nodes = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', 
                     'ros2 lifecycle set /socket_can_receiver_node configure && '
                     'ros2 lifecycle set /socket_can_receiver_node activate && '
                     'ros2 lifecycle set /socket_can_sender_node configure && '
                     'ros2 lifecycle set /socket_can_sender_node activate'],
                output='screen'
            )
        ]
    )
    
    # # Start gimbal controller after 3 seconds
    # gimbal_controller = TimerAction(
    #     period=3.0,
    #     actions=[
    #         Node(
    #             package='dji_rs3pro_ros_controller',
    #             executable='control_gimbal_node.py',
    #             name='gimbal_controller',
    #             output='screen',
    #             parameters=[config_file],
    #         )
    #     ]
    # )
    # Start gimbal controller after 3 seconds (as LifecycleNode)
    gimbal_controller = TimerAction(
        period=3.0,
        actions=[
            LifecycleNode(
                package='dji_rs3pro_ros_controller',
                executable='control_gimbal_node.py',
                name='gimbal_controller',
                namespace='',
                output='screen',
                parameters=[config_file],
            )
        ]
    )

    # Configure and activate gimbal after 5 seconds
    # (CAN nodes active at 2s + gimbal node starts at 3s + 2s settle)
    activate_gimbal = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'ros2 lifecycle set /gimbal_controller configure && '
                     'ros2 lifecycle set /gimbal_controller activate'],
                output='screen'
            )
        ]
    )

    # # Start vision bridge after 5 seconds
    # # Gives gimbal controller time to initialize first
    # pkg_photographer = get_package_share_directory('manriix_photographer')
    # vision_bridge = TimerAction(
    #     period=7.0,
    #     actions=[
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([
    #                 pkg_photographer, '/launch/vision_bridge.launch.py'
    #             ])
    #         )
    #     ]
    # )

    return LaunchDescription([
        # Enable colored output
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),
        
        # CAN nodes
        can_receiver,
        can_sender,
        activate_nodes,
        
        # Gimbal controller
        gimbal_controller, 
        activate_gimbal,        
        # # Vision bridge — starts 5s after gimbal
        # vision_bridge,               
    ])
