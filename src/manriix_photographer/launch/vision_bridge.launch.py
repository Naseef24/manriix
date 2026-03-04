#!/usr/bin/env python3
"""
ROS 2 Launch file for Vision Bridge Node

Launches the MQTT↔ROS2 bridge node with configurable parameters.

Usage:
    ros2 launch ai_photo vision_bridge.launch.py
    ros2 launch ai_photo vision_bridge.launch.py mqtt_broker:=192.168.1.100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
import os


def generate_launch_description():
    """Generate launch description for vision bridge node"""
    
    # Get the install directory for manriix_photographer
    manriix_photographer_install = get_package_prefix('manriix_photographer')
    manriix_photographer_bin = os.path.join(manriix_photographer_install, 'bin', 'manriix_photographer')
     
    
    # Declare launch arguments
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='localhost',
        description='MQTT broker address'
    )
    
    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )

    # AI Photographer Bridge (converts ROS2 topics to gimbal commands)
    ai_photographer_bridge_node = Node(
        package='manriix_photographer',
        executable='ai_photographer_bridge.py',
        name='ai_photographer_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    # Create bridge node
    vision_bridge_node = Node(
        package='manriix_photographer',
        executable='ros2_vision_bridge',
        name='vision_bridge_node',
        output='screen',
        parameters=[{
            'mqtt_broker': LaunchConfiguration('mqtt_broker'),
            'mqtt_port': LaunchConfiguration('mqtt_port'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        mqtt_broker_arg,
        mqtt_port_arg,
        LogInfo(msg=['Starting Vision Bridge Node...']),
        LogInfo(msg=['MQTT Broker: ', LaunchConfiguration('mqtt_broker')]),
        LogInfo(msg=['MQTT Port: ', LaunchConfiguration('mqtt_port')]),
        ai_photographer_bridge_node,
        vision_bridge_node,
    ])
