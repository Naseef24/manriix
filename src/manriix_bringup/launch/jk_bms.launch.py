#!/usr/bin/env python3
"""
================================================================================
jk_bms.launch.py

Launches the JK BMS reader node.

Hardware:
  JK BMS, 8s LiFePO4, ~25.6V nominal, 120Ah
  Connected via USB-RS485 dongle on /dev/bms (udev symlink to /dev/ttyUSB2)
  FTDI serial number A50285BI

Topic:
  /main_battery/state    sensor_msgs/BatteryState @ 0.5 Hz

USAGE (standalone):
  ros2 launch manriix_bringup jk_bms.launch.py

USAGE (via main launch):
  ros2 launch manriix_bringup navigation.launch.py enable_battery_monitor:=True

File: ~/manriix2_ws/src/manriix_bringup/launch/jk_bms.launch.py
================================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/bms',
        description='Serial device for JK BMS (udev symlink preferred)',
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate for JK BMS communication',
    )

    bms_node = Node(
        package='manriix_bringup',
        executable='jk_bms_node.py',
        name='jk_bms_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            # Hardware spec — matches JK BMS firmware config
            'cell_count': 8,
            'design_capacity_ah': 120.0,
            'location': 'manriix_base',
            # Cell voltage thresholds (mirror of BMS firmware settings)
            'cell_uvp_v': 2.60,
            'cell_ovp_v': 3.60,
            'soc_0_v': 2.62,
            'soc_100_v': 3.56,
            'cell_balance_threshold_v': 0.005,            
        }],
        respawn=True,            # auto-restart if BMS dies (USB hiccup, etc.)
        respawn_delay=3.0,
    )

    banner = LogInfo(msg=[
        '\n',
        '======================================================\n',
        '[JK BMS] Battery monitor starting\n',
        '  Port:     ', LaunchConfiguration('port'), '\n',
        '  Baud:     ', LaunchConfiguration('baudrate'), '\n',
        '  Topic:    /main_battery/state (sensor_msgs/BatteryState)\n',
        '  Hardware: JK BMS 8s LiFePO4, 25.6V nominal, 120Ah\n',
        '======================================================',
    ])

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        banner,
        bms_node,
    ])