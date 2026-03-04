from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch CAN Sniffer for debugging"""
    
    # Declare arguments
    can_channel_arg = DeclareLaunchArgument(
        'can_channel',
        default_value='can2',
        description='CAN bus interface name'
    )
    
    bitrate_arg = DeclareLaunchArgument(
        'bitrate',
        default_value='1000000',
        description='CAN bus bitrate'
    )
    
    show_extended_arg = DeclareLaunchArgument(
        'show_extended',
        default_value='false',
        description='Show extended CAN frames'
    )
    
    # Sniffer node
    sniffer_node = Node(
        package='zllg80asm250_monitor',
        executable='can_sniffer.py',
        name='can_sniffer',
        output='screen',
        parameters=[{
            'can_channel': LaunchConfiguration('can_channel'),
            'bitrate': LaunchConfiguration('bitrate'),
            'show_extended': LaunchConfiguration('show_extended'),
        }]
    )
    
    return LaunchDescription([
        can_channel_arg,
        bitrate_arg,
        show_extended_arg,
        sniffer_node,
    ])
