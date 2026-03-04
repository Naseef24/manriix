from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch ZLLG80ASM250 V1.0 Motor Monitor
    
    Motor: ZLLG80ASM250-L-24V
    Kt = 0.8 Nm/A
    """
    
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
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='Data update rate in Hz'
    )
    
    kt_arg = DeclareLaunchArgument(
        'kt_constant',
        default_value='0.8',
        description='Motor torque constant (Nm/A) for ZLLG80ASM250'
    )
    
    # Monitor node
    monitor_node = Node(
        package='zllg80asm250_monitor',
        executable='motor_monitor.py',
        name='zllg80asm250_monitor',
        output='screen',
        parameters=[{
            'can_channel': LaunchConfiguration('can_channel'),
            'bitrate': LaunchConfiguration('bitrate'),
            'update_rate': LaunchConfiguration('update_rate'),
            'kt_constant': LaunchConfiguration('kt_constant'),
        }]
    )
    
    return LaunchDescription([
        can_channel_arg,
        bitrate_arg,
        update_rate_arg,
        kt_arg,
        monitor_node,
    ])
