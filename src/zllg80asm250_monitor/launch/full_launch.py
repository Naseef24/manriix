from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch full ZLLG80ASM250 V1.0 debug suite
    
    Launches both diagnostics (topic publishing) and monitor (visualization)
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
    
    kt_arg = DeclareLaunchArgument(
        'kt_constant',
        default_value='0.8',
        description='Motor torque constant (Nm/A) for ZLLG80ASM250'
    )
    
    # Diagnostics node (publishes to topics)
    diagnostics_node = Node(
        package='zllg80asm250_monitor',
        executable='can_diagnostics.py',
        name='zllg80asm250_diagnostics',
        output='screen',
        parameters=[{
            'can_channel': LaunchConfiguration('can_channel'),
            'bitrate': LaunchConfiguration('bitrate'),
            'publish_rate': 10.0,
            'kt_constant': LaunchConfiguration('kt_constant'),
        }]
    )
    
    # Monitor node (visualization)
    monitor_node = Node(
        package='zllg80asm250_monitor',
        executable='motor_monitor.py',
        name='zllg80asm250_monitor',
        output='screen',
        parameters=[{
            'can_channel': LaunchConfiguration('can_channel'),
            'bitrate': LaunchConfiguration('bitrate'),
            'update_rate': 10.0,
            'kt_constant': LaunchConfiguration('kt_constant'),
        }]
    )
    
    return LaunchDescription([
        can_channel_arg,
        bitrate_arg,
        kt_arg,
        diagnostics_node,
        monitor_node,
    ])
