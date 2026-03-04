from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch ZLLG80ASM250 V1.0 Diagnostics
    Publishes motor data to ROS2 topics
    
    Topics:
        /zllg80asm250/voltage       - Bus voltage (V)
        /zllg80asm250/power         - Total power (W)
        /zllg80asm250/currents      - [LF, LB, RF, RB] in Amps
        /zllg80asm250/torques_nm    - [LF, LB, RF, RB] in Nm
        /zllg80asm250/speeds        - [LF, LB, RF, RB] in RPM
        /zllg80asm250/motor_temps   - [Left, Right] in °C
        /zllg80asm250/driver_temps  - [D1, D2] in °C
        /zllg80asm250/status_json   - Complete status as JSON
        /zllg80asm250/faults        - Fault descriptions
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
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Topic publish rate in Hz'
    )
    
    kt_arg = DeclareLaunchArgument(
        'kt_constant',
        default_value='0.8',
        description='Motor torque constant (Nm/A) for ZLLG80ASM250'
    )
    
    # Diagnostics node
    diagnostics_node = Node(
        package='zllg80asm250_monitor',
        executable='can_diagnostics.py',
        name='zllg80asm250_diagnostics',
        output='screen',
        parameters=[{
            'can_channel': LaunchConfiguration('can_channel'),
            'bitrate': LaunchConfiguration('bitrate'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'kt_constant': LaunchConfiguration('kt_constant'),
        }]
    )
    
    return LaunchDescription([
        can_channel_arg,
        bitrate_arg,
        publish_rate_arg,
        kt_arg,
        diagnostics_node,
    ])
