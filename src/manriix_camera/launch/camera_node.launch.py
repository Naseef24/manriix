from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    local_streaming_arg = DeclareLaunchArgument(
        'local_streaming',
        default_value='true',
        description='Enable local HTTP streaming (Flask)'
    )
    
    internet_streaming_arg = DeclareLaunchArgument(
        'internet_streaming',
        default_value='true',
        description='Enable internet streaming with authentication'
    )
    
    # Camera node
    camera_node = Node(
        package='manriix_camera',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'local_streaming': LaunchConfiguration('local_streaming'),
            'internet_streaming': LaunchConfiguration('internet_streaming'),
        }]
    )
    
    return LaunchDescription([
        local_streaming_arg,
        internet_streaming_arg,
        camera_node
    ])
