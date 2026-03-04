import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Package paths
    gazebo_pkg = get_package_share_directory('manriix_gazebo')
    manriix_desc_pkg = get_package_share_directory('manriix_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # File paths
    xacro_file = os.path.join(gazebo_pkg, 'urdf', 'manriix_gazebo.urdf.xacro')
    world_file = os.path.join(gazebo_pkg, 'worlds', 'empty_ignition.sdf')

    # Process xacro
    robot_description = xacro.process_file(xacro_file).toxml()

    # Set mesh path for Ignition to find STL files
    install_share_dir = os.path.dirname(manriix_desc_pkg)
    
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=install_share_dir
    )

    # Ignition Gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # Bridge clock from Ignition to ROS2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Spawn Robot in Ignition
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'manriix',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Joint State Broadcaster
    joint_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 4WS Controller
    controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['manriix_4ws_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        set_ign_resource_path,
        ignition_gazebo,
        clock_bridge,
        robot_state_publisher,
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=10.0, actions=[joint_broadcaster]),
        TimerAction(period=12.0, actions=[controller])
    ])