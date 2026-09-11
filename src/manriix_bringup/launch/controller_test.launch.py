import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    
    # ============================================
    # GET PACKAGE PATHS
    # ============================================
    description_pkg = get_package_share_directory('manriix_description')
    hardware_pkg = get_package_share_directory('manriix_hardware')
    
    # ============================================
    # FILE PATHS
    # ============================================
    urdf_file = os.path.join(description_pkg, 'urdf', 'manriix2.urdf.xacro')
    controllers_file = os.path.join(hardware_pkg, 'config', 'manriix_controllers.yaml')
    rviz_file = os.path.join(description_pkg, 'rviz2', 'display.rviz')
    
    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # ============================================
    # ROBOT STATE PUBLISHER
    # ============================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # ============================================
    # CONTROLLER MANAGER
    # ============================================
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ],
        output='screen'
    )
    
    # ============================================
    # SPAWN CONTROLLERS (with delay)
    # ============================================
    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # 4WS Controller (spawn after joint_state_broadcaster)
    manriix_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['manriix_4ws_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # ============================================
    # RVIZ
    # ============================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        output='screen'
    )
    
    # ============================================
    # DELAYED SPAWNING
    # ============================================
    # Wait for controller_manager to start before spawning controllers
    delayed_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_manriix_controller = TimerAction(
        period=3.0,
        actions=[manriix_controller_spawner]
    )
    
    # ============================================
    # LAUNCH DESCRIPTION
    # ============================================
    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        delayed_joint_state_broadcaster,
        delayed_manriix_controller,
        rviz
    ])

# ### Understanding the Launch File:
# ```
# ┌─────────────────────────────────────────────────────────────────────────┐
# │                       LAUNCH SEQUENCE                                   │
# ├─────────────────────────────────────────────────────────────────────────┤
# │                                                                         │
# │  T=0s:  Start robot_state_publisher                                     │
# │         Start controller_manager (ros2_control_node)                    │
# │         Start RViz                                                      │
# │                                                                         │
# │  T=2s:  Spawn joint_state_broadcaster                                   │
# │         (Publishes /joint_states for RViz)                              │
# │                                                                         │
# │  T=3s:  Spawn manriix_4ws_controller                                    │
# │         (Our 4WS controller!)                                           │
# │                                                                         │
# ├─────────────────────────────────────────────────────────────────────────┤
# │                                                                         │
# │  After launch, these topics will be available:                          │
# │                                                                         │
# │    /joint_states              ← Joint positions/velocities              │
# │    /manriix_4ws_controller/cmd_vel  ← Send velocity commands            │
# │    /manriix_4ws_controller/odom     ← Robot odometry                    │
# │    /tf                        ← Transforms                              │
# │                                                                         │
# └─────────────────────────────────────────────────────────────────────────┘

