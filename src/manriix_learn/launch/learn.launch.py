#!/usr/bin/env python3
"""
learn.launch.py
Starts action server and client together.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    action_server = Node(
        package='manriix_learn',
        executable='action_server_node.py',
        name='manriix_learn_action_server',
        output='screen'
    )

    #start the client after a delay to ensure the server is up
    action_client = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='manriix_learn',
                executable='action_client_node.py',
                name='manriix_learn_action_client',
                output='screen'
            )
        ]
    )       

    return LaunchDescription([
        action_server,
        action_client
    ])