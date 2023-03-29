from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    talker_node = Node(
        package="lab1_pkg",
        executable="talker_node.py",
        parameters=[
                {'v': 10},
                {'d': 5},
            ],
    )
    listener_node = Node(
        package="lab1_pkg",
        executable="relay_node.py"
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
