from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ros2 run turtlesim turtlesim_node
    turtlesim = Node(
        package = 'turtlesim',
        executable = 'turtlesim_node',
        remappings = [
            ('/turtle1/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        turtlesim,
    ])