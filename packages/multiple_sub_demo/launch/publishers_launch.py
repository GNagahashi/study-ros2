from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ros2 run multiple_sub_demo sensor_a
    sensor_a = Node(
        package = 'multiple_sub_demo',
        executable = 'sensor_a',
    )
    # ros2 run multiple_sub_demo sensor_b
    sensor_b = Node(
        package = 'multiple_sub_demo',
        executable = 'sensor_b',
    )

    return LaunchDescription([
        sensor_a,
        sensor_b,
    ])