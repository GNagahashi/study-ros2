#!/usr/bin/env python3

# references:
# https://github.com/ros2/message_filters

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32
from std_msgs.msg import Float32

from message_filters import (
    ApproximateTimeSynchronizer,
    Subscriber,
)


class MultipleSub(Node):

    def __init__(self):
        super().__init__('multiple_sub')
        self.sub_for_sensor_a = Subscriber(self, Float32, 'sensor_A', qos_profile = qos_profile_sensor_data)
        self.sub_for_sensor_b = Subscriber(self, Int32, 'sensor_B', qos_profile = qos_profile_sensor_data)
        self.ats = ApproximateTimeSynchronizer(
            [self.sub_for_sensor_a, self.sub_for_sensor_b],  # subscribers
            50,  # queue size
            1.0,  # delay
            allow_headerless = True,
        )
        self.ats.registerCallback(self.multiple_sub_callback)

    def multiple_sub_callback(self, msg_sensor_a, msg_sensor_b):
        self.get_logger().info(f'sensor_A: {msg_sensor_a.data:.3f}, sensor_B: {msg_sensor_b.data:.3f}')


def main():
    rclpy.init()
    node = MultipleSub()

    rclpy.spin(node)


if __name__ == '__main__':
    main()