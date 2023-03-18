#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32


class DummySensorB(Node):

    def __init__(self):
        super().__init__('dummy_sensor_B')
        self.pub = self.create_publisher(Int32, 'sensor_B', qos_profile_sensor_data)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.num = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.num
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data:.3f}')
        self.num = self.num + 1


def main():
    rclpy.init()
    node = DummySensorB()

    rclpy.spin(node)


if __name__ == '__main__':
    main()