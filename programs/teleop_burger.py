#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist


class TeleopBurger(Node):

    BURGER_MAX_ANG_VEL = 2.84
    BURGER_MAX_LIN_VEL = 0.22

    def __init__(self):
        """Telemetry operation of TurtleBot3(burger) with a keyboard"""
        super().__init__('teleop_burger')
        self.__publisher = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth = 10))
        self.__twist = Twist()
        self.set_angular_velocity([0.0, 0.0, 0.0])
        self.set_linear_velocity([0.0, 0.0, 0.0])

    def __constrain(self, input_vel, low_bound, high_bound):
        """Adjust velocity"""
        if input_vel < low_bound:
            return_vel = low_bound
        elif input_vel > high_bound:
            return_vel = high_bound
        else:
            return_vel = input_vel

        return return_vel

    def check_angular_limit_velocity(self, velocity):
        """Verify that the angular velocity does not exceed the threshold"""
        return self.__constrain(velocity, -self.BURGER_MAX_ANG_VEL, self.BURGER_MAX_ANG_VEL)

    def check_linear_limit_velocity(self, velocity):
        """Verify that the linear velocity does not exceed the threshold."""
        return self.__constrain(velocity, -self.BURGER_MAX_LIN_VEL, self.BURGER_MAX_LIN_VEL)

    def current_velocity(self):
        """Return current velocity: (angular, linear)"""
        angular = [self.__twist.angular.x, self.__twist.angular.y, self.__twist.angular.z]
        linear = [self.__twist.linear.x, self.__twist.linear.y, self.__twist.linear.z]
        return (angular, linear)


    def get_key(self):
        """Get the first character input from a keyboard"""
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)

        rlist, _, _ = select.select([sys.stdin], [], [])
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def print_velocity(self):
        pass

    def publish_velocity(self):
        """Send velocity to the topic"""
        self.__publisher.publish(self.__twist)
        # self.get_logger().info('Publishing: {}'.format(self.__twist))

    def set_angular_velocity(self, velocity):
        """Set the value of Twist.angular"""
        self.__twist.angular.x = velocity[0]
        self.__twist.angular.y = velocity[1]
        self.__twist.angular.z = velocity[2]

    def set_linear_velocity(self, velocity):
        """Set the value of Twist.linear"""
        self.__twist.linear.x = velocity[0]
        self.__twist.linear.y = velocity[1]
        self.__twist.linear.z = velocity[2]


def main():
    rclpy.init()
    teleop_burger = TeleopBurger()

    print('start')

    key = teleop_burger.get_key()
    if key == 'w':
        target_linear_velocity = teleop_burger.check_linear_limit_velocity(0.2)
    elif key == 's':
        target_linear_velocity = 0.0

    teleop_burger.set_linear_velocity([target_linear_velocity, 0.0, 0.0])
    teleop_burger.publish_velocity()

    teleop_burger.set_angular_velocity([0.0, 0.0, 0.0])
    teleop_burger.set_linear_velocity([0.0, 0.0, 0.0])
    teleop_burger.publish_velocity()

    print('end')
    pass


if __name__ == '__main__':
    main()