import select
import sys
import termios
import tty

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist


class TeleopBurger(Node):

    BURGER_MAX_LIN_VEL = 0.22
    BURGER_MAX_ANG_VEL = 2.84

    def __init__(self):
        """Telemetry operation of TurtleBot3(burger) with a keyboard"""
        super().__init__('teleop_burger')
        self.__publisher = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth = 10))
        self.__twist = Twist()

    def __constrain():
        pass

    def check_angular_limit_velocity():
        pass

    def check_linear_limit_velocity():
        pass

    def current_velocity():
        """"""
        angular = [self.__twist.angular.x, self.__twist.angular.y, self.__twist.angular.z]
        linear = [self.__twist.linear.x, self.__twist.linear.y, self.__twist.linear.z]
        return (angular, linear)


    def get_key():
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

    def print_velocity():
        pass

    def publish_velocity():
        """"""
        self.__publisher.publish(self.__twist)
        # self.get_logger().info('Publishing: {}'.format(self.__twist))

    def set_angular_velocity(velocity):
        """Set the value of Twist.angular"""
        self.__twist.angular.x = velocity[0]
        self.__twist.angular.y = velocity[1]
        self.__twist.angular.z = velocity[2]

    def set_linear_velocity(velocity):
        """Set the value of Twist.linear"""
        self.__twist.linear.x = velocity[0]
        self.__twist.linear.y = velocity[1]
        self.__twist.linear.z = velocity[2]


def main():
    pass


if __name__ == '__main__':
    main()