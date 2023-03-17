#!/usr/bin/env python3

# references:
# https://github.com/ROBOTIS-GIT/turtlebot3/blob/foxy-devel/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py

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

    ANG_VEL_STEP_SIZE = 0.1
    LIN_VEL_STEP_SIZE = 0.01

    # @todo adjust this message
    MSG = """
    Control Your TurtleBot3!
    ---------------------------
    Moving around:

            w
        a   s   d

    w/x : increase/decrease linear velocity (-0.22 ~ 0.22)
    a/d : increase/decrease angular velocity (-2.84 ~ 2.84)
    space key: force stop
    CTRL-C to quit
    """

    def __init__(self):
        """Telemetry operation of TurtleBot3(burger) with a keyboard"""
        super().__init__('teleop_burger')
        self.__publisher = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth = 10))
        self.__twist = Twist()
        self.set_angular_velocity(0.0, 0.0, 0.0)
        self.set_linear_velocity(0.0, 0.0, 0.0)

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

    def get_current_velocity(self):
        """Return current velocity: (angular, linear)"""
        angular = dict(x = self.__twist.angular.x, y = self.__twist.angular.y, z = self.__twist.angular.z)
        linear = dict(x = self.__twist.linear.x, y = self.__twist.linear.y, z = self.__twist.linear.z)
        return (angular, linear)


    def get_key(self):
        """Get the first character input from a keyboard"""
        key = ''
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)

        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        return key

    def print_velocity(self, velocity):
        """Print velocity"""
        print('currentry:\n\tangular velocity: {}\n\tlinear velocity: {}'.format(velocity[0], velocity[1]))

    def publish_velocity(self):
        """Send velocity to the topic"""
        self.__publisher.publish(self.__twist)
        # self.get_logger().info('Publishing: {}'.format(self.__twist))

    def set_angular_velocity(self, x, y, z):
        """Set the value of Twist.angular"""
        self.__twist.angular.x = x
        self.__twist.angular.y = y
        self.__twist.angular.z = z

    def set_linear_velocity(self, x, y, z):
        """Set the value of Twist.linear"""
        self.__twist.linear.x = x
        self.__twist.linear.y = y
        self.__twist.linear.z = z


def main():
    rclpy.init()
    teleop_burger = TeleopBurger()

    print(teleop_burger.MSG)

    target_lin_vel_x = 0.0
    target_ang_vel_z = 0.0

    try:
        while(1):
            # get current velocity
            current_ang_vel, current_lin_vel = teleop_burger.get_current_velocity()

            # get input from a keyboard
            key = teleop_burger.get_key()

            if key == 'w':
                # forward
                target_ang_vel_z = current_ang_vel['z']
                target_lin_vel_x = teleop_burger.check_linear_limit_velocity(current_lin_vel['x'] + teleop_burger.LIN_VEL_STEP_SIZE)
            elif key == 'a':
                # left rotation
                target_ang_vel_z = teleop_burger.check_angular_limit_velocity(current_ang_vel['z'] + teleop_burger.ANG_VEL_STEP_SIZE)
                target_lin_vel_x = current_lin_vel['x']
            elif key == 's':
                # backward
                target_ang_vel_z = current_ang_vel['z']
                target_lin_vel_x = teleop_burger.check_linear_limit_velocity(current_lin_vel['x'] - teleop_burger.LIN_VEL_STEP_SIZE)
            elif key == 'd':
                # right rotation
                target_ang_vel_z = teleop_burger.check_angular_limit_velocity(current_ang_vel['z'] - teleop_burger.ANG_VEL_STEP_SIZE)
                target_lin_vel_x = current_lin_vel['x']
            elif key == ' ':
                # stop
                target_ang_vel_z = 0.0
                target_lin_vel_x = 0.0
            else:
                # without change
                target_ang_vel_z = current_ang_vel['z']
                target_lin_vel_x = current_lin_vel['x']

            # update velocity
            teleop_burger.set_angular_velocity(0.0, 0.0, target_ang_vel_z)
            teleop_burger.set_linear_velocity(target_lin_vel_x, 0.0, 0.0)

            # if you change the velocity, print those values
            if (target_ang_vel_z != current_ang_vel['z']) or (target_lin_vel_x != current_lin_vel['x']):
                teleop_burger.print_velocity(teleop_burger.get_current_velocity())

            # publish message
            teleop_burger.publish_velocity()
    finally:
        # before exiting this program, stop TB3
        teleop_burger.set_angular_velocity(0.0, 0.0, 0.0)
        teleop_burger.set_linear_velocity(0.0, 0.0, 0.0)
        teleop_burger.publish_velocity()


if __name__ == '__main__':
    main()