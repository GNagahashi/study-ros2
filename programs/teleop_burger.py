#!/usr/bin/env python3

# references:
# https://github.com/ROBOTIS-GIT/turtlebot3/blob/foxy-devel/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py

#
# import library
#

import select
import sys
import termios
import tty

import rclpy

from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist


#
# define constant
#

BURGER_MAX_ANG_VEL = 2.84
BURGER_MAX_LIN_VEL = 0.22

ANG_VEL_STEP_SIZE = 0.1
LIN_VEL_STEP_SIZE = 0.01


#
# define variables
#

current_velocity = Twist()
next_velocity = Twist()


#
# define functions
#

def get_key():
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

def constrain(input_vel, low_bound, high_bound):
    """Adjust velocity"""
    if input_vel < low_bound:
        # when `input_vel` is lower than the threshold, return the `low_bound`
        return_vel = low_bound
    elif input_vel > high_bound:
        # when `input_vel` is higher than the threshold, return the `high_bound`
        return_vel = high_bound
    else:
        return_vel = input_vel

    return return_vel

def check_lin_limit_velocity(input_vel):
    """Verify that the linear velocity does not exceed the threshold."""
    return constrain(input_vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

def check_ang_limit_velocity(input_vel):
    """Verify that the angular velocity does not exceed the threshold"""
    return constrain(input_vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

def print_velocity(velocity):
    """Print velocity"""
    pass

def set_velocity(target_vel, linear, angular):
    """Set the values for Twist"""
    target_vel.linear.x = linear['x']
    target_vel.linear.y = linear['y']
    target_vel.linear.z = linear['z']
    target_vel.angular.x = angular['x']
    target_vel.angular.y = angular['y']
    target_vel.angular.z = angular['z']


def main():
    # create node and publisher
    rclpy.init()
    node = rclpy.create_node('teleop_burger')
    pub = node.create_publisher(Twist, '/cmd_vel', QoSProfile(depth = 10))

    # initialize
    set_velocity(
        current_velocity,
        dict(x = 0.0, y = 0.0, z = 0.0),
        dict(x = 0.0, y = 0.0, z = 0.0),
    )
    set_velocity(
        next_velocity,
        dict(x = 0.0, y = 0.0, z = 0.0),
        dict(x = 0.0, y = 0.0, z = 0.0),
    )
    next_velocity_lin_x = 0.0
    next_velocity_ang_z = 0.0

    try:
        key = get_key()

        if key == 'w':
            # forward
            next_velocity_lin_x = current_velocity.linear.x + LIN_VEL_STEP_SIZE
            next_velocity_ang_z = current_velocity.angular.z
        elif key == 'a':
            # left rotation
            next_velocity_lin_x = current_velocity.linear.x
            next_velocity_ang_z = current_velocity.angular.z + ANG_VEL_STEP_SIZE
        elif key == 's':
            # backward
            next_velocity_lin_x = current_velocity.linear.x - LIN_VEL_STEP_SIZE
            next_velocity_ang_z = current_velocity.angular.z
        elif key == 'd':
            # right rotation
            next_velocity_lin_x = current_velocity.linear.x
            next_velocity_ang_z = current_velocity.angular.z - ANG_VEL_STEP_SIZE
        elif key == ' ':
            # stop
            next_velocity_lin_x = 0.0
            next_velocity_ang_z = 0.0
        else:
            # without change
            next_velocity_lin_x = current_velocity.linear.x
            next_velocity_ang_z = current_velocity.angular.z

        # update velocity
        set_velocity(
            next_velocity,
            dict(x = next_velocity_lin_x, y = 0.0, z = 0.0),
            dict(x = 0.0, y = 0.0, z = next_velocity_ang_z),
        )

        # if you change the velocity, print those values
        if next_velocity != current_velocity:
            print_velocity()

        # publish message
        pub.publish(next_velocity)
    finally:
        # before exiting this program, stop TB3
        set_velocity(
            next_velocity,
            dict(x = 0.0, y = 0.0, z = 0.0),
            dict(x = 0.0, y = 0.0, z = 0.0),
        )
        pub.publish(next_velocity)


if __name__ == '__main__':
    main()