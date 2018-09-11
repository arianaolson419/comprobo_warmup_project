#!usr/bin/env python
from __future__ import print_function, division

import tty
import select
import sys
import termios

import rospy
from geometry_msgs.msg import Twist, Vector3

class Teleop(object):
    def __init__(self):
        rospy.init_node('teleop')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Initialze state for keybindings
        self.settings = termios.tcgetattr(sys.stdin)

        # Initial speed.
        self.linear_speed = 0.5
        # TODO: do math to convert from linear to angular!
        self.angular_speed = 0.5

        # Initialize the movement vectors such that the robot is stationary.
        # Put the linear velocity vector in a defined state. Only the x
        # component of the vector will change.
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0

        # Put the angular velocity vector in a defined state. Only the z
        # component of the vector will change.
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def increase_speed(self):
        max_speed = 1.0
        self.linear_speed += 0.1
        # TODO: do math to convert from linear to angular!
        self.angular_speed += 0.1
        
        if self.linear_speed > max_speed:
            self.linear_speed = max_speed

        if  self.angular_speed > max_speed:
            self.angular_speed = max_speed

    def decrease_speed(self):
        min_speed = 0.0
        self.linear_speed -= 0.1
        self.angular_speed -= 0.1

        if self.linear_speed < min_speed:
            self.linear_speed = min_speed

        if self.angular_speed < min_speed:
            self.angular_speed = min_speed

    def stop_motors(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

    def move_forward(self):
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = 0.0

    def move_backwards(self):
        self.twist.linear.x = -self.linear_speed
        self.twist.angular.z = 0.0

    def spin_clockwise(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = self.angular_speed

    def spin_counter_clockwise(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = -self.angular_speed

    def circle_clockwise(self):
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = self.angular_speed

    def circle_counter_clockwise(self):
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = -self.angular_speed

    def control_motors(self):
        """There are several different "modes": move forward, move backward,
        spin around center clockwise, spin counter clockwise, circle
        clockwise, circle counter clockwise. We can control the speed at
        which the robot operates in wach of these modes
        """
        #while key !- '\x03':   # cntrl-C
        key = None
        key = self.getKey()
        if key == 'k':
            self.stop_motors()
        elif key == 'i':
            self.move_forward()
        elif key == ',':
            self.move_backwards()
        # TODO: verify that this is a consistent key mapping.
        elif key == 'j':
            self.spin_clockwise()
        elif key == 'l':
            self.spin_counter_clockwise()
        elif key == 'u':
            self.circle_clockwise()
        elif key == 'o':
            self.circle_counter_clockwise()
        elif key == 'm':
            self.increase_speed()
        elif key == '.':
            self.decrease_speed()
        elif key == '\x03':
            return
        print(self.linear_speed)
        print(self.angular_speed)
        self.pub.publish(self.twist)

    def run(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.control_motors()
            r.sleep()
                
op = Teleop()
op.run()
