"""
Controls a Neato using keyboard commands similarly to teleop_twist.
Authors: arianaolson419, abuchele
"""
#!usr/bin/env python
from __future__ import print_function, division

import tty
import select
import sys
import termios

import rospy
from geometry_msgs.msg import Twist, Vector3

class Teleop(object):
    """Controls the robot using keyboard commands. The key mapping is as follows:
        k: stop
        i: move forward
        ,: move backward
        j: spin counter-clockwise
        u: circle counter-clockwise
        l: spin clockwise
        o: circle clockwise
        m: increase speed
        .: decrease speed
    """
    def __init__(self):
        rospy.init_node('teleop')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Initialze state for keybindings
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        self.last_key = None    # The last movement key pressed.

        # Initial speed.
        self.linear_speed = 0.5
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
        """Reads keyboard input.

        Returns
        -------
        A character representing the key that was read from stdin.
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def increase_speed(self):
        """Increases the linear and angular speeds of the robot."""
        max_speed = 1.0
        self.linear_speed += 0.1
        self.angular_speed += 0.1
        
        if self.linear_speed > max_speed:
            self.linear_speed = max_speed

        if  self.angular_speed > max_speed:
            self.angular_speed = max_speed

    def decrease_speed(self):
        """Decreases the linear and angular speed settings of the robot."""
        min_speed = 0.0
        self.linear_speed -= 0.1
        self.angular_speed -= 0.1

        if self.linear_speed < min_speed:
            self.linear_speed = min_speed

        if self.angular_speed < min_speed:
            self.angular_speed = min_speed

    def stop_motors(self):
        """Stops the robot."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

    def move_forward(self):
        """Moves the robot forward in a straight line (linear velocity only)."""
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = 0.0

    def move_backwards(self):
        """Moves the robot backwards in a straight line (linear velocity only)."""
        self.twist.linear.x = -self.linear_speed
        self.twist.angular.z = 0.0

    def spin_clockwise(self):
        """Spins the robot in place clockwise (angular velocity only)."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = self.angular_speed

    def spin_counter_clockwise(self):
        """Spins the robot in place counter-clockwise (angular velocity only)."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = -self.angular_speed

    def circle_clockwise(self):
        """Moves the robot forward clockwise in a circle (both linear and angular velocity)."""
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = self.angular_speed

    def circle_counter_clockwise(self):
        """Moves the robot forward counter-clockwise in a circle (both linear and angular velocity)."""
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = -self.angular_speed

    def update_velocity(self, key):
        """Updates the linear and angular velocity according to the given key.
        Parameters
        ----------
        key: a character representing the pressed key.
        """
        if key == 'k' or not key:
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

    def control_motors(self):
        """Interprets keyboard input and updates the state accordingly.
        Publishes the velocity.
        """
        movement_keys = ['k', 'i', ',', 'j', 'l', 'u', 'o']
        if self.key in movement_keys:
            self.last_key = self.key
        self.key = self.getKey()

        if self.key == '\x03':
            # Handles cntrl-c.
            return False
        elif self.key == 'm':
            self.increase_speed()
            # Updates the velocity with the new linear and angular speeds.
            self.update_velocity(self.last_key)
        elif self.key == '.':
            self.decrease_speed()
            # Updates the velocity with the new linear and angular speeds.
            self.update_velocity(self.last_key)
        else:
            self.update_velocity(self.key)
        
        self.pub.publish(self.twist)
        return True

    def run(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            if not self.control_motors():
                break
            r.sleep()
                
# Run the node.
op = Teleop()
op.run()
