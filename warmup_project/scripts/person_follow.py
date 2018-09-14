#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from neato_node.msg import Bump

"""
Defines a node that controls a Neato vacuum robot to detect and follow a person.
authors: Ariana Olson and Anna Buchele
"""

class PersonFollowNode(object):
    """A node that controls a robot such that it follows a "person" around.

    This node is written to be used with a Neato robot.
    """
    def __init__(self):
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
        rospy.Subscriber('/bump', Bump, self.emergency_stop)
        rospy.init_node('person_follow')

        self.twist = Twist()

        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        self.radius_max = 1.0
        self.radius_min = 0.3

        self.integrated_error_linear = 0.0
        self.integrated_error_angular = 0.0

    def pol_to_cart(self, radius, theta):
        x = np.cos(theta) * radius
        y = np.sin(theta) * radius
        return x, y

    def angle_normalize_radians(self, z):
        return np.arctan2(np.sin(z), np.cos(z))

    def process_scan(self, msg):
        rospy.on_shutdown(self.stop_motors)
        valid_points = []
        for i, r in enumerate(msg.ranges[:-1]):
            if r != 0.0 and r < self.radius_max and r > self.radius_min:
                valid_points.append([r, self.angle_normalize_radians(np.deg2rad(i))])
        if len(valid_points) == 0:
            self.stop_motors()

        elif any([msg.leftFront,
            msg.rightFront,
            msg.leftSide,
            msg.rightSide]):
            self.stop_motors()
        else:
            center_of_mass = np.mean(valid_points, axis=0)
            if np.abs(np.sum(center_of_mass)) > 0.1:
                self.twist.linear.x = center_of_mass[0] *0.75 + self.integrated_error_linear * 0.02
                self.twist.angular.z = center_of_mass[1] * 0.75 + self.integrated_error_angular * 0.02
            else:
                self.stop_motors()

            self.integrated_error_linear += center_of_mass[0]
            self.integrated_error_angular += center_of_mass[1]
        self.twist_pub.publish(self.twist)

    def emergency_stop(self, msg):
        if any([msg.leftFront,
            msg.rightFront,
            msg.leftSide,
            msg.rightSide]):
            self.stop_motors()

    def stop_motors(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.twist_pub.publish(self.twist)

    def run(self):
        rospy.spin()

follow = PersonFollowNode()
follow.run()
