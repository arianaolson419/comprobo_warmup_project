#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class PersonFollow(object):
    def __init__(self):
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
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

    def pol_to_cart(self, radius, theta):
        x = np.cos(theta) * radius
        y = np.sin(theta) * radius
        return x, y

    def angle_normalize_radians(self, z):
        return np.arctan2(np.sin(z), np.cos(z))

    def process_scan(self, msg):
        valid_points = []
        for i, r in enumerate(msg.ranges[:-1]):
            if r != 0.0 and r < self.radius_max and r > self.radius_min:
                valid_points.append([r, self.angle_normalize_radians(np.deg2rad(i))])
        center_of_mass = np.mean(valid_points, axis=0)
        print(center_of_mass)
        if msg.ranges == [0.0] * 361:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
        elif np.abs(center_of_mass[1]) > 0.1:
            self.twist.linear.x = center_of_mass[0] - 0.5
            self.twist.angular.z = center_of_mass[1] * 0.75
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
        self.twist_pub.publish(self.twist)

    def run(self):
        rospy.spin()

follow = PersonFollow()
follow.run()
