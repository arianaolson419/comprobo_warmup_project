#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class PersonFollow(object):
    def __init__(self):
        rospy.init_node('perosn_follow')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/stable_scan', LaserScan, self.control_motors)
        self.max_radius = 2.0
        self.angular_speed = 1.0
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

    def pol_to_cart(self, angle_degrees, distance):
        x = np.cos(np.radians(angle_degrees))
        y = np.sin(np.radians(angle_degrees))
        return np.array([x, y])

    def cart_to_pol(self, cart_vector):
        r = np.sqrt(cart_vector[0] ** 2 + cart_vector[1] ** 2)
        theta = np.rad2deg(np.arctan2(cart_vector[1], cart_vector[0]))
        return np.array([theta, r])

    def calculate_center_of_mass(self, point_vectors):
        return np.mean(point_vectors, axis=0)
        
    def control_motors(self, msg):
        valid_ranges = []
        for i, r in enumerate(msg.ranges[:-1]):
            if r < self.max_radius and r != 0.0:
                valid_ranges.append(self.pol_to_cart(i, r))
        cart_vector = self.calculate_center_of_mass(np.array(valid_ranges))
        pol_vector = self.cart_to_pol(cart_vector)
        
        turn_time = np.deg2rad(180 + np.floor(pol_vector[0])) / self.angular_speed
        print(180 + np.floor(pol_vector[0]))
        print(turn_time)
        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(turn_time)
        
        while (rospy.Time.now() < end_time and not rospy.is_shutdown()):
            self.twist.angular.z = self.angular_speed
            self.publisher.publish(self.twist)
        self.twist.angular.z = 0.0

        

    def run(self):
        rospy.spin()
        
follow = PersonFollow()
follow.run()
