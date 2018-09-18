#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from neato_node.msg import Bump
from visualization_msgs.msg import Marker

"""
Person following. Part of the CompRobo warmup project.

authors: Ariana Olson and Anna Buchele
"""

class PersonFollowNode(object):
    """A node that controls a robot such that it follows a "person" around.

    Data from the Neato's LIDAR scanner is used to detect objects and control
    the movement of the robot.
    """
    def __init__(self):
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vis_pub =rospy.Publisher('/visualization_marker', Marker, queue_size=10)
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

        self.mark = Marker()
        
        self.mark.ns = "person_follow"
        self.mark.id = 0
        self.mark.type = 2
        self.mark.action = 0
        self.mark.scale.x = 0.1
        self.mark.scale.y = 0.1
        self.mark.scale.z = 0.1
        self.mark.color.r = 0.0
        self.mark.color.g = 1.0
        self.mark.color.b = 0.0
        self.mark.color.a = 1.0
        self.mark.pose.position.x = 0.0
        self.mark.pose.position.y = 0.0
        self.mark.pose.position.z = 0.0
        self.mark.pose.orientation.x = 0.0
        self.mark.pose.orientation.y = 0.0
        self.mark.pose.orientation.z = 0.0
        self.mark.pose.orientation.w = 1.0
        self.mark.header.frame_id = "base_link"
        self.mark.header.stamp = rospy.Time()

        self.radius_max = 1.5
        # A minimum is needed to ignore LIDAR detecting parts of the robot.
        self.radius_min = 0.3 

        self.integrated_error_linear = 0.0
        self.integrated_error_angular = 0.0

    def pol_to_cart(self, radius, angle):
        """Convert  laser scan data to two-dimensional cartesian coordinates.

        Parameters
        ----------
        radius: the range of the LIDAR scan point.
        angle: the angle in radians of the LIDAR scan point.

        Returns
        -------
        a tuple (x, y) of cartesian coordinates.
        """
        x = np.cos(angle) * radius
        y = np.sin(angle) * radius
        return x, y

    def angle_normalize_radians(self, z):
        """Normalizes an angle in radians to between -pi and pi."""
        return np.arctan2(np.sin(z), np.cos(z))

    def process_scan(self, msg):
        """Uses LIDAR scan data to control the movement of the robot.

        The robot moves toward the center of mass of points within a circular
        region around the robot. This center of mass is considered to be the
        person.
        """
        # Stops movement when the node is killed.
        rospy.on_shutdown(self.stop_motors)

        # Each point in this list is of the form [radius, angle]
        valid_points = []

        for i, r in enumerate(msg.ranges[:-1]):
            if r != 0.0 and r < self.radius_max and r > self.radius_min:
                valid_points.append([r, self.angle_normalize_radians(np.deg2rad(i))])

        if len(valid_points) == 0:
            self.stop_motors()

        else:
            # "Person" detection.
            # TODO: Consider refining person detection further to detect valid
            # objects and movement.
            center_of_mass = np.mean(valid_points, axis=0)

            if np.abs(np.sum(center_of_mass)) > 0.1:
                # PI control is used for both the angle of the robot and the distance from the target.
                self.twist.linear.x = center_of_mass[0] *0.75 + self.integrated_error_linear * 0.02
                self.twist.angular.z = center_of_mass[1] * 0.75 + self.integrated_error_angular * 0.02
            else:
                # TODO: Consider making the robot wander if it cannot detect a person.
                self.stop_motors()

            self.integrated_error_linear += center_of_mass[0]
            self.integrated_error_angular += center_of_mass[1]
            
            # Convert to cartesian coordinates for visualization
            center_cart = self.pol_to_cart(center_of_mass[0], center_of_mass[1])
            self.mark.pose.position.x = center_cart[0]
            self.mark.pose.position.y = center_cart[1]

        self.vis_pub.publish(self.mark)
        self.twist_pub.publish(self.twist)

    def emergency_stop(self, msg):
        """Callback function to stop if the robot hits an object."""
        if any([msg.leftFront,
            msg.rightFront,
            msg.leftSide,
            msg.rightSide]):
            self.stop_motors()

    def stop_motors(self):
        """Stops all movement of the robot."""
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.twist_pub.publish(self.twist)

    def run(self):
        rospy.spin()

follow = PersonFollowNode()
follow.run()
