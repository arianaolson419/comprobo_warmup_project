#!/usr/bin/env python
from __future__ import print_function, division
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Twist, Vector3, Pose, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool, ColorRGBA
import random
import numpy as np
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix

class ObstacleAvoider(object):
    def __init__(self):
        rospy.init_node('obstacle_avoider')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.update_neato_pos)
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        rospy.Subscriber('/projected_stable_scan', PointCloud, self.process_scan_projected)
        self.position = Pose()

    def process_scan_projected(self, msg):
        print("!!!!!!!!!!!!!!!!!")

    def process_scan(self, msg):
        (x_ranges, y_ranges) = self.add_range_vectors(self.clean_scan(msg.ranges))
        (r, theta) = self.cartesian_to_polar(x_ranges, y_ranges)
        msg_out = Twist()
        msg_out.linear.x = 0.1
        msg_out.angular.z = theta
        print("theta: %f" % theta)
        self.publish_marker(0.5, theta)
        print("r: %f" % r)
        self.publisher.publish(msg_out)

    def publish_marker(self, r, theta):
        # Build the marker and publish it
        marker = Marker()
        marker.type = 2
        marker.header.frame_id = "base_link"
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(100, 100, 100, 255)
        marker.pose = Pose()
        marker.pose.position = Point()
        (marker.pose.position.x, marker.pose.position.y) = self.polar_to_cartesian(theta, r)
        self.marker_publisher.publish(marker)


    def add_range_vectors(self, ranges):
        total_x = 0
        total_y = 0
        for range_val in ranges:
            (x, y) = self.polar_to_cartesian(range_val[0], 1/(np.power(range_val[1], 3)))
            total_x += x
            total_y += y
        return (total_x, total_y)

    def angle_normalize_radians(self, z):
        """Normalizes an angle in radians to between -pi and pi."""
        return np.arctan2(np.sin(z), np.cos(z))

    def clean_scan(self, raw_ranges):
        ranges_out = []
        for i in range(len(raw_ranges)):
            if (raw_ranges[i] != 0.0):
                ranges_out.append([i, raw_ranges[i]])
        return ranges_out

    def polar_to_cartesian(self, theta, r):
        x = np.cos(theta)*r
        y = np.sin(theta)*r
        return (x, y)

    def cartesian_to_polar(self, x, y):
        r = np.sqrt(np.square(x) + np.square(y))
        theta = np.arctan(y/x)
        return (r, theta)

    def update_neato_pos(self, msg):
        self.position = msg.pose.pose.position
        print(self.position)

    def calc_distances(self, points):
        """Calculate the Euclidean distance between each point and the Neato.
        Parameters
        ---------
        points: a list of Cartesion coordinates of the form [x, y, z].

        Returns
        -------
        a list of Euclidian distances of the same size as points.
        """
        current_pos = [self.position.x, self.position.y, self.position.z]
        distances = [np.linalg.norm([point - current_pos]) for point in points]
        return distances
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    obstacle_avoider = ObstacleAvoider()
    obstacle_avoider.run()
