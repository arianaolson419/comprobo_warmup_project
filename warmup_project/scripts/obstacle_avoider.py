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
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        rospy.Subscriber('/projected_stable_scan', PointCloud, self.process_scan)
        self.position = Pose()

    def process_scan(self, msg):
        print("!!!!!!!!!!!!!!!!!")

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
