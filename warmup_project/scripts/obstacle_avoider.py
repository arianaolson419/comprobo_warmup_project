#!/usr/bin/env python
from __future__ import print_function, division
import rospy
from sensor_msgs.msg import LaserScan
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
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)

    def process_scan(self, msg):
        pass

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    obstacle_avoider = ObstacleAvoider()
    obstacle_avoider.run()