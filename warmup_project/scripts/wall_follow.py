#!/usr/bin/env python
from __future__ import print_function, division
import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Twist, Vector3, Pose, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool, ColorRGBA
import random
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from scipy import stats

class WallFollow(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        rospy.Subscriber('/projected_stable_scan', PointCloud, self.process_scan_ransac)
        rospy.Subscriber('/odom', Odometry, self.update_neato_pos)
        self.position = Pose()

    def publish_neato_pos(self):
        self.publish_marker(self.position.position.x, self.position.position.y)

    def publish_marker(self, pointx, pointy):
        # Build the marker and publish it
        marker = Marker()
        marker.type = 2
        marker.header.frame_id = "base_link"
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(100, 100, 100, 255)
        marker.pose = Pose()
        marker.pose.position = Point()
        marker.pose.position.x = pointx
        marker.pose.position.y = pointy
        self.marker_publisher.publish(marker)

    def publish_line(self, slope, intercept):
        # Build a line and publish it
        marker = Marker()
        marker.type = 4 # LINE_STRIP
        marker.header.frame_id = "odom"
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0, 255, 255, 255)
        point1 = Point()
        point2 = Point()
        point1.x = intercept
        point1.y = 0
        point2.x = 3
        point2.y = 3*slope
        marker.points += point1
        marker.points += point2
        self.marker_publisher.publish(marker)

    def update_neato_pos(self, msg):
        self.position = msg.pose.pose.position
        print(self.position)

    def process_scan_ransac(self, msg):
        self.approach_follow_wall(([val.x, val.y] for val in msg.points))

    def approach_follow_wall(self, ranges):
        # Find a wall, check how far we are from it, approach it if needed, else follow
        slope, intercept, r_value, p_value, std_err = self.ransac_ranges(ranges)
        self.publish_line(slope, intercept)
        self.publish_neato_pos()
        perpendicular_slope = -1./slope
        along_wall_incr = 0.1 # increment to travel along wall
        target_distance_from_wall = 0.5 # distance from wall we want to be
        error_thresh = 0.2 # deviation from target distance from wall allowed
        dist_from_line = self.dist_from_line(slope, intercept, (self.position.position.x, self.position.position.y))
        if (abs(dist_from_line - target_distance_from_wall) > error_thresh):
            self.go_to_point(self.position_along_slope(perpendicular_slope, (dist_from_line - target_distance_from_wall)))
        else:
            self.go_to_point(self.position_along_slope(slope, along_wall_incr))

    def position_along_slope(self, slope_to_follow, distance):
        # Find a new position "distance" closer along "slope_to_follow"
        if (distance > 0):
            new_x = np.sqrt(np.square(distance)/(np.square(slope_to_follow) + 1)) + self.position.position.x
            new_y = slope_to_follow * new_x + self.position.position.y
        else:
            new_x = self.position.position.x - np.sqrt(np.square(distance)/(np.square(slope_to_follow) + 1))
            new_y = self.position.position.y - slope_to_follow * new_x 
        return (new_x, new_y)

    def compare_position(self, target):
        # Compare current position to target position
        xdiff = self.position.poistion.x - target.x
        ydiff = self.position.position.y - target.y
        return xdiff, ydiff

    def dist_from_line(self, line_slope, line_intercept, point):
        # Return distance from given point to given line
        return (abs(point[0]*line_slope - point[1] + line_intercept)/np.sqrt(np.square(line_slope) + 1))

    def go_to_point(self, point):
        pass

    def ransac_ranges(self, ranges):
        iterations = 0
        max_iterations = 20
        n = 20 # minimum number of data points required to estimate model parameters
        d = 10 # number of close data points required to assert that a model fits well to data
        bestfit = None
        besterr = 100
        error_thresh = 0.5
        while (iterations < max_iterations):
            maybeinliers = random.sample(ranges, n)
            maybemodel = slope, intercept, r_value, p_value, std_err = stats.linregress((val[0] for val in maybeinliers), (val[1] for val in maybeinliers))
            alsoinliers = []
            for val in (ranges not in maybeinliers):
                if (abs(val[0]*slope + intercept - val[1]) < error_thresh ): #if point fits maybemodel with an error smaller than t
                    alsoinliers += val
            if (len(alsoinliers) > d):
                bettermodel = slope, intercept, r_value, p_value, std_err = stats.linregress((val[0] for val in (maybeinliers + alsoinliers)), (val[1] for val in (maybeinliers + alsoinliers))) # model parameters fitted to all points in maybeinliers and alsoinliers
                if (std_err < besterr):
                    bestfit = bettermodel
                    besterr = thiserr
            iterations += 1
        return bestfit

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    wall_follow = WallFollow()
    wall_follow.run()