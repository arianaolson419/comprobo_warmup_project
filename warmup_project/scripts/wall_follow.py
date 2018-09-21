#!/usr/bin/env python
from __future__ import print_function, division
import rospy
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Twist, Vector3, Pose, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool, ColorRGBA
import random
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from scipy import stats

class WallFollow(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        #self.publisher = rospy.Publisher('/wall_follow/cmd_vel', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.line_publisher = rospy.Publisher('/line_marker', Marker, queue_size=10)
        rospy.Subscriber('/projected_stable_scan', PointCloud, self.process_scan_ransac)
        rospy.Subscriber('/odom', Odometry, self.update_neato_pos)
        rospy.Subscriber('/bump', Bump, self.update_bump)
        self.position = Pose()
        self.bump = False

    def publish_neato_pos(self):
        self.publish_marker(self.position.position.x, self.position.position.y)

    def publish_marker(self, pointx, pointy):
        # Build the marker and publish it
        marker = Marker()
        marker.type = 2
        marker.header.frame_id = "odom"
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(255, 100, 100, 255)
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
        marker.color = ColorRGBA(100, 255, 255, 255)
        point1 = Point()
        point2 = Point()
        point1.x = 0
        point1.y = intercept
        point2.x = 3
        point2.y = 3*slope
        marker.points.append(point1)
        marker.points.append(point2)
        self.line_publisher.publish(marker)

    def update_bump(self, msg):
        if any([msg.leftFront,
            msg.rightFront,
            msg.leftSide,
            msg.rightSide]):
            self.bump = True
        else:
            self.bump = False

    def update_neato_pos(self, msg):
        self.position = msg.pose.pose

    def cartesian_to_polar(self, x, y):
        r = np.sqrt(np.square(x) + np.square(y))
        theta = np.arctan(y/x)
        return (r, theta)

    def convert_pose_to_xy_and_theta(self, pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return (pose.position.x, pose.position.y, angles[2])

    def angle_normalize(self, z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return math.atan2(math.sin(z), math.cos(z))

    def angle_diff(self, a, b):
        """ Calculates the difference between angle a and angle b (both should be in radians)
            the difference is always based on the closest rotation from angle a to angle b
            examples:
                angle_diff(.1,.2) -> -.1
                angle_diff(.1, 2*math.pi - .1) -> .2
                angle_diff(.1, .2+2*math.pi) -> -.1
        """
        a = self.angle_normalize(a)
        b = self.angle_normalize(b)
        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2

    def process_scan_ransac(self, msg):
        rospy.on_shutdown(self.stop_motors)
        self.approach_follow_wall([[val.x, val.y] for val in msg.points])

    def approach_follow_wall(self, ranges):
        if not self.bump:
            # Find a wall, check how far we are from it, approach it if needed, else follow
            slope, intercept, r_value, p_value, std_err = self.ransac_ranges(ranges)
            self.publish_neato_pos()
            if slope is not 0:
                perpendicular_slope = -1./slope
                along_wall_incr = 0.1 # increment to travel along wall
                target_distance_from_wall = 0.5 # distance from wall we want to be
                error_thresh = 0.2 # deviation from target distance from wall allowed
                dist_from_line = self.dist_from_line(slope, intercept, (self.position.position.x, self.position.position.y))
                self.publish_line(slope, intercept)
                if (abs(dist_from_line - target_distance_from_wall) > error_thresh):
                    self.go_to_point(self.position_along_slope(perpendicular_slope, (dist_from_line - target_distance_from_wall)))
                else:
                    self.go_to_point(self.position_along_slope(slope, along_wall_incr))
            else:
                print("received bad data.")
        else:
            twist = Twist()
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.publisher.publish(twist)

    def position_along_slope(self, slope_to_follow, distance):
        # Find a new position "distance" closer along "slope_to_follow"
        if (distance > 0):
            new_x = np.sqrt(np.square(distance)/(np.square(slope_to_follow) + 1)) + self.position.position.x
            new_y = slope_to_follow * new_x + self.position.position.y
        else:
            new_x = self.position.position.x - np.sqrt(np.square(distance)/(np.square(slope_to_follow) + 1))
            new_y = self.position.position.y - slope_to_follow * new_x 
        return (new_x, new_y)

    def dist_from_line(self, line_slope, line_intercept, point):
        # Return distance from given point to given line
        return (abs(point[0]*line_slope - point[1] + line_intercept)/np.sqrt(np.square(line_slope) + 1))

    def go_to_point(self, point):
        """Go to a target point. 
        point: a cartesian point of the form (x, y) in the odometry frame.
        """
        current_position = self.convert_pose_to_xy_and_theta(self.position)

        # The target point relative to the robot's coordinate system.
        point_rel = np.array(point) - np.array(current_position[:2])
        point_rel_polar = self.cartesian_to_polar(point_rel[0], point_rel[1])

        # The distance from the target point (linear error).
        distance = np.linalg.norm([np.array(point) - np.array(current_position[:2])])
        # The angle between the heading of the robot and the target point (angular error).
        angle = -self.angle_diff(current_position[2], point_rel_polar[1])
        print(angle)

        angle_tolerance = 0.9

        self.publish_marker(point[0], point[1])
        twist = Twist()
        if np.abs(angle) < angle_tolerance:
            twist.linear.x = 0.2 * distance
        else:
            twist.linear.x = 0.0
        twist.angular.z = 0.5 * angle
        self.publisher.publish(twist)

    def ransac_ranges(self, ranges):
        # Implementation of the RANSAC algorithm adopted from wikipedia pseudocode: https://en.wikipedia.org/wiki/Random_sample_consensus
        iterations = 0
        max_iterations = 30 # number of iterations we want to run
        n = 30
        d = 20
        bestfit = (0, 0, 0, 0, 0)
        besterr = 100
        error_thresh = 0.2  # meters away from the line a point can be before it's too far to fit the model
        while (iterations < max_iterations):
            if (len(ranges) > n):
                maybeinliers = random.sample(ranges, n)
                maybemodel = slope, intercept, r_value, p_value, std_err = stats.linregress([val[0] for val in maybeinliers], [val[1] for val in maybeinliers])
                alsoinliers = []
                for val in ranges:
                    if (val not in maybeinliers):
                        if (abs(val[0]*slope + intercept - val[1]) < error_thresh ): #if point fits maybemodel with an error smaller than t
                            alsoinliers += [val]
                if (len(alsoinliers) > d):
                    bettermodel = slope, intercept, r_value, p_value, std_err = stats.linregress([val[0] for val in (maybeinliers + alsoinliers)], [val[1] for val in (maybeinliers + alsoinliers)]) # model parameters fitted to all points in maybeinliers and alsoinliers
                    if (std_err < besterr):
                        bestfit = bettermodel
                        besterr = std_err
                iterations += 1
            else:
                break
        return bestfit

    def stop_motors(self):
        """Stops all movement of the robot."""
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.publisher.publish(twist)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    wall_follow = WallFollow()
    wall_follow.run()
