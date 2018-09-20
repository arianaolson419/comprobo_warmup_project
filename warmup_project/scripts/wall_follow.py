#!/usr/bin/env python
from __future__ import print_function, division
import rospy
from sensor_msgs.msg import LaserScan
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
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
        rospy.Subscriber('/projected_stable_scan', PointCloud, self.process_scan_ransac)
        rospy.Subscriber('/odom', Odometry, self.update_neato_pos)
        self.position = Pose()

    def process_scan(self, msg):
        rospy.on_shutdown(self.stop_motors)
        target = (2, 2)
        self.go_to_point(target)
#        ranges = self.check_ranges(msg.ranges) # Find nonzero ranges, plus or minus up to five degrees
#        range0 = ranges[0]
#        range1 = ranges[1]
#        range2 = ranges[2]
#        range3 = ranges[3]
#        out_msg = Twist()
#        # Compare the sums of each set of ranges, with zeros counting as 20
#        if (self.sum_for_compare(range0[1], range1[1]) < self.sum_for_compare(range2[1], range3[1])):
#            # Find which set has both ranges closest, or both ranges at all, or one range that's closer
#            turn_coords = self.turn_from_ranges(range0, range1)
#            out_msg.angular.z = turn_coords[0]
#            out_msg.linear.x = turn_coords[1]
#        else:
#            turn_coords = self.turn_from_ranges(range2, range3)
#            out_msg.angular.z = turn_coords[0]
#            out_msg.linear.x = turn_coords[1]
#        self.publisher.publish(out_msg)

    def check_ranges(self, ranges):
        # Try to get a nonzero range value for each of the angles, with up to five degrees in either direction off
        angles = [45, 135, 225, 315]
        adjustments = [0, -1, 1, -2, 2, -3, 3, -4, 4, -5, 5]
        ranges_out = [[angles[0], 0.0], [angles[1], 0.0], [angles[2], 0.0], [angles[3], 0.0]]
        if (len(ranges) > 359): 
            for i in range(len(angles)):
                for adjustment in adjustments:
                    if (ranges[angles[i] + adjustment] != 0.0):
                        ranges_out[i] = [angles[i] + adjustment, ranges[angles[i] + adjustment]]
                        break
            return ranges_out
        else:
            return ranges_out

    def publish_marker(self, range_tuple0, range_tuple1):
        # Build the marker and publish it
        marker = Marker()
        marker.type = 2
        marker.header.frame_id = "base_link"
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(100, 100, 100, 255)
        marker.pose = Pose()
        marker.pose.position = Point()
        marker.pose.position.x = np.cos(range_tuple0[0])*range_tuple0[1]
        marker.pose.position.y = np.sin(range_tuple0[0])*range_tuple0[1]
        self.marker_publisher.publish(marker)

    def sum_for_compare(self, range1, range2):
        # If either range is zero, set that range to equal 20
        # So when we compare the sets of ranges it will select for the ones with the closest sets of nozero points
        if (range1 == 0.0):
            range1 = 20.0
        if (range2 == 0.0):
            range2 = 20.0
        return (range1 + range2)

    def update_neato_pos(self, msg):
        self.position = msg.pose.pose

    def turn_from_ranges(self, range_tuple0, range_tuple1):
        tolerance = 0.05 # Allowable difference in ranges
        kp = 1.5 # Proportional constant
        range0 = range_tuple0[1]
        range1 = range_tuple1[1]
        if ((range0 != 0.0) and (range1 != 0.0)):
            # We're getting ranges for both angles
            difference = range0 - range1
            proportion = (difference * 2)/(range0 + range1)
            # Making turn speed more regular
            if (difference > tolerance):
                # Adjust angle if we are not aligned
                return (proportion/kp, 0.0)
            elif (difference < -tolerance):
                # Adjust angle if we are not aligned
                return (proportion/kp, 0.0)
            else:
                # If we are aligned to wall, move forward and publish a marker to where the wall is
                self.publish_marker(range_tuple0, range_tuple1)
                return (0.0, 0.25)
        else:
            if (range0 != 0.0):
                # if we're only getting one range, adjust to turn the missing range closer
                return (-0.25, 0.0)
            elif (range1 != 0.0):
                # if we're only getting one range, adjust to turn the missing range closer
                return (0.25, 0.0)
            else:
                # if we're not getting any ranges, rotate slowly in a random direction. 
                return (0.25 * random.choice([-1, 1]), 0.0)

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
        self.approach_follow_wall(([val.x, val.y] for val in msg.points))

    def approach_follow_wall(self, ranges):
        # Find a wall, check how far we are from it, approach it if needed, else follow
        slope, intercept, r_value, p_value, std_err = self.ransac_ranges(ranges)
        perpendicular_slope = -1./slope
        des_distance_from_wall = 0.5 
        error_thresh = 0.2
        dist_from_line = self.dist_from_line(slope, intercept, (self.position.position.x, self.position.position.y))
        if (abs(dist_from_line - des_distance_from_wall) > error_thresh):
            self.go_to_point(self.position_along_slope(perpendicular_slope, (dist_from_line - des_distance_from_wall)))
        else:
            pass


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
        """Go to a target point. 
        point: a cartesian point of the form (x, y).
        """
        current_position = self.convert_pose_to_xy_and_theta(self.position)
        print(current_position)
        distance = np.linalg.norm([np.array(point) - np.array(current_position[:2])])
        point_rel = np.array(point) - np.array(current_position[:2])
        point_rel_polar = self.cartesian_to_polar(point_rel[0], point_rel[1])
        angle_error = self.angle_diff(current_position[2], point_rel_polar[1])
        print(angle_error)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.2 * angle_error
        self.publisher.publish(twist)

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
