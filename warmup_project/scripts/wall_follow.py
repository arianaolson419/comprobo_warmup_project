#!/usr/bin/env python
from __future__ import print_function, division
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import random


class WallFollow(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
    def process_scan(self, msg):
        ranges = self.check_ranges(msg.ranges) # Find nonzero ranges, plus or minus up to five degrees
        range0 = ranges[0]
        range1 = ranges[1]
        range2 = ranges[2]
        range3 = ranges[3]
        out_msg = Twist()
        print("Sum of 0 and 1: %f" % self.sum_for_compare(range0, range1))
        print("Sum of 2 and 3: %f" % self.sum_for_compare(range2, range3))
        if (self.sum_for_compare(range0, range1) < self.sum_for_compare(range2, range3)):
            turn_coords = self.turn_from_ranges(range0, range1)
            out_msg.angular.z = turn_coords[0]
            out_msg.linear.x = turn_coords[1]
        else:
            turn_coords = self.turn_from_ranges(range2, range3)
            out_msg.angular.z = turn_coords[0]
            out_msg.linear.x = turn_coords[1]
        self.publisher.publish(out_msg)

    def check_ranges(self, ranges):
        # Try to get a nonzero range value for each of the angles, with up to five degrees in either direction off
        angles = [45, 135, 225, 315]
        #angles = [315, 225, 45, 135]
        adjustments = [0, -1, 1, -2, 2, -3, 3, -4, 4, -5, 5]
        ranges_out = [0.0, 0.0, 0.0, 0.0]
        if (len(ranges) > 359): 
            for i in range(len(angles)):
                for adjustment in adjustments:
                    if (ranges[angles[i] + adjustment] != 0.0):
                        ranges_out[i] = ranges[angles[i] + adjustment]
                        break
            return ranges_out
        else:
            return ranges_out

    def sum_for_compare(self, range1, range2):
        # If either range is zero, set that range to equal 20
        # So when we compare the sets of ranges it will select for the ones with the closest sets of nozero points
        if (range1 == 0.0):
            range1 = 20.0
        if (range2 == 0.0):
            range2 = 20.0
        return (range1 + range2)

    def turn_from_ranges(self, range0, range1):
        tolerance = 0.05 # Allowable difference in ranges
        kp = 1.5 # Proportional constant
        if ((range0 != 0.0) and (range1 != 0.0)):
            # We're getting ranges for both angles
            difference = range0 - range1
            proportion = (difference * 2)/(range0 + range1)
            # Making turn speed more regular
            print(difference)
            if (difference > tolerance):
                # Adjust angle if we are not aligned
                return (proportion/kp, 0.0)
            elif (difference < -tolerance):
                # Adjust angle if we are not aligned
                return (proportion/kp, 0.0)
            else:
                # If we are aligned to wall, move forward
                return (0.0, 0.25)
        else:
            if (range0 != 0.0):
                return (-0.25, 0.0)
            elif (range1 != 0.0):
                return (0.25, 0.0)
            else:
                return (0.25 * random.choice([-1, 1]), 0.0)
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    wall_follow = WallFollow()
    wall_follow.run()