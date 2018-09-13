#!/usr/bin/env python
from __future__ import print_function, division
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class WallFollow(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
    def process_scan(self, msg):
        range0 = msg.ranges[45]
        range1 = msg.ranges[135]
        range2 = msg.ranges[225]
        range3 = msg.ranges[315]
        out_msg = Twist()
        if ((range0 + range1) < (range2 + range3)):
            turn_coords = self.turn_from_ranges(range0, range1)
            if turn_coords is not None:
                print("0 and 1")
                out_msg.angular.z = turn_coords[0]
                out_msg.linear.x = turn_coords[1]
            else:
                turn_coords = self.turn_from_ranges(range3, range2)
                if turn_coords is not None:
                    print("3 and 2")
                    out_msg.angular.z = turn_coords[0]
                    out_msg.linear.x = turn_coords[1]
                else:
                    out_msg.angular.z = 0.25
        else:
            turn_coords = self.turn_from_ranges(range3, range2)
            if turn_coords is not None:
                print("3 and 2")
                out_msg.angular.z = turn_coords[0]
                out_msg.linear.x = turn_coords[1]
            else:
                turn_coords = self.turn_from_ranges(range0, range1)
                if turn_coords is not None:
                    print("0 and 1")
                    out_msg.angular.z = turn_coords[0]
                    out_msg.linear.x = turn_coords[1]
                else:
                    out_msg.angular.z = 0.25
        self.publisher.publish(out_msg)
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
            return None
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    wall_follow = WallFollow()
    wall_follow.run()