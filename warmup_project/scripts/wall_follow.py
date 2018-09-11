#!usr/bin/env python
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
        rospy.Subscriber('/emergency_stop', Bool, self.emergency_stop)
        self.range_0 = None
        self.range_90 = None
    def process_scan(self, msg):
    	if (msg.ranges[0] != 0.0):
    		curr_range = msg.ranges[0]
    		if self.range_0 is None:
    			self.range_0 = curr_range
    		if ((self.range_0 - curr_range) > 0):
    			# positive change
    			pass
    		else:
    			# negative change
    			pass
    		self.range_0 = curr_range
    	if (msg.ranges[90] != 0.0):
    		curr_range = msg.ranges[90]
    		if self.range_90 is None:
    			self.range_90 = curr_range
    		if ((self.range_90 - curr_range) > 0):
    			# positive change
    			pass
    		else:
    			# negative change
    			pass
    		self.range_90 = curr_range
    			

