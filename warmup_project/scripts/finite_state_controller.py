#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Twist, Vector3, Pose, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool, ColorRGBA
import random
import numpy as np
from nav_msgs.msg import Odometry

# States
E_STOP = 0
WALL_FOLLOW = 1
OBST_AVOID = 2


class FiniteStateController(object):
	def __init__(self):
		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/obst/cmd_vel', Twist, self.obst_cmd_vel_cb)
		rospy.Subscriber('/wall_follow/cmd_vel', Twist, self.wall_follow_cmd_vel_cb)
		rospy.Subscriber('/emergency_stop', Bool, self.e_stop_cb)
		self.obst_cmd_vel = Twist()
		self.wall_follow_cmd_vel = Twist()
		self.e_stop = False

	def obst_cmd_vel_cb(self, msg):
		self.obst_cmd_vel = msg
		self.choose_state()

	def wall_follow_cmd_vel_cb(self, msg):
		self.wall_follow_cmd_vel = Twist()
		self.choose_state()

	def e_stop_cb(self, msg):
		if (msg.data == True):
			self.publish_state(E_STOP)
			self.e_stop = True
		else:
			self.e_stop = False

	def publish_state(self, state):
		if (state == E_STOP):
			self.publisher.publish(Twist())
		elif (state == WALL_FOLLOW):
			self.publisher.publish(self.wall_follow_cmd_vel)
		elif (state == OBST_AVOID):
			self.publisher.publish(self.obst_cmd_vel)

	def choose_state(self):
		# Method of choosing what state to run in, probably based on whether the obstacle avoider is actively avoiding an obstacle or not
		if not self.e_stop:
			self.publish_state(E_STOP)
			if (self.obst_cmd_vel.linear.x > 0):
				self.publish_state(OBST_AVOID)
			else:
				self.publish_state(WALL_FOLLOW)




