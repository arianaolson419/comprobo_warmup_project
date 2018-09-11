#!/usr/bin/env python

import rospy
from neato_node.msg import Bump
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time

class DriveSquare(object):
	def __init__(self):
		super(DriveSquare, self).__init__()
		rospy.init_node('drive_square')
		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.rate = rospy.Rate(10)
		self.straight_time = 2
		self.turn_time = 3.1

	def pub_straight(self):
		outmsg = Twist()
		outmsg.linear.x = 0.5
		self.publisher.publish(outmsg)

	def pub_turn(self):
		outmsg = Twist()
		outmsg.angular.z = 0.5
		self.publisher.publish(outmsg)

	def straight_cycle(self):
		start_time = rospy.Time.now()
		end_time = start_time + rospy.Duration(self.straight_time)
		self.pub_straight()
		while (rospy.Time.now() < end_time):
			self.rate.sleep()

	def turn_cycle(self):
		start_time = rospy.Time.now()
		end_time = start_time + rospy.Duration(self.turn_time)
		self.pub_turn()
		while (rospy.Time.now() < end_time):
			self.rate.sleep()

	def run(self):
		while not rospy.is_shutdown():
			self.straight_cycle()
			self.turn_cycle()
			self.rate.sleep()


if __name__ == '__main__':
	motors = DriveSquare()
	motors.run()