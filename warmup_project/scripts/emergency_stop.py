#!/usr/bin/env python
from __future__ import print_function, division

from neato_node.msg import Bump
from std_msgs.msg import Bool
import rospy

class EmergencyStopNode(object):
    def __init__(self):
        rospy.init_node('emergency_stop')
        self.sub = rospy.Subscriber('/bump', Bump, self.control_motors)
        self.pub = rospy.Publisher('/emergency_stop', Bool, queue_size=10)

        self.speed = 0.5

        self.bump = Bool()
        print(self.bump)

    def control_motors(self, b):
        if b.leftFront or b.rightFront or b.leftFront or b.rightFront:
            self.bump.data = True
        else:
            self.bump.data = False
        self.pub.publish(self.bump)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    stop = EmergencyStopNode()
    stop.run()
