#!/usr/bin/env python
<<<<<<< HEAD
"""
Ben Ziemann
Last updated: 9/9/18

Stops the Neato based on the bump sensor
"""

import rospy
import math
from std_msgs.msg import String
from neato_node.msg import Bump
from geometry_msgs.msg import Pose, Twist, Vector3


class EStop:

    def __init__(self):

        #States
        self.bumped = 0

        #Robot properities
        self.linVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angVector = Vector3(x=0.0, y=0.0, z=0.0)

        #ROS
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.init_node('eStop')

        rospy.Subscriber("/bump", Bump, self.checkBump)


    def checkBump(self, msg):
        """
        Check if any bump sensor is pressed
        """
        if msg.leftFront or msg.rightFront or msg.leftSide or msg.rightSide:
            self.bumped = 1


    def publish(self, linX):
        """
        Publish a given velocity
        """
        self.linVector.x = linX
        self.pub.publish(Twist(linear=self.linVector, angular=self.angVector))


    def run(self):
        """
        Move forward until the robot's bumped sensor is triggered
        """
        self.publish(1.0)
        while not self.bumped and not rospy.is_shutdown()::
            continue
        self.publish(0.0)


if __name__ == '__main__':
    eStop = EStop()
    try:
        eStop.run()
    except rospy.ROSInterruptException:
        eStop.publish(0.0)
=======

"""
Created on 29 July 2012
@author: Lisa Simpson
"""

from __future__ import print_function, division
import rospy
from neato_node.msg import Bump
from geometry_msgs.msg import Twist, Vector3

class EmergencyStopNode(object):
    def __init__(self):
        rospy.init_node('emergency_stop')
        rospy.Subscriber('/bump', Bump, self.process_bump)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.desired_velocity = 0.3

    def process_bump(self, msg):
        if any((msg.leftFront, msg.leftSide, msg.rightFront, msg.rightSide)):
            self.desired_velocity = 0.0

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.desired_velocity)))
            r.sleep()

if __name__ == '__main__':
    estop = EmergencyStopNode()
    estop.run()
>>>>>>> 2046111ffbf66c217ec99bc57444c898be0c5d8a
