#!/usr/bin/env python
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