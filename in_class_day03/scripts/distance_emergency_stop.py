#!/usr/bin/env python
"""
Ben Ziemann
Last updated: 9/9/18

Stops the Neato based on laser scan directly in front of it
"""

import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist, Vector3


class EStop:

    def __init__(self):

        #States
        self.forwardDist = 999999

        #Robot properities
        self.linVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.slowingZone = 1.5
        self.stoppingZone = .6

        #ROS
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.init_node('eStop')

        rospy.Subscriber("/scan", LaserScan, self.checkLaser)


    def checkLaser(self, msg):
        """
        Get the distance of the laser scan at 0deg (front of robot)
        """
        self.forwardDist = msg.ranges[0]


    def publish(self, linX):
        """
        Publish a given velocity
        """
        self.linVector.x = linX
        self.pub.publish(Twist(linear=self.linVector, angular=self.angVector))


    def run(self):
        """
        Move forward and slow as the Neato approaches a wall
        """

        self.publish(.5)
        while self.forwardDist > self.slowingZone and not rospy.is_shutdown():
            continue
        while (self.forwardDist < self.slowingZone) and (self.forwardDist > self.stoppingZone ) and not rospy.is_shutdown():
            self.publish((self.forwardDist - self.stoppingZone)*.1)
            #self.publish(.33*self.forwardDist)
        self.publish(0.0)
       


if __name__ == '__main__':
    eStop = EStop()
    try:
        eStop.run()
    except rospy.ROSInterruptException:
        eStop.publish(0.0)