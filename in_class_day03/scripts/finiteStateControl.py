#!/usr/bin/env python
"""
Ben Ziemann
Last updated: 9/7/18

Example to show finite state control using forward, backwards, and turning
based on bump, laser, and timing conditions
"""

import rospy
import math
from std_msgs.msg import String
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist, Vector3


class FSC:

    def __init__(self):

        #States
        self.debugOn = False
        self.bumped = 0
        self.forwardDist = 99999
        self.turnTime = 0
        self.states = {"forward": self.forward, "backward": self.backward, "lturn": self.turnLeft}
        self.state = "forward"


        #Robot properities
        self.linVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.distExceed = 1

        #ROS
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.init_node('finite_state_control')
        rospy.Subscriber("/bump", Bump, self.checkBump)
        rospy.Subscriber("/scan", LaserScan, self.checkLaser)


    def checkLaser(self, msg):
        """
        Get the distance of the laser scan at 0deg (front of robot)
        """
        self.forwardDist = msg.ranges[0]


    def checkBump(self, msg):
        """
        Check if any bump sensor is pressed
        """
        if msg.leftFront or msg.rightFront or msg.leftSide or msg.rightSide:
            self.bumped = 1


    def publish(self, linX, angZ):
        """
        Publish a given velocity
        """
        self.linVector.x = linX
        self.angVector.z = angZ
        self.pub.publish(Twist(linear=self.linVector, angular=self.angVector))


    def forward(self):
        """
        if any bump sensor becomes activated, change state to moving backward.
        otherwise, stay in the moving forward state
        """
        if self.debugOn: print("Forwarding")
        self.publish(0.3, 0.0)
        while not self.bumped and not rospy.is_shutdown():
            print(self.forwardDist)
        self.publish(0.0, 0.0)
        self.state = "backward"



    def backward(self):
        """
        if the obstacle detected in front of the robot exceeds a specified threshold, change state to rotating left.  Also, record the time at which the robot began rotating left.
        otherwise, stay in the moving backward state.
        """
        if self.debugOn: print("Backing")
        self.publish(-0.3, 0.0)
        while (self.forwardDist < self.distExceed) and not rospy.is_shutdown():
            continue
        self.publish(0.0, 0.0)
        self.bumped = 0
        self.state = "lturn"
        self.turnTime = rospy.get_time()




    def turnLeft(self):
        """
        if the current time is more than 1 second greater than when the robot started rotating left, change state to moving forward.
        otherwise, stay in the rotating left state.
        """
        if self.debugOn: print("Turning")
        self.publish(0.0, 1.0)
        while (rospy.get_time() - self.turnTime < 1) and not rospy.is_shutdown():
            continue
        self.publish(0.0, 0.0)
        self.state = "forward"



    def run(self):
        """
        Have the robot navigate based on the example's rules for behavior
        """
        while(not rospy.is_shutdown()):
            self.states[self.state].__call__()
        

if __name__ == '__main__':
    fsc = FSC()
    try:
        fsc.run()
    except rospy.ROSInterruptException:
        fsc.publish(0.0, 0.0)