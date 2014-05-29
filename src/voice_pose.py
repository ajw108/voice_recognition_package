#!/usr/bin/env python

import roslib; roslib.load_manifest('voice_recognition')
import rospy
import math

from std_msgs.msg import String
import sensor_msgs.msg

import baxter_interface
from poses import Poses

#import subprocess

class VoicePose:
    """
    VoicePose is a node that switches between poses found in poses.py with 
    voice commands. This node is out of date and should not be used. Each
    moveToPose call needs to be updated to include the 'bend' boolean to move 
    safely around obstacles

    voice command structure:

    joint number ('one' through 'twelve', or the number of the pose in poses.py)
    'move' to move to that pose
    'cancel' to reset the pose number
    """

    def __init__(self):

        #subprocess.Popen(['rosrun', 'baxter_interface', 'trajectory_controller.py'])

        self.rs = baxter_interface.RobotEnable()
        self.rs.disable()

        self.poseSet = False

        self.rs.enable()

        self.poses = Poses()

        rospy.Subscriber('recognizer/output', String, self.speechControl)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            r.sleep()
        
    def speechControl(self, msg):

        print msg.data

        if not self.poseSet:
            self.getPose(msg)

        if self.poseSet and msg.data.find('move') > -1:
            self.poses.moveToPose(self.pose_num)
            self.poseSet = False

        if msg.data.find('cancel') > -1:
            self.poseSet = False

    def getPose(self, msg):

        if msg.data.find('one') > -1:
            self.pose_num = 1
            self.poseSet = True
        elif msg.data.find('two') > -1:
            self.pose_num = 2
            self.poseSet = True
        elif msg.data.find('three') > -1:
            self.pose_num = 3
            self.poseSet = True
        elif msg.data.find('four') > -1:
            self.pose_num = 4
            self.poseSet = True
        elif msg.data.find('five') > -1:
            self.pose_num = 5
            self.poseSet = True
        elif msg.data.find('six') > -1:
            self.pose_num = 6
            self.poseSet = True
        elif msg.data.find('seven') > -1:
            self.pose_num = 7
            self.poseSet = True
        elif msg.data.find('eight') > -1:
            self.pose_num = 8
            self.poseSet = True
        elif msg.data.find('nine') > -1:
            self.pose_num = 9
            self.poseSet = True
        elif msg.data.find('ten') > -1:
            self.pose_num = 10
            self.poseSet = True
        elif msg.data.find('eleven') > -1:
            self.pose_num = 11
            self.poseSet = True
        elif msg.data.find('twelve') > -1:
            self.pose_num = 12
            self.poseSet = True
            
if __name__=="__main__":
    rospy.init_node('voice_pose')
    try:
        var = VoicePose()
    except:
        pass

