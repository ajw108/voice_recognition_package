#!/usr/bin/env python

"""
appears to be exactly the same as voice_control, ignore
"""

import roslib; roslib.load_manifest('voice_recognition')
import rospy
import math

from std_msgs.msg import String
import sensor_msgs.msg

import baxter_interface

class voice_control:

    def __init__(self):

        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable()
        rs.disable()

        print("Enabling robot... ")
        rs.enable()

        self.limb = baxter_interface.Limb('left')
        self.joint = 0
        self.inc = .1

        rospy.Subscriber('recognizer/output', String, self.voiceMove)

        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()

        self.cleanup()

        
    def voiceMove(self, msg):

        left = baxter_interface.Limb('left')
        right = baxter_interface.Limb('right')

        if msg.data.find("left") > -1:
            self.limb = left
        elif msg.data.find("right") > -1:
            self.limb = right

        limb = self.limb
        jl = limb.joint_names()

        if msg.data.find("s zero") > -1:
            self.joint = 0
        elif msg.data.find("s one") > -1:
            self.joint = 1
        elif msg.data.find("e zero") > -1:
            self.joint = 2
        elif msg.data.find("e one") > -1:
            self.joint = 3
        elif msg.data.find("w zero") > -1:
            self.joint = 4
        elif msg.data.find("w one") > -1:
            self.joint = 5
        elif msg.data.find("w two") > -1:
            self.joint = 6

        joint = self.joint

        if msg.data.find("increase") > -1:
            current_pos = limb.joint_angle(jl[joint])
            joint_comm = {jl[joint]: current_pos + self.inc}
            limb.set_joint_positions(joint_comm)
        elif msg.data.find("decrease") > -1:
            current_pos = limb.joint_angle(jl[joint])
            joint_comm = {jl[joint]: current_pos - self.inc}
            limb.set_joint_positions(joint_comm)

        if msg.data.find("joint") > -1:
            print "\ns0 ",limb.joint_angle(jl[0])
            print "s1 ",limb.joint_angle(jl[1])
            print "e0 ",limb.joint_angle(jl[2])
            print "e1 ",limb.joint_angle(jl[3])
            print "w0 ",limb.joint_angle(jl[4])
            print "w1 ",limb.joint_angle(jl[5])
            print "w2 ",limb.joint_angle(jl[6])

    def cleanup(self):
        print "Disabling robot..."
        rs = baxter_interface.RobotEnable()
        rs.disable()

if __name__=="__main__":
    rospy.init_node('voice_control')
    try:
        voice_control()
    except:
        pass

