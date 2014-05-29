#!/usr/bin/env python

"""
voicetest.py is a simple demo of speech recognition.
"""

import roslib; roslib.load_manifest('voice_recognition')
import rospy
import math

from std_msgs.msg import String
import sensor_msgs.msg

import cv
import cv_bridge

import baxter_interface

class voicetest:

    def __init__(self):
        rospy.Subscriber('recognizer/output', String, self.speech)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            r.sleep()
        
    def speech(self, msg):
        rospy.loginfo(msg.data)

        print msg.data

        if msg.data.find("rock") > -1:
            img = cv.LoadImage("/home/nxr-baxter/catkin_ws/src/voice_recognition/src/images/rock.jpeg")
            msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
            pub = rospy.Publisher('/sdk/xdisplay', sensor_msgs.msg.Image, latch=True)
            pub.publish(msg)
        if msg.data.find("paper") > -1:
            img = cv.LoadImage("/home/nxr-baxter/catkin_ws/src/voice_recognition/src/images/paper.jpeg")
            msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
            pub = rospy.Publisher('/sdk/xdisplay', sensor_msgs.msg.Image, latch=True)
            pub.publish(msg)
        if msg.data.find("scissors") > -1:    
            img = cv.LoadImage("/home/nxr-baxter/catkin_ws/src/voice_recognition/src/images/scissors.jpeg")
            msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
            pub = rospy.Publisher('/sdk/xdisplay', sensor_msgs.msg.Image, latch=True)
            pub.publish(msg)

    def cleanup(self):
        img = cv.LoadImage("/home/nxr-baxter/catkin_ws/src/nxr_baxter/images/nxr.jpg")
        msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
        pub = rospy.Publisher('/sdk/xdisplay', sensor_msgs.msg.Image, latch=True)
        pub.publish(msg)

if __name__=="__main__":
    rospy.init_node('voicetest')
    try:
        var = voicetest()
    except:
        pass

