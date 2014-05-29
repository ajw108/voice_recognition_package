#!/usr/bin/env python

"""
an unimportant script used to test simple python concepts, ignore or use as desired
"""

import roslib; roslib.load_manifest('voice_recognition')
import rospy

a = [1,1]

if [1,1] == a:
	print 'yes'
else:
	print 'no'