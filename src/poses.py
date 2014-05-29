#!/usr/bin/env python

import roslib; roslib.load_manifest('voice_recognition')
import rospy
from std_msgs.msg import UInt16
import math
import numpy

import threading
import Queue

import baxter_interface
import baxter_dataflow
from vector_operations import (vector_from_points,
                               vector_projection_onto_plane)

from trajectory import Trajectory

class Poses(object):
	"""
	The poses class contains the information defining the poses that comprise the
	memory game. There are two dictionaries below: one consists of the Baxter joint
	angles, and the other consists of the bounds (in degrees) that correspond to 
	a user successfully executing that pose. Editing the bounds will change the 
	sensitivity of the detector node.

	The poses are defined by the angle of the user's upper arm and the angle of
	the user's forearm (i.e. between shoulder and elbow, and elbow and hand)
	"""

	def __init__(self):

		self.left_arm = baxter_interface.Limb('left')
		self.right_arm = baxter_interface.Limb('right')

		self.left_traj = Trajectory('left')
		self.right_traj = Trajectory('right')

		#Baxter left arm and Baxter right arm

		self.poses = { 
		1 : {'left' : [.8, -.8, -3, 0, 0, 0, 0], #Y
		'right' : [-.8, -.8, 3, 0, 0, 0, 0]},
		2 : {'left' : [.8, 0, -3, 1.5, 0, 0, 0], #biceps
		'right' : [-.8, 0, 3, 1.5, 0, 0, 0]},
		3 : {'left' : [.8, -1.3, -3, 0, 0, 0, 0], #L
		'right' : [-.8, 0, 1.6, .4, 0, .4, 0]},
		4 : {'left' : [.8, 0, -1.5, .4, 0, .4, 0], #backwards L
		'right' : [-.8, -1.3, 3, 0, 0, 0, 0]},
		5 : {'left' : [.8, -.8, -3, 0, 0, 0, 0], #/
		'right' : [-.8, .65, 0, 0, 0, 0, 0]},
		6 : {'left' : [.8, .65, 0, 0, 0, 0, 0], #\
		'right' : [-.8, -.8, 3, 0, 0, 0, 0]}, 
		7 : {'left' : [.8, -1.3, -3, 0, 0, 0, 0], #up
		'right' : [-.8, -1.3, 3, 0, 0, 0, 0]},
		8 : {'left' : [.8, 0, -1.5, .4, 0, .4, 0], #backwards r
		'right' : [-.8, .85, 0, .8, 0, 0, 0]},
		9 : {'left' : [.8, .85, 0, .8, 0, 0, 0], #r
		'right' : [-.8, 0, 1.6, .4, 0, .4, 0]},
		10 : {'left' : [.8, .65, 0, 0, 0, -.1, 0], #rocket
		'right' : [-.8, .65, 0, 0, 0, -.1, 0]},
		11 : {'left' : [.8, 0, -1.5, .4, 0, .4, 0], #straight
		'right' : [-.8, 0, 1.6, .4, 0, .4, 0]},
		12 : {'left' : [.8, .85, 0, .4, 0, .4, 0], #down
		'right' : [-.8, .85, 0, .4, 0, .4, 0]}}

		#human left arm and human right arm, mirrored from  baxter

		self.poseVectors = { # UNITS IN DEGREES
		#-----------------------------------------------------------------------Y------------------------------------------------
		1 : {'left' : {'upper_arm' : [22.5, 67.5], 'forearm' : [22.5, 67.5]},
		'right' : {'upper_arm' : [112.5, 157.5], 'forearm' : [112.5, 157.5]}},
		#-----------------------------------------------------------------------biceps----------------------------------------------
		2 : {'left' : {'upper_arm' : [-22.5, 22.5], 'forearm' : [67.5, 112.5]},
		'right' : {'upper_arm' : [157.5, 202.5], 'forearm' : [67.5, 112.5]}},
		#-----------------------------------------------------------------------L----------------------------------------------
		3 : {'left' : {'upper_arm' : [-22.5, 22.5], 'forearm' : [-22.5, 22.5]},
		'right' : {'upper_arm' : [67.5, 112.5], 'forearm' : [67.5, 112.5]}},
		#-----------------------------------------------------------------------backwards L----------------------------------------------
		4 : {'left' : {'upper_arm' : [67.5, 112.5], 'forearm' : [67.5, 112.5]},
		'right' : {'upper_arm' : [157.5, 202.5], 'forearm' : [157.5, 202.5]}},
		#-----------------------------------------------------------------------/----------------------------------------------
		5 : {'left' : {'upper_arm' : [292.5, 337.5], 'forearm' : [292.5, 337.5]},
		'right' : {'upper_arm' : [123.5, 157.5], 'forearm' : [112.5, 157.5]}},
		#-----------------------------------------------------------------------\----------------------------------------------
		6 : {'left' : {'upper_arm' : [22.5, 67.5], 'forearm' : [22.5, 67.5]},
		'right' : {'upper_arm' : [202.5, 247.5], 'forearm' : [202.5, 247.5]}},
		#-----------------------------------------------------------------------up------------------------------------------------
		7 : {'left' : {'upper_arm' : [67.5, 112.5], 'forearm' : [67.5, 112.5]},
		'right' : {'upper_arm' : [67.5, 112.5], 'forearm' : [67.5, 112.5]}},
		#-----------------------------------------------------------------------backwards r---------------------------------------------
		8 : {'left' : {'upper_arm' : [247.5, 292.5], 'forearm' : [247.5, 292.5]},
		'right' : {'upper_arm' : [157.5, 202.5], 'forearm' : [157.5, 202.5]}},
		#-----------------------------------------------------------------------r------------------------------------
		9 : {'left' : {'upper_arm' : [-22.5, 22.5], 'forearm' : [-22.5, 22.5]},
		'right' : {'upper_arm' : [247.5, 292.5], 'forearm' : [247.5, 292.5]}},
		#-----------------------------------------------------------------------rocket----------------------------------------------
		10 : {'left' : {'upper_arm' : [292.5, 337.5], 'forearm' : [292.5, 337.5]},
		'right' : {'upper_arm' : [202.5, 247.5], 'forearm' : [202.5, 247.5]}},
		#-----------------------------------------------------------------------straight----------------------------------------------
		11 : {'left' : {'upper_arm' : [-22.5, 22.5], 'forearm' : [-22.5, 22.5]},
		'right' : {'upper_arm' : [157.5, 202.5], 'forearm' : [157.5, 202.5]}},
		#-----------------------------------------------------------------------down----------------------------------------------
		12 : {'left' : {'upper_arm' : [247.5, 292.5], 'forearm' : [247.5, 292.5]},
		'right' : {'upper_arm' : [247.5, 292.5], 'forearm' : [247.5, 292.5]}}}

	def moveToPose(self, pose_num, bend):
		"""
		moveToPose takes as input the desired pose number (see dictionaries above)
		and a boolean bend, which determines whether or not the poses must bend before
		or after moving to avoid obstacles.

		This function uses the trajectory controller to move between poses as specified
		by the memory node.
		"""

		if pose_num == 2: # special case: the safe bend point of the 'biceps' pose is different
			leftP1 = self.poses[pose_num]['left'][0:4]+[0,.5,0]
			rightP1 = self.poses[pose_num]['right'][0:4]+[0,.5,0]
		else: # safe bend point of the other poses
			leftP1 = self.poses[pose_num]['left'][0:3]+[.5,0,.5,0]
			rightP1 = self.poses[pose_num]['right'][0:3]+[.5,0,.5,0]

		rightP2 = self.poses[pose_num]['right']
		leftP2 = self.poses[pose_num]['left']

		if pose_num < 8 or pose_num == 12:
			
			# poses 8, 9, 10, and 11 are safe to move to more quickly

			t1 = 3.7 # time to reach first point
			t2 = 5 # time to reach second point (from beginning)
		else:
			t1 = 2
			t2 = 3

		if bend:
			self.left_traj.add_point(leftP1,t1)
			self.left_traj.add_point(leftP2,t2)
			self.right_traj.add_point(rightP1,t1)
			self.right_traj.add_point(rightP2,t2)
		else:
			self.left_traj.add_point(leftP2,.005)
			self.left_traj.add_point(leftP1,1)
			self.right_traj.add_point(rightP2,.005)
			self.right_traj.add_point(rightP1,1)

		self.left_traj.start()
		self.right_traj.start()

		self.left_traj.wait(t1 + t2)
		self.right_traj.wait(t1 + t2)

		self.left_traj.clear('left')
		self.right_traj.clear('right')

	def makeAngles(self, left, right):
		"""
		makeAngles converts the data from the skeleton tracker
		to angles in degrees in the x-z plane as visible from
		the chair in baxter's cage.
		"""

		angles = []
		for arm in [left, right]:

			for i in [1, 2]:

				vec = vector_from_points(arm[i], arm[i-1])
				projVec = vector_projection_onto_plane(vec, [1,0,0], [0,1,0])
				angle = self.angleRefine(projVec)

				angles = angles + [angle]

		return angles

	def angleRefine(self, vec):
		"""
		angleRefine handles the cases outside the general
		arcsin, arccos range. It takes a vector projected
		into the x-z plane as input, either from the user's
		upper arm or the user's forearm. It returns the
		corrected angle value.
		"""

		mag = numpy.linalg.norm(vec)
		theta = numpy.arccos(vec[0]/mag)

		if vec[1]/mag <= 0:
			theta = 2*math.pi-theta

		return theta*180/math.pi

	def angleCompare(self, angles):
		"""
		angleCompare tests angle data to determine if
		the current user position is within the pose 
		boundaries, and returns the pose number.
		"""

		poseNum = False

		for i in range(1, 13):
			if self.inBounds(angles, i):
				poseNum = i

		return poseNum

	def inBounds(self, angles, poseNum):
		"""
		inBounds is the test to see if the angles
		fall within the specified pose bounds. It
		returns a boolean.
		"""

		bounded = True
		i = 0

		for limb in ['left', 'right']:
			for arm in ['upper_arm', 'forearm']:

				if angles[i] > 337.5:
					angles[i] = angles[i] - 360
				
				if not self.limTest(angles[i], self.poseVectors[poseNum][limb][arm]):
					bounded = False

				i = i + 1

		return bounded

	def limTest(self, val, lims):
		"""
		A simple test for bounding
		"""

		if lims[0] < val and val < lims[1]:
			return True
		else: 
			return False 