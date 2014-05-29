#!/usr/bin/env python

import roslib; roslib.load_manifest('voice_recognition')
import rospy

class Words(object):
	"""
	Words is the list that will be used in the detector for the memory game.
	See the language model overview for methods on updating this list.
	"""

	def __init__(self):

		self.words = [
		'one',
		'two',
		'three',
		'four',
		'five',
		'six',
		'seven',
		'eight',
		'nine',
		'ten'];