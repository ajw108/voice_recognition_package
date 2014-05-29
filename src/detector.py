#!/usr/bin/env python

################
# ROS IMPORTS: #
################
import roslib; roslib.load_manifest('voice_recognition')
import rospy
from std_msgs.msg import String

####################
# RETHINK IMPORTS: #
####################
import baxter_interface

###############
# NU IMPORTS: #
###############
from poses import Poses
from words import Words
from skeletonmsgs_nu.msg import Skeletons
from skeletonmsgs_nu.msg import Skeleton
from voice_recognition.msg import memory

####################
# OTHER IMPORTS: #
####################
import math

class Detector:

    """
    The class Detector creates an object that subscribes to both the skeleton message from 
    the NU skeleton tracker node and the speech recognition message from the pocketsphinx
    node, and 'parses' this data into the required information for the memory node. It acts 
    as a type of filter, condensing the skeleton data into just the predetermined poses 
    and the speech data into just the predefined words from the language model.

    The output is a published custom message of type memory, which contains an int32 number 
    describing the current pose (defined in poses.py), a string containing the current word,
    and a boolean variable describing whether or not a current user is chosen.
    """

    def __init__(self):

        self.poses = Poses()
        self.words = Words().words
        self.resetBool()

        rospy.Subscriber('recognizer/output', String, self.parseSpeech)
        rospy.Subscriber("skeletons", Skeletons, self.skeletonCallback)
        rospy.Timer(rospy.Duration(.05), self.output)
        
        self.pub = rospy.Publisher('/detector/output', memory)

        rs = baxter_interface.RobotEnable()
        rs.disable()
        rs.enable()

    def resetBool(self):
        """
        Resets booleans when user is done with action
        """
        print "\n**Booleans reset**\n"
        self.userid_chosen = False
        self.poseID = 0
        self.speech = ''
        self.skel_count = 0

    #=========================================================#
    #                        ACTIONS:                         #
    #=========================================================#

    def chooseUser(self, skeletons):
        """
        Selects primary user to avoid abgiguity. The primary user
        is currently selected as the first seen by the tracker.
        """
        for skeleton in skeletons:
            if skeleton.userid:

                self.main_userid = skeleton.userid
                self.userid_chosen = True
                print "\n\nMain user chosen.\nUser %s, please proceed.\n" % str(self.main_userid)
                self.user_starting_position = skeleton.torso.transform.translation
                rospy.sleep(2)

    def poseDetect(self, skeleton):
        """
        takes the skeleton data as input and returns the current pose, if any.
        See poses.py for more detailed information on the comparison.
        """

        self.skel_count+=1

        l_sh = skeleton.left_shoulder.transform.translation
        l_el = skeleton.left_elbow.transform.translation
        l_ha = skeleton.left_hand.transform.translation
        r_sh = skeleton.right_shoulder.transform.translation
        r_el = skeleton.right_elbow.transform.translation
        r_ha = skeleton.right_hand.transform.translation

        self.angles = self.poses.makeAngles([l_sh, l_el, l_ha], [r_sh, r_el, r_ha])

        if self.skel_count % 10 == 0: #use % 100 for pose testing, % 10 otherwise
            self.poseID = self.poses.angleCompare(self.angles)

            # if self.poseID:
            #    self.poses.moveToPose(self.poseID) #pose testing, uses the detecor
            ## node to swap between poses given by the user

    #######################
    # SUBSCRIBER CALLBACK #
    #######################
    
    def output(self, time):
        """
        output publishes to the custom memory message. The publisher
        runs off of a timer at 20 hz.
        """

        self.pub.publish(self.poseID, self.speech, self.userid_chosen)

    def skeletonCallback(self, data):
        """
        the callback for the skeleton subscriber. This function either
        passes along the current user's skeleton data from the message
        or finds a new main user if the first has left or has not been
        found.
        """

        if self.userid_chosen == True:
            found = False
            for skeleton in data.skeletons:
                if skeleton.userid == self.main_userid:
                    skel = skeleton
                    found = True

        if self.userid_chosen == False:
            self.choice = self.chooseUser(data.skeletons)

        elif found:
            p1_x = self.user_starting_position.x
            p1_z = self.user_starting_position.z
            p2_x = skel.torso.transform.translation.x
            p2_z = skel.torso.transform.translation.z

            dx = p2_x - p1_x
            dz = p2_z - p1_z

            if not (math.fabs(dx) > 0.25 and math.fabs(dz) > 0.25):
                self.poseDetect(skel)
            else: self.resetBool()

        elif not found:           
            self.resetBool()

    def parseSpeech(self, msg):
        """
        parseSpeech is the callback function for the pocketsphinx subscriber.
        it compares the current speech data to the list found in words.py.

        there is a known bug in this function, which has to do with the fact
        that occasionally the pocketsphinx node will pick up two words in place
        of one. If, for example, someone pronounces 'eight' as 'eigh-tuh' the
        speech recognizer will likely return 'eight two'. The loop break in the
        below code accounts for specifically this case as eight is found first,
        but other cases may arise where this creates an error.
        """

        speech = msg.data

        for i in self.words:
            if msg.data.find(i) > -1:
                self.speech = i
                break
                
        if msg.data.find('continue') > -1:
            self.speech = 'continue'
    
if __name__=='__main__':
    rospy.init_node('detector')
    Detector()

    while not rospy.is_shutdown():
        rospy.spin()



