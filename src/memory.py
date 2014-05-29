#!/usr/bin/env python

import roslib; roslib.load_manifest('voice_recognition')
import rospy

from voice_recognition.msg import memory
from sensor_msgs.msg import Image

from poses import Poses
from words import Words

import cv
import cv_bridge

from random import randint

class Memory:
    """
    The memory node is the main controller for the memory game. The only subscriber is to the
    memory message from the detector node. Memory takes as input the current speech, pose, and user 
    and controls baxter accordingly to run the full game. There are two separate modes for the game, 
    one which contains speech recognition elements and one which does not. The modes are controlled 
    through the launch file parameter useSpeech, which is used throughout this node. Additionally,
    there is a hardLevel variable found in restart. Once the user reaches a sequence of the length
    found in this variable, the game switches between just words and poses to both at once. The game 
    structure works as follows:

    baxter displays a number in the sequence
    baxter either displays a word or makes a pose, or both
    baxter displays "you try!" and waits for the user to mimic the given action
    baxter displays the 'yes' image (green checkmark) and asks the user to say 'continue' if speech enabled
    baxter tells the user to start the sequence, and displays the sequence number to help the user
    if the user takes too long to get a single sequence element, then the game ends and restarts.

    The game is currently tailored to the specific 12 poses and 10 number words found in poses.py and
    words.py respectively. See the pocketsphinx documentation for information on how to change the 
    words list. Additional poses can also be added to poses.py, but currently each function is created
    to work with poses that lie only within the x-z plane, and significant changes would be needed to
    add poses outside of this range.

    Additionally, there is one known bug in the game caused by the speech buffer. If there is ever a 
    case where two instances of the same word appear with a pose between them, the buffer will likely
    not have reset between them and the second instance will be counted as correct. This problem is
    caused by the pocketsphinx node and not by the detector node.
    """

    def __init__(self):

        r = rospy.Rate(30)

        self.poses = Poses()
        self.words = Words().words

        self.restart()

        rospy.Subscriber('/detector/output', memory, self.readData)
        self.face = rospy.Publisher('/robot/xdisplay', Image, latch=True)

        rospy.on_shutdown(self.quit)

        try:
            self.useSpeech = rospy.get_param('~speech')
        except:
            self.useSpeech = False

        while not rospy.is_shutdown():
            self.elemLoop()

    def restart(self):
        """
        restarts the game and resets all variables.
        hardLevel is the sequence length to reach
        hard mode.s
        """

        self.hasUser = False
        self.poseFound = False
        self.hardMode = False
        self.hardLevel = 8

        self.pose = 0
        self.poseIn = [0,0,0,0,0,0,0,0]
        self.poseInd = 0
        self.speech = ''
        self.speechIn = ''
        self.run = []

    def quit(self):
        """
        quit runs whenever the node is shutdown, and moves baxter
        safely to the resting pose to avoid hitting any obstacles
        """

        self.poses.moveToPose(12,True)

    def faceChange(self, typ):
        """
        changes baxter's face to the image found in the specified directory.
        takes a string that is the filename as input.
        """

        imgOut = cv.LoadImage("/home/nxr-baxter/groovyws/src/voice_recognition/images/" + typ + ".jpeg")
        msg = cv_bridge.CvBridge().cv_to_imgmsg(imgOut)
        self.face.publish(msg)

    def readData(self, memory):
        """
        readData is the callback function for the memory message. This runs on a timer of 20 hz.
        For a pose to be recognized, the same pose must be held for 8 separate cycles which prevents
        pose detection by random motion. the poseIn variable (ring buffer) holds the last 8 values
        of the pose from the memory message.
        """

        self.poseIn[self.poseInd] = memory.poseID
        self.poseInd = (self.poseInd + 1)%8

        poseChanged = False

        for i in self.poseIn:
            if i != self.poseIn[self.poseInd]:
                poseChanged = True

        if not poseChanged:
            self.pose = self.poseIn[self.poseInd]
        else:
            self.pose = 0

        self.speech = memory.speech
        self.hasUser = memory.hasUser

    def elemLoop(self):
        """
        elemLoop is the continuous loop that controls the main gameplay as described above.
        An element in the sequence is added and tested, and then the entire sequence is looped
        through and each element is tested. The variable self.run holds each element of the
        sequence. This variable contains the type of the element ('word','pose', or 'both'),
        the poseID of the element (if type 'word', unused) and the wordID of the element
        (if type 'pose', unused)
        """

        self.addElem()
        self.elemFound = False

        self.faceChange(str(len(self.run))) # displays sequence number
        self.elemTest(self.run[-1]) # user test

        self.faceChange('ready')
        rospy.sleep(1)
        self.faceChange('start')
        rospy.sleep(1)

        count = 0

        for i in self.run:
            self.elemFound = False

            count = count + 1
            self.faceChange(str(count)) # sequence number

            time = 0

            elemTyp = i[0]
            poseID = i[1]
            wordID = i[2]

            if elemTyp == 'pose':
                elemRef = poseID # elemRef is the reference that the user needs to match
            elif elemTyp == 'word':
                elemRef = self.words[wordID]
            else:
                elemRef = [poseID,self.words[wordID]]

            while not self.elemFound and time < 5000000: # time is a couple seconds, arbitrary value
                elem = self.getData(elemTyp) # checks for user input
                time = time + 1

                if elem == elemRef:
                    self.elemFound = True
                    self.faceChange('yes') # successful match
                    rospy.sleep(1)
                    self.pose = 0

            if not self.elemFound:
                self.faceChange('no') # failure
                rospy.sleep(1)
                break

        if self.elemFound:
            self.faceChange('good') 
            rospy.sleep(1)
            if len(self.run) == self.hardLevel: # hard mode switch
                self.faceChange('easy')
                rospy.sleep(2)
                self.faceChange('hard')
                rospy.sleep(2)

        else:
            self.faceChange('done') # game over
            rospy.sleep(5)
            self.restart()

    def elemTest(self,elem):
        """
        elemTest controls the user test for a given element, 
        which is an array containing the element type, poseID, 
        and wordID. There is currently no time limit on this test.

        If a word is displayed, baxter requests the user to say 'continue'.
        This is to reset the buffer in the pocketsphinx node. Otherwise, if
        there were no speech input between elements, the game might consider 
        the test speech to be the correct answer for the game speech and the
        sequence would skip ahead without waiting for input.
        """

        elemTyp = elem[0]
        poseID = elem[1]
        wordID = elem[2]

        if elemTyp == 'pose':
            elemRef = poseID
            self.poses.moveToPose(poseID,True) # Baxter shows a pose
            self.faceChange('test')
            rospy.sleep(1)
        elif elemTyp == 'word':
            rospy.sleep(1)
            elemRef = self.words[wordID]
            self.faceChange(elemRef) # Baxter displays a word
            rospy.sleep(1)
        else:
            elemRef = [poseID,self.words[wordID]]
            self.poses.moveToPose(poseID,True)
            self.faceChange(self.words[wordID]) # baxter displays both
            rospy.sleep(1)

        while not self.elemFound and not rospy.is_shutdown():

            elem = self.getData(elemTyp)

            if elem == elemRef:

                self.elemFound = True
                self.faceChange('yes') # success

                if elemTyp == 'pose' or elemTyp == 'both':

                    # resets pose

                    self.poses.moveToPose(poseID,False)
                    rospy.sleep(.5)
                    self.poses.moveToPose(12,True)
                    self.pose = 0

                if elemTyp == 'word' or elemTyp == 'both':

                    # requests 'continue'

                    rospy.sleep(1)
                    self.faceChange('continue')
                    waiting = True

                    while waiting:

                        speech = self.getData('word')
                        if speech == 'continue':
                            self.faceChange('yes')
                            rospy.sleep(1)
                            waiting = False

    def addElem(self):
        """
        addElem randomly adds an element to the current sequence. An additional
        check is added so the exact same element is never performed twice in a row. 
        This is to make sure the game does not proceed rapidly and confuse users.
        """

        opt = ['pose','word','both']

        if self.useSpeech:
            if len(self.run) < self.hardLevel:
                typ = opt[randint(0,1)]
            elif len(self.run) == self.hardLevel:
                typ = 'both'
            else:
                typ = opt[randint(0,2)]
        else:
            typ = 'pose'

        numPoses = 11 # pose 12 is the chosen neutral pose
        numWords = len(self.words)-1

        elemChosen = False

        while not elemChosen and not rospy.is_shutdown():
            
            poseElem = randint(1,numPoses)
            wordElem = randint(0,numWords)

            if self.run == [] or typ != self.run[-1][0]: # if first element or different type than the previous element
                self.run = self.run + [[typ,poseElem,wordElem]]
                elemChosen = True

            elif typ == 'pose' and poseElem != self.run[-1][1]: # if a pose and not the same as the last pose
                self.run = self.run + [[typ,poseElem,wordElem]]
                elemChosen = True

            elif typ == 'word' and wordElem != self.run[-1][2]: # if a word "                           " word
                self.run = self.run + [[typ,poseElem,wordElem]]
                elemChosen = True

            elif typ == 'both' and poseElem != self.run[-1][1] and wordElem != self.run[-1][1]: # if different pose and word
                self.run = self.run + [[typ,poseElem,wordElem]]
                elemChosen = True

    def getData(self,typ):
        """
        getData is a simple function that returns the data from the
        memory callback. This is used to differentiate between data
        structures for poses, words, or both poses and words.
        """
        if typ == 'pose':
            return self.pose
        if typ == 'word':
            return self.speech
        if typ == 'both':
            return [self.pose,self.speech]

if __name__=='__main__':
    rospy.init_node('memory')
    Memory()