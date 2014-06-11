# NxR-Baxter Voice Recognition
## Project Overview
The goal of this project was to provide an additional dimension of human-robot interaction to the Baxter (nicknamed RJ) robot (from Rethink Robotics) currently on display in the Technological Institute at Northwestern University. Previous projects have used a motion tracker to control the robot; this project used pocketsphinx speech recognition software developed by researchers at Carnegie Mellon University (CMU) as an additional means to communicate with the robot. The end result of this was a "memory"-style game that challenged the user to memorize and match various sequences of poses and words provided directly by the robot. This format created an interestingly difficult competition between robot and human; ultimately, this aspect is what accomplished the goal of new forms of interaction and emotion as facilitated by voice recognition.

For more information on Rethink Robotics and Baxter, check [this website.](http://www.rethinkrobotics.com/products/baxter/)

For more information on the Northwestern NxR lab, see [this website.](http://nxr.northwestern.edu/)

For more information on the CMU sphinx software, see [this website.](http://cmusphinx.sourceforge.net/)
### Game Guide
#### Starting the Game
A small amount of setup is required to start the memory game. First, if speech recognition is to be used, the sound settings on the computer must be tested and adjusted if necessary. The microphone to be used should be tested in its final location as if the game were being played. Ubuntu has basic sound options that allow input to be tested. Ensure that the microphone is registering a signal that spans most of the range of input volumes when speaking at an acceptable volume.

Next, the trajectory controller node must be started. Currently, this is not done through a .launch file and must be done independently of the memory game. The command is:

	rosrun baxter_tools joint_trajectory_action joint_trajectory_action_server.py
	
After this, the file memory.launch will start everything else required. The game will begin immediately after entering:

	roslaunch voice_recogntion memory.launch
	
If the memory node shuts down for any reason, the robot will safely move to the rest position before the node stops.
#### Playing the Game
There are two separate modes of the game: one with speech recognition and one without. The two modes play very similarly. In the game, RJ gives the user an action, then the user must repeat all previously given actions in order, ending with the most recent. Gameplay flows along this basic sequence:  

1. Element addition  
2. User element test  
3. RJ reset  
4. User sequence  
5. RJ evaluate  

An "element" is a single action that the user is required to perform. In the speech recognition game, this will be either a pose made by RJ that the user must mirror, or a word displayed on RJ's face screen that the user must speak. This step in the process is indicated by a single number displayed on RJ's screen, followed by a short pause, and then the presentation of the pose or word. The number indicates the position of the current action in the overall sequence of actions that the user must complete. If a pose is given, the number will be displayed continuously on the screen. IF a word is given, the number will be replaced by the word.

The user element test occurs directly after RJ presents the user with an action. The user must perform the current action immediately (outside of the full sequence). This test allows the user to make sure they understand exactly how to perform the action, helping to mitigate confusion. If a pose is given, the user must mirror that pose. The test will be complete once a green check mark appears on the screen. If a word is given, the user must speak that word. Once successfully spoken, a green check mark will appear. The user will then be prompted to say "continue", and after this the user element test is complete.

The RJ reset is the short transition phase after the user tests the new element. All that happens here is RJ moves back into the resting pose, which is both arms downward. This should not be confused with a new pose, however.

The user sequence will be initialized by RJ with the messages "Ready..." and "Start!". Once this happens, the user will then be required to repeat each element in order. The number of each element will be displayed on the screen at this time, and a successful match will result in a green checkmark displayed. The sequence ends when all previous elements have been successfully performed by the user or when the user fails to recreate the element before the timer runs out (timer is a few seconds).

Once the user sequence ends, RJ will evaluate the entire sequence and display a message accordingly. There are three separate cases that can occur. If the user succeeds in replicating the sequence, RJ will display "Good!", and the game will return to the element addition step. If the user fails at any point in the sequence, the sequence will terminate, a red 'X' will be displayed, and then the "Game Over" screen will show. The game then clears the sequence and resets itself, then returns to the element addition step. The final scenario is a special case that can only occur when speech recognition is enabled and after the sequence reaches a certain specified length. In this case, the game will switch to a harder difficulty, and instead of each element being either a pose or a word, the elements will be a pose, a word, or both a pose and a word. A short message will indicate that this switch has been made, and then the game will return to the element addition step.
#### Quitting the Game
Currently, the game will run continuously until manually quit on the terminal. There is no way for a user to end the entire process. However, if the user simply walks away, the game sequence will stop and the system will wait for input from the next user to walk by. The current user detection system is not very robust; the game will choose the first person it sees as the user. If this user does not give the game any input, no other user will be able to play until the first user leaves. This, along with the user quitting procedure, needs to be updated for integration with a full display.

## Technical Documentation
### File Structure
The voice recognition directory contains 5 subdirectories: images, launch, model, msg, and src. Launch, msg, and src are ROS standard directories, images contains all of the image files for the memory game, and model contains the language model files for the pocketsphinx recognizer.
### Source Code
Within the src directory are scripts for 5 ROS nodes. The comments within each script give a more thorough overview of the purpose of each node and the specifics of each function.

1. 	detector.py
	
	detector.py continuously reads the data from the skeleton tracker and the pocketsphinx recognizer. It analyzes the body position of the user or the list of spoken words and publishes any configurations that match those required by the memory game. Additionally, the detector node handles the tracking of the main user.  
	
	The matching of skeleton data to a pose for the memory game is done by projecting the spatial positions of the user's shoulder, elbow, and hand into the x-z plane. From there, the angle of the vectors between the user's 	shoulder and elbow (upper arm) and between the elbow and hand (forearm) are calculated. These vector angles are then compared to a dictionary of poses that contain bounds on these angles; and if the angles lie within the bounds for a specific pose, that pose is successfully matched.      
	
	For example, one pose condition is:
	
		3 : {'left' : {'upper_arm' : [-22.5, 22.5], 'forearm' : [-22.5, 22.5]}, 'right' : {'upper_arm' : [67.5, 112.5], 'forearm': [67.5, 112.5]}}
	
	All angles are determined with respect to horizontal formed by looking at the user from the robot's perspective. Thus, if the user's arms were straight horizontally (mimicking an axis), the user's left arm would represent the +x direction, the user's right arm would represent the -x direction, and +z and -z are standard up and down. The representation shown above means that the angles of the user's left upper arm and forearm are both within regions centered around 0 degrees, and the angles of the user's right upper arm and forearm are both within regions centered around 90 degrees. This would correspond to the user making an "L" shape with the right and left arms.
	
	All of the pose data, including the values of RJ's joint angles and the angular bounds for the poses, as well as the functions to compare angles and move between poses are all contained within the script poses.py. This script is non-executable and is imported into nodes to be able to call those functions. Poses.py also uses functions from the script vector_operations.py, which contains mathematical functions to determine angles from vectors.
	
	Motion between poses is currently done using the trajectory controller, which designs a trajectory for RJ by taking as input a series of points in RJ's joint-space as well as a time value in which to complete the trajectory. Poses.py contains a dictionary that has these joint-space values. For example: 
	
		1 : {'left' : [.8, -.8, -3, 0, 0, 0, 0], #Y
		'right' : [-.8, -.8, 3, 0, 0, 0, 0]},
	
	This structure represents RJ's left arm and right arm. These values were determined by physically moving RJ to the desired configuration and checking the various angles using software. This was necessary due to the need to avoid obstacles within the cell that contains RJ. This also makes trajectory generation more complex, as having straight arms will very likely hit the walls when moving between poses, and having bent arms makes the poses very ambiguous. This is done by including an additional parameter "bend" passed to the moveToPose function. If "bend" is true, the arms will move to a safe bent state before moving to another pose. This should always be used. In fact, the parameter is probably unnecessary and the program can be simplified to exclude it and always move to the bent state first. Also of note is the fact that each of these bent states is "hardcoded" based off of the particular pose. There is likely a much more robust way of handling this motion.
	
	User tracking, as stated above, is done simply by choosing the first user seen by the skeleton tracker. The skeleton tracker software handles the movement of users, so the only parameter needed to keep track of the current user is an assigned UserID and the user's starting position. From there, before accepting any input, the detector node checks to make sure the input is coming from the current UserID and that this user is not too far away from the starting position, as specified by a tolerance. If the user is too far away or the current UserID cannot be found, the program resets all boolenas and restarts the user-choosing process.	
	
		p1_x = self.user_starting_position.x
		p1_z = self.user_starting_position.z
		p2_x = skel.torso.transform.translation.x
		p2_z = skel.torso.transform.translation.z
		
		dx = p2_x - p1_x
		dz = p2_z - p1_z
		
		if not (math.fabs(dx) > 0.25 and math.fabs(dz) > 0.25):
	
	The above code handles the user checking. The values dx and dz represent the change in the user's x- and z- position as seen by the skeleton tracker. The y-direction would represent moving closer to the tracker. For a new user to be chosen, the user's position would need to change in both the x- and z- direction by more than the tolerance of 0.25.
	
	The speech recognition comparison is done in a single function, parseSpeech. It is simply comparing strings found from the speech recognition data to those found in words.py (a list of strings for the game), and returning any match between them. Further information on the speech recognition data can be found in the pocketsphinx documentation section.

2. memory.py
	
	memory.py is the executable that structures the game sequence described above, taking as input the output from the detector node. It uses images found in the img directory to handle the screen display, and functions from poses.py to move RJ to the required poses. Otherwise, it relies on the detector node to determine success or failure of the user's actions. All of the main functions of this node are explained in other sections. See the source code for more details.

3. voice_control.py
 	
	voice_control.py is another node that uses speech recognition, but it is not required for the memory game. It is a simple example node for controlling RJ's various joints using voice commands.

4. voice_pose.py
 	
	voice_pose.py is another node that uses voice commands to switch between poses. It requires the trajectory controller (discussed later) to be running. This node is not required for the memory game and is also out of date with the latest version of poses.py.

5. voicetest.py
	
	voicetest.py is the simplest example of controlling RJ with voice recognition; it simple uses voice commands to change the screen display. This is not required for the memory game.

### Launch Files
Within the launch directory are six xml files to be used with ROSlaunch to start the launch nodes described above. Their syntax is very simple.

1. pocketsphinx.launch
	
	pocketsphinx.launch starts the pocketsphinx recognizer node for speech recognition. This file includes two parameters: the file location of RJ.lm, and RJ.dic. These files are required for the recognizer node to understand the list of words required for the memory game. The structure of these files will be explained in a later section. This file can include an additional optional parameter to specify which microphone is needed, but if only one mic is in use the node will automatically determine the correct microphone.

2. detector.launch
 	
	detector.launch starts the detector node and includes the files pocketsphinx.launch and nu_skeletontracker.launch, which start the recognizer (speech recognition) node and the skeletontracker_nu (motion tracking) node, respectively.

3. memory.launch
	
	memory.launch starts the memory node and includes the file detector.launch. There is a single parameter in memory.launch, which is a boolean parameter "speech". If speech is specified as true, then the memory game will be run with voice recognition and motion tracking. If false, only motion tracking. Additionally, the general ROS parameter "respawn" was set to true, so the node will restart itself if it crashes for any reason.

4. voice_control.launch
	
	Starts the voice_control node and includes pocketsphinx.launch

5. voice_pose.launch
	
	Starts the voice_pose node and includes pocketsphinx.launch

6. voicetest.launch
	
	Starts the voicetest node and includes pocketsphinx.launch

### ROS Messages
The msg directory contains a single 3-line file memory.msg that holds the custom message used for the memory game.

	int32 poseID  
	string speech  
	bool hasUser
	
poseID is the integer number of the current pose matched by the user. The value is zero if there is no current pose. All pose numbers can be found in poses.py. speech is the string representing a match between spoken words and the list of game words from poses.py. hasUser is a boolean that is true if a main user has been chosen and false if not.

The detector node publishes this message at a 20hz frequency, and the memory node subscribes to it.

### Images
The images directory contains all images used for the nodes described above, as well as extraneous images used in earlier iterations of the project.

### Language Model Files
The model directory contains three files that are used for the creation and implementation of a language model that the pocketsphinx recognizer utilizes for speech recognition. The pocketsphinx software will be described in detail in a later section.

1. corpus.txt
	
	corpus.txt is a simple text file with a specific syntax. Each line contains one word or phrase that the pocketsphinx software will recognize. More information on the specifics of the syntax can be found [here](http://www.speech.cs.cmu.edu/tools/lmtool.html)

2. RJ.dic
	
	RJ.dic is a dictionary file produced by CMU's language model tools that contains all words listed in corpus.txt. Each word is broken down into its standard [phonemes](http://en.wikipedia.org/wiki/Phoneme).

3. RJ.lm
	
	RJ.lm is a language model file produced by CMU's language model tools. It contains the probabilistic model that determines the likelihood that a given word in corpus.txt was spoken.

### Pocketsphinx Documentation
Pocketsphinx is the speech recognition software developed at Carnegie Mellon University that can be used for robust speech-recogntion tasks. This project currently only utilizes it in a very limited form. Full documentation on the software and the theory behind it can be found [here](http://cmusphinx.sourceforge.net/wiki/).

For this project, the language model and dictionary file are automatically produced by the [Sphinx Knowledge Base Tool](http://www.speech.cs.cmu.edu/tools/lmtool-new.html). Simply upload the corpus.txt file and download the resulting .lm and .dic files.

This language model tool uses various open-source Sphinx tools that are all open-source and all available for download. These tools provide much more versatility and options that the web tool does not. It would be a good topic of research for future additions to RJ and the memory game.
## Project Takeaways
### Difficulties
The main difficulties of this project were found in the game design itself and meeting the constraints caused by the speech recognition, motion tracking, and the robot. Both the speech recognition and motion tracking peripherals run at fast enough rates to encourage real-time interaction between the robot and user, but RJ moves at a relatively slow pace, which does not allow for quick, reflexive movements. For an interactive game, this speed constraint poses somewhat of a problem. Additionally, both the motion tracking and voice recognition needed to be implemented in a way that showcases their capabilities and without making either feel like a trivial add-on. The iniital thought was to create a rock-paper-scissors game, where the user and the robot play against each other and the robot provides both feedback and the game interface. However, rock-paper-scissors is not much fun to play unless the game can be played quickly, and this makes it non-ideal for RJ. Additionally, the game could be completed entirely with voice recognition or motion tracking, which meant that one would likely end up being trivialized. Add on top of this the fact that RJ cannot physically perform the standard rock-paper-scissors symbols, and it is clear why this initial idea was dismissed.
  
The memory game, however, succeeds where rock-paper-scissors did not due to a different effect of adding both peripherals. This game could certainly (and was, in fact) completed solely with voice recognition or solely with motion tracking, but the blend of the two adds a new level of difficulty instead of overlapping to accomplish the same task. The game also inherently requires relatively long periods of motion inactivity for the robot, which is desirable, but it is during these periods that the user is performing all of their actions and the other peripherals are providing real-time feedback. This means that the game meets all performance constraints without feeling like any aspects are too limited.

From a technical perspective, the largest difficulty was finding a good way of moving between poses. Rethink Robotics includes a few different basic tools to move the arms. The position controller worked, but it was much too slow for any type of entertaining game. The trajectory controller fixed this speed problem, but it actually required some slight modification to work. There is a single line commented out in the joint_trajectory_action_server.py code that needed to be uncommented and the next line commented instead. This line handles the stopping conditions of the trajectory controller. Originally it set joint velocities to zero instead of holding constant position (which the line swap accomplished), and this for some reason disabled the robot and caused the arms to drop. I believe this was a bug, but if not found then the motion problem would have been much more difficult. However, both the trajectory controller and the position controller do not have any means of avoiding obstacles, which resulted in the workarounds described in previous sections.
### Next Steps
In its current state, the project cannot be fully realized due to the available hardware. Specifically, an adequate microphone setup is required to implement the voice recognition portion of the game. There are significant challenges associated with this, though. RJ is located in a public space that has regular traffic, although it is usually light foot traffic of people passing through and not heavy crowds. Due to possible echoing and other noises, it is possible that the accuracy of the speech recognition would drop significantly as compared to the current setup, which uses a headset microphone. However, the game currently is simple enough that this may not be an issue, as the only speech inputs are the numbers one through ten.
  
This brings up another potential improvement to the game and to the robot in general, which is more complex voice recognition. The pocketsphinx software has capabilities far beyond just single-word recognition, which could certainly create a much more impressive and interactive display. Even the simpler recognition could be more interesting than the numbers one through ten; for example, it would be possible to pull random relevant words or short phrases from the internet and use them to play the game. In general, the game could have much more depth than it currently does, with a greater variety of actions and words offered to the user. There is definitely potential for features such as pulling random words or phrases from the internet or other documents, generating a corresponding image automatically, and displaying it.

One more speech recognition issue is the fact that there is a single known bug that has yet to be resolved. This bug is documented in the source code, but it arises from the fact that the pocketsphinx software holds speech input in a buffer that is not cleared until new words are spoken. In the memory game, if the same word appears twice in a row, and nothing is spoken between these words, then the game will immediately count the second appearance as correct even if the user has not given any input. This only happens in very rare cases, as there is a check to make sure that two instances of the same word cannot appear directly in a row. If they appear indirectly in order, such as with a pose in between, then the error can occur. This can be solved with additional checks, but in the current software that requires some additional variables.

Also in need of improvement is the means by which the motion tracker registers and holds the main user. This is currently being explored by others working on the same robot, so hopefully there will be a robust solution in place that can be adapted to this application. The current solution of choosing the first user will not work very well in a crowd, or even in an area where people occasionally congregate. In general, this game was developed as an independent side project, and needs changes to fit into a larger display. A means of starting the game outside of the launch file is required as well as one of exiting the game without a manual quit.

The current motion planning system does work, but if the location or contents of the cell the robot is held in were changed in any way, it likely would not. The current system also makes the addition of new poses difficult, as both the position and the safe position need to be found and correctly implemented. Any solution that can plan trajectories around obstacles would be significantly better.

Finally, the last recommendation for immediate future work on this project is simply to thoroughly test it. For example, different users may not register the same way on the motion tracker, different accents may cause unintended results from speech recognition, or small bugs or unnaccounted-for edge cases may cause the game to fail. This may be best accomplished through taking statistics while part of a continuously-running display or by specifically monitoring a variety of users over multiple trials, but in either case this first revision likely has some issues that need to be resolved.
