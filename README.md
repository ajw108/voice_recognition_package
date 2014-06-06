# NxR-Baxter Voice Recognition
## Project Overview
The goal of this project was to provide an additional dimension of human-robot interaction to the Baxter (nicknamed RJ) robot (from Rethink Robotics) currently on display in the Technological Institute at Northwestern University. Previous projects have used a motion tracker to control the robot; this project used CMU's pocketsphinx speech recognition software as an additional means to communicate with the robot. The end result of this was a "memory"-style game that challenged the user to memorize and match various sequences of poses and words provided directly by the robot. This format created an interestingly difficult competition between robot and human; ultimately, this aspect is what accomplished the goal of new forms of interaction and emotion as facilitated by voice recognition.
### Game Guide
This section explains how to physically play the game once it is running. There are two separate modes: one with speech recognition and one without. The two modes play very similarly. In the game, RJ gives the user an action, then the user must repeat all previously given actions in order, ending with the most recent. Gameplay flows along this basic sequence:  

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
### Lessons Learned
The main difficulties of this project were found in the game design itself and meeting the constraints caused by the speech recognition, motion tracking, and the robot. Both the speech recognition and motion tracking peripherals run at fast enough rates to encourage real-time interaction between the robot and user, but RJ moves at a relatively slow pace, which does not allow for quick, reflexive movements. For an interactive game, this speed constraint poses somewhat of a problem. Additionally, both the motion tracking and voice recognition needed to be implemented in a way that showcases their capabilities and without making either feel like a trivial add-on. The iniital thought was to create a rock-paper-scissors game, where the user and the robot play against each other and the robot provides both feedback and the game interface. However, rock-paper-scissors is not much fun to play unless the game can be played quickly, and this makes it non-ideal for RJ. Additionally, the game could be completed entirely with voice recognition or motion tracking, which meant that one would likely end up being trivialized. Add on top of this the fact that RJ cannot physically perform the standard rock-paper-scissors symbols, and it is clear why this initial idea was dismissed.
  
The memory game, however, succeeds where rock-paper-scissors did not due to a different effect of adding both peripherals. This game could certainly (and was, in fact) completed solely with voice recognition or solely with motion tracking, but the blend of the two adds a new level of difficulty instead of overlapping to accomplish the same task. The game also inherently requires relatively long periods of motion inactivity for the robot, which is desirable, but it is during these periods that the user is performing all of their actions and the other peripherals are providing real-time feedback. This means that the game meets all performance constraints without feeling like any aspects are too limited.
### Next Steps
In its current state, the project cannot be fully realized due to the available hardware. Specifically, an adequate microphone setup is required to implement the voice recognition portion of the game. There are significant challenges associated with this, though. RJ is located in a public space that has regular traffic, although it is usually light foot traffic of people passing through and not heavy crowds. Due to possible echoing and other noises, it is possible that the accuracy of the speech recognition would drop significantly as compared to the current setup, which uses a headset microphone. However, the game currently is simple enough that this may not be an issue, as the only speech inputs are the numbers one through ten.
  
This brings up another potential improvement to the game and to the robot in general, which is more complex voice recognition. The pocketsphinx software has capabilities far beyond just single-word recognition, which could certainly create a much more impressive and interactive display. Even the simpler recognition could be more interesting than the numbers one through ten; for example, it would be possible to pull random relevant words or short phrases from the internet and use them to play the game. In general, the game could have much more depth than it currently does, with a greater variety of actions and words offered to the user.

Also in need of improvement is the means by which the motion tracker registers and holds the main user. This is currently being explored by others working on the same robot, so hopefully there will be a robust solution in place that can be adapted to this application.

Finally, the last recommendation for immediate future work on this project is simply to thoroughly test it. For example, different users may not register the same way on the motion tracker, different accents may cause unintended results from speech recognition, or small bugs or unnaccounted-for edge cases may cause the game to fail. This may be best accomplished through taking statistics while part of a continuously-running display or by specifically monitoring a variety of users over multiple trials, but in either case this first revision likely has some issues that need to be resolved.

## Technical Documentation
### File Structure
The voice recognition directory contains 5 subdirectories: images, launch, model, msg, and src. Launch, msg, and src are ROS standard directories, images contains all of the image files for the memory game, and model contains the language model files for the pocketsphinx recognizer.
####Source Code
Within the src directory are scripts for 6 ROS nodes. The comments within each script give a more thorough overview of the purpose of each node and the specifics of each function.

1. detector.py

detector.py continuously reads the data from the skeleton tracker and the pocketsphinx recognizer. It analyzes the body position of the user or the list of spoken words and publishes any configurations that match those required by the memory game. Additionally, the detector node handles the tracking of the main user.

The matching of skeleton data to a pose for the memory game is done by projecting the spatial positions of the user's shoulder, elbow, and hand into the x-z plane. From there, the angle of the vectors between the user's shoulder and elbow and between the elbow and hand are calculated. These vector angles are then compared to a dictionary of poses that contain bounds on these angles; and if the angles lie within the bounds for a specific pose, that pose is successfully matched.  
For example, one pose condition is:

3 : {'left' : {'upper_arm' : [-22.5, 22.5], 'forearm' : [-22.5, 22.5]},
		'right' : {'upper_arm' : [67.5, 112.5], 'forearm' : [67.5, 112.5]}},
		
All angles are determined with respect to horizontal formed by looking at the user from the robot's perspective. Thus, if the user's arms were straight horizontally (mimicking an axis), the user's left arm would represent the +x direction, the user's right arm would represent the -x direction, and +z and -z are standard up and down. The representation shown above means that the angles of the user's left upper arm and forearm are both within regions centered around 0 degrees, and the angles of the user's right upper and and forearm are both within regions centered around 90 degrees. This would correspond to the user making an "L" shape with the right and left arms.

All of the pose data, including the values of RJ's joint angles, the angular bounds for all poses, and the functions to compare angles and move between poses are all contained within the script poses.py. This script is non-executable and is imported into nodes to be able to call those functions. Poses.py also uses functions from the script vector_operations.py, which contains mathematical functions to determine angles from vectors.

The speech recognition comparison is done in a single function, parseSpeech. It is simply comparing strings found from the speech recognition data to those found in words.py (a list of strings for the game), and returning any match between them. Further information on the speech recognition data can be found in the pocketsphinx documentation section.

2. memory.py
3. voice_control.py
4. voice_pose.py
5. voice_trajectory.py
6. voicetest.py
