# q_learning_project Jonah Kaye Zack Wang 

## GIF
![gif][gif]

## Implementation Plan 
* Q-learning algorithm
    * Executing the Q-learning algorithm: Almost identical to in lecture 10, we define a matrix with rows for each state, and columns for each action (64 by 9). We will start in state 0, and pick a random action. We use the same algorithm from lecture 10 to update the matrix. In order to test it, we can manually calculate a single trajectory, starting from the blank Q-matrix, and see if that single step matches the computer execution. More robustly, we can take the Q-matrix at any time, specify a trajectory, and then compare the results of the program and calculate by hand. 
    * Determining when the Q-matrix has converged: To check that it has converged, we can do a test-run, where we start at the origin and do a large number of actions. We keep a running sum of the magnitude of changes. Should this magnitude exceed a set threshold, we know that it is not done converging. To test this, we take the Q-matrix that is deemed 'converged' by the program and randomly check the result from a state and an action. Basically, we select a state at random, and a valid action for that state at random: if the matrix is converged, the update step for this action should not change any Q-value (significantly). 
    * Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward: Just like in lecture 10, once the Q-matrix is constructed, the optimal reward is "trickled down" through the path to get there. For the current state, we take the maximum value of all the possible actions. To test, we manually check which action the program decides and compare that to what we "know apriori" is a correct choice. We know what states have a higher reward associated with them, so we can validate the choices by asking "is this action the program decides getting us to the optimal state?". 
* Robot perception
    * Determining the identities and locations of the three colored dumbbells: Using the color detection code from lecture 3, we can program color ranges for each of the dumbbells. We will have Turtlebot drive towards the dumbbells using scan, use the rgb camera to detect which dumbbell is which color, and save this information to map metadata. To test this, we place the Turtlebot in an environment with just one colored dumbbell, and see if it detects it using the RViz popup, and navigates to it (just eyeball test). We repeat this for each color. Once we confirmed the program is detecting each color correctly, we test to see if the map is being constructed to how we "know" it should look like. 
    * Determining the identities and locations of the three numbered blocks: Similar to the dumbbell process, except instead of using the rgb camera and color ranges, we use the keras library to identify the number. This may involve a more strict driving command, where the Turtlebot has to be face-to-face with the side with the number on it. To test, we first see if our keras method correctly identifies an image by situating the Turtlebot directly in front of a numbered block. After checking all possible numbers, we do a similar procedure from the dumbbell test, seeing if the Turtlebot correctly adds the numbered tiles to its map metadata. 
    * Picking up and putting down the dumbbells with the OpenMANIPULATOR arm: This can be broken down into smaller steps of- get to dumbbell based on stored location, extend arm, pinch and lift dumbbell. Putting it down is the same motions in reverse. To test this, we can just experiment with a basic environment of a dumbbell and Turtlebot right next to it. The method works if we can pick up the dumbbell off the ground and can release it similarly. 
    * Navigating to the appropriate locations to pick up and put down the dumbbells: With the locations stored from the visualization methods, we instruct the Turtlebot to move based on an error term, representing its current location and where we want it to go. A slight adjustment may be needed so that Turtlebot always maintains some distance from objects, to prevent it from ramming into objects. To test this, we initiate a command like "go to block labelled 1" and see if it navigates towards it. 
* Timeline
    * Q-Matrix: Built and testing by end of May 2nd
    * Visualization: Implemented and testing by May 5th
    * Arm movement: Implemented and testing by May 5th
    * Navigation: Implemented and testing by May 7th
    * Putting it together/overall testing: Up to May 10th  

# Writeup

## Objectives Description

The goal for this project is to encode a q-matrix, that has values that correspond to rewards for some world. Using this trained q-matrix, we can instruct the robot to manuever and place the dumbbells to optimize the reward.

## High Level Description
The q-matrix is constructed with rows representing the current state (where dumbbells are) and columns representing actions the robot can take (moving dumbbell to specific location). Each of these spaces represents a value, corresponding to a reward. To "train" the matrix, we simulate hundreds of possible actions, updating each space with a formula that roughly indicates the true "value" of choosing such an action. Because the rewards are published only once all three dumbbells are placed with a respective block, this reward will percolate through the possible actions. Once we've finished training, selecting the action corresponds to a better reward. We follow this maximizing action, which should guide us to the optimal reward. 

## Code Organization

* Q_Learning Algorithm: Training is done exclusively in the q_learning.py file. To select actions for the phantom robot, in the "test_an_action" method we consult the action matrix with the row corresponding to the current state. Values of -1 mean the robot cannot do such an action, so we select at random an action corresponding to a nonnegative value. We update the state, and publish the action. In the "update_q" callback function, we subscribe to a reward corresponding to the action published. Using the formula from class, we update the qmatrix with the reward received. We continue to do this until the matrix converges, which we determine in "test_convergence", which basically returns true if, after at least hundreds of iterations, the last 60 had no impact. 

* Action and Perception: In the action.py file, we handle the "executing the optimal path", "perception", and "robot manipulation" parts. During initialization, we read in the saved qmatrix csv file, and start in state 0. We then select the optimal actions as being the ones with the highest q-value, and save those actions in a queue. In the "run" method, we infinitely loop in a process to pick up dumbbells and place them according to the queue. This loop is based on two helper functions "dumbel_rec" and "image_rec". In "dumbel_rec", if we are not currently holding a dumbbell we use code similar to the LineFollower example to identify the color we are looking for, and drive towards it. This uses scan and rbg camera data to get us close enough to send a signal to the arm to pick it up. We then signal that we are ready for "image_rec" to run. In "image_rec", if we are holding a dumbbell we identify the block numbers using keras and the camera. This returns the list of blocks_ids as the robot sees them. Then, referencing the saved queue we drive to the block of the indicated block_id. We lower the block, and repeat the loop until all dumbbells are placed.  

## Challenges

The implementation of the mechanics of the actions of picking up and putting down the dumbells was incredibly difficult and time consuming. To compensate, our implementation was just tested over and over, until it worked a majority of the time. Gazebo was also quite glitchy and needed to be shut down and restarted each time, and we were never able to completely eliminate random errors, like the robot LIDAR occasionally reading infinite values and zooming towards the blocks. We also had a lingering error where the range of values we used for red was on the wrong end of the spectrum, so the robot could only pick it up from one angle, but from another, the lighting was dark enough because of the shadow that it couldnt recognize the red dumbell. Finally, properly implementing the pid to drop off the dumbells at the blocks was a challenge. We could use the coordinates from the OCR pipeline because it didnt update fast enough, so we had to just use LIDAR. But because the angles were different and the dumbell was being carried on the right side of the robot, placing the dumbell in front of the left most block was a challenge, and we were ultimately unable to get the dumbell in front of the leftmost block. 

## Future Work 

With more time, we would fix the issue of the leftmost block and dumbell, and implement a more effective version of PID than lidar to navigate more precisely. We might have also found a way to direct our robot not towards the first part of the block it sees, but to the center of the block instead. This could be achieved by  measuring the distance from the edge to the center of the block. We would have also liked to make the robot move faster, and improved its accuracy in picking up dumbells. I would say our robot works and picks and drops everything off without dropping the dumbells around 51% of the time, and in future work, we would bring that percentage up. Breaking code into more helper functions, with less code in each, would also make the project more flexible. Another change would be to replace keras with a faster method of identifying numbers, so we could analyze and direct the Turtlebot more actively. 


## Takeaways

1) There is a huge difference between writing the logic of code that is correct and getting it to functionally work with detailed and fine-tuned robot tasks. Everything needs to be endlessly tuned based on the actual physics of the robot dynamics

2) Certain pipelines and built in processes have serious limitations. The Keras OCR needed to be fed a range of strings that look like numbers, and the OCR needed several secconds to run the pipeline on each image, creating constraints for PID. It is crucial to either significantly accomodate for these drawbacks or explore alternatives 

[gif]: https://github.com/Zwky26/q_learning_project/blob/c93b73215f19d55292b5d7b44ded38ea73f373e6/scripts/robot_vid3.gif
