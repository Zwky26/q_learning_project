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

Action and Perception: In the action.py file, we handle the "executing the optimal path", "perception", and "robot manipulation" parts. During initialization, we read in the saved qmatrix csv file, and start in state 0. In the "run" method, we infinitely loop in a process to pick up dumbbells and place them according to the the q-values. This loop is based on two helper functions "dumbel_rec" and "image_rec". 

This is where we will send the actual cmd_vel and arm messages to maneuver the robot, based on the saved q matrix. The overall layout is: we read from the q matrix to determine the action. This action will tell us a color for the dumbbell and a number for the block. We call a method to drive to the dumbbell, which will loop until we reach it. Then, we call another method to pick it up and rotate the arm, so the dumbbell is not blocking the camera (might be optional). We then issue a third method to drive to the block, using the computer vision code from lecture 11. We run the second method in reverse, placing the dumbbell where is should go. One minor addition that might be needed is a helper method that rotates the Turtlebot around until it gets visual of what it is looking for. That is, if something is out of sight, find it.

[gif]: https://github.com/Zwky26/q_learning_project/blob/c93b73215f19d55292b5d7b44ded38ea73f373e6/scripts/robot_vid3.gif