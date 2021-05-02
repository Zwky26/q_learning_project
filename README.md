# q_learning_project Jonah Kaye Zack Wang 

## Implementation Plan (Rough Draft)

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

## Code Organization

Because the code is scattered across multiple files, outlines here. 

* Q_Learning file: We initialize with the proper subscribers and publishers, and define a Q-matrix according to the fixed actions included. For the node, every x seconds we do the same procedure. Check if the Q-matrix has converged. If yes, save the matrix to a csv and exit. Otherwise, we pick a random action to be tested. We update the qmatrix using the reward and our new state from the action matrix and repeat. We have a property "waiting" so that the publishing step and the updating step don't overlap. 

