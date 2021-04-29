# q_learning_project Jonah Kaye Zack Wang 

## Implementation Plan (Rough Draft)

* Q-learning algorithm
    * Executing the Q-learning algorithm: Almost identical to in lecture 10, we define a matrix with rows for each state, and columns for each action (64 by 9). We will start in state 0, and pick a random action. We use the same algorithm from lecture 10 to update the matrix. 
    * Determining when the Q-matrix has converged: To check that it has converged, we can do a test-run, where we start at the origin and do a large number of actions. We keep a running sum of the magnitude of changes. Should this magnitude exceed a set threshold, we know that it is not done converging. 
    * Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward: Just like in lecture 10, once the Q-matrix is constructed, the optimal reward is "trickled down" through the path to get there. For the current state, we take the maximum value of all the possible actions. 
* Robot perception
    * Determining the identities and locations of the three colored dumbbells: Using the color detection code from lecture 3, we can program color ranges for each of the dumbbells. We will have Turtlebot drive towards the dumbbells using scan, use the rgb camera to detect which dumbbell is which color, and save this information to map metadata. 
    * Determining the identities and locations of the three numbered blocks: Similar to the dumbbell process, except instead of using the rgb camera and color ranges, we use the keras library to identify the number. This may involve a more strict driving command, where the Turtlebot has to be face-to-face with the side with the number on it. 
    * Picking up and putting down the dumbbells with the OpenMANIPULATOR arm: Steps broken down to- get to dumbbell based on stored location, extend arm, pinch and lift dumbbell, move to box, do these arm movements in reverse. 
    * Navigating to the appropriate locations to pick up and put down the dumbbells: either have location stored in state as coordinate using SLAM? or use color camera like the line-follower-lecture from class. Find the color, drive towards it
* Timeline


