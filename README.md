# q_learning_project

## Implementation Plan (Rough Draft)

* Q-learning algorithm
    * Executing the Q-learning algorithm: define matrix and run the algorithm provided from class over and over? For each iteration, we go through whole matrix and keep track of the sum of changes we updated that iteration. After that drops below threshhold stop
    * Determining when the Q-matrix has converged
    * Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
* Robot perception
    * Determining the identities and locations of the three colored dumbbells
    * Determining the identities and locations of the three numbered blocks: Using SLAM and image recognition library, store locations of boxes?
* Robot manipulation & movement
    * Picking up and putting down the dumbbells with the OpenMANIPULATOR arm: Steps broken down to- get to dumbbell, extend arm, pinch and lift dumbbell, move to box, do arm movements in reverse, return to start?
    * Navigating to the appropriate locations to pick up and put down the dumbbells: either have location stored in state as coordinate using SLAM? or use color camera like the line-follower-lecture from class. Find the color, drive towards it
* Timeline


