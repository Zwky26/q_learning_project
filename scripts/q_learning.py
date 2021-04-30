#!/usr/bin/env python3

import rospy
import numpy as np
import os
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveDBToBlock
import random
from q_learning_project.msg import QMatrix
# import messages. 


# Path of directory on where this file is located
path_prefix = "/home/jonahkaye/catkin_ws/src/q_learning_project/scripts" + "/action_states/"
print(path_prefix)

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        alpha = 1 
        gamma = 0.5 
        
        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { dumbbell: "red", block: 1}
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))

        # publish the current matrix 
        self.matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        self.reward_sub = rospy.Subscriber("/q_learning/reward", QLearningReward, self.update_q) # how do I make this give me different values
        self.execute_pub = rospy.Publisher("/q_learning/robot_action" , RobotMoveDBToBlock, queue_size=10)


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the red, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        # Initialize q-table values to 0
        self.Q = np.zeros((len(self.states), len(self.actions)))
        self.current_state = 0
        self.next_state = 0 
        self.q_count = 0

        self.MyMove = RobotMoveDBToBlock()

        ##print("actions", self.actions)
        ##print("states", self.states)
        ##print("action_matrix", self.action_matrix[0][12])
    
    def update_q(self, data): # Main logic occurs here, as the rostopic subscriber gets updates to reward values

        print("update_q")
        if self.q_convergence:
            self.save_q_matrix
            exit()
        self.Q[self.current_state, self.action] = self.Q[self.current_state, self.action] + alpha * (data.reward  + gamma * int(np.max(self.Q[self.new_state, :])) - self.Q[self.current_state, self.action])
        Q_anon = self.serialize_q_matrix()
        self.execute_pub.publish(Q_anon)
        self.current_state = self.new_state
        # pick random and publish
        self.next_state, self.action = self.random_action(self.action_matrix[current_state])
        ## MyMove = RobotMoveDBToBlock()
        self.MyMove.robot_db = self.actions[int(self.action)]['dumbbell']
        self.MyMove.block_id = self.actions[int(self.action)]['block']
        self.execute_pub.publish(self.MyMove)

    def q_convergence(self):
        print("q_convergence")
        if self.q_count > 10:
            return 1
        else: 
            self.q_count += 1
            return 0

    def serialize_q_matrix(self):
        # takes a numpy q anon and turns it into the proper msg type to be published
        print("serialize_the_q")
        Q_anon = QMatrix()
        for i in range(len(self.Q)):
            Q_row = QMatrixRow()
            Q_row.q_matrix_row = self.Q[i]
            Q_anon.q_matrix[i] = Q_row
        return Q_anon

    def random_action(self, row):
        print("random_action")
        viable = [] 
        for i in range(len(row)):
            if row[i] != -1:
                viable.append((i,row[i]))
        return random.choice(viable)

    def run(self): #First iteration of selecting the random action. All subsequent selections happen in the callback 
        print("run!")
        ## print(self.action_matrix[self.current_state])
        self.next_state, self.action = self.random_action(self.action_matrix[self.current_state])
        ## print (self.actions[int(action)])
        ## print (self.states[state])
        ## print(state, action) 
        self.MyMove.robot_db = self.actions[int(self.action)]['dumbbell']
        self.MyMove.block_id = self.actions[int(self.action)]['block']
        self.execute_pub.publish(self.MyMove)

    def save_q_matrix(self):
        print("save_the_q")
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        for i in self.Q:
            for j in self.Q[i]:
                print(" " + j + " ")
            print("\n")

if __name__ == "__main__":
    print("execute")
    node = QLearning()
    node.run()
