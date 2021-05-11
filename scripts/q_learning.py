#!/usr/bin/env python3

import rospy
import numpy as np
import os
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveDBToBlock
import random
from q_learning_project.msg import QMatrix, QMatrixRow
import csv
# import messages 


# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"
# print(path_prefix)

class QLearning(object):
    def __init__(self):
        ''' Defines publisher to qmatrix and subscriber to reward.
        Also makes blank qmatrix'''
        # Initialize this node
        rospy.init_node("q_learning")
 
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

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the red, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        #publishers
        self.matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10) #for q matrix
        self.reward_sub = rospy.Subscriber("/q_learning/reward", QLearningReward, self.update_q) #for rewards based on action
        self.execute_pub = rospy.Publisher("/q_learning/robot_action" , RobotMoveDBToBlock, queue_size=10) #for actions
        rospy.sleep(1)

        # Initialize q-table values to 0, starting state to 0, etc
        self.Q = self.init_q_matrix()
        self.current_state = 0
        self.next_state = -1
        self.action = -1 
        self.q_count = 0 #tracks how many steps done
        self.in_a_row = 0 #tracks how many uneventful steps have occurred in a row
        self.converged = False #bool for convergence
        self.waiting = False #bool to make sure we dont have overlap of topics

    def init_q_matrix(self):
        ''' initializes the q matrix with all zeros'''
        m = QMatrix()
        m.q_matrix = []
        for _ in range(len(self.states)):
            row = QMatrixRow()
            row.q_matrix_row = [0] * len(self.actions)
            m.q_matrix.append(row)
        return m
    
    def update_q(self, data):
        '''called when we receive the reward info for the tested action. We want to use
        the algorithm from class to update the qmatrix, then publish'''
        alpha = 1
        gamma = 0.8
        old_q_matrix = self.Q.q_matrix
        reward = data.reward
        old_q_row = old_q_matrix[self.current_state].q_matrix_row #row of q vals, yet to be updated
        old_q_val = old_q_row[self.action]
        future_state = old_q_matrix[self.next_state].q_matrix_row
        change = alpha * (reward + (gamma * max(future_state) - old_q_val)) #formula from class
        old_q_val += change
        old_q_row[self.action] = int(old_q_val)
        self.Q.q_matrix[self.current_state].q_matrix_row = old_q_row
        self.matrix_pub.publish(self.Q)
        self.current_state = self.next_state
        self.waiting = False
        self.q_count += 1
        if change <= 1: #if no significant changes
            self.in_a_row += 1
        else:
            self.in_a_row = 0
        self.test_convergence()

    def test_convergence(self):
        ''' method for testing convergence. Basically have floor of how many iterations'''
        print(self.q_count)
        if self.q_count > 500:
            if self.in_a_row > 60:
                self.converged = True

    def test_an_action(self):
        ''' find all viable actions, pick one and publish '''
        row = self.action_matrix[self.current_state]
        viable = [] 
        for i in range(len(row)):
            if row[i] != -1:
                viable.append((int(i),row[i]))
        if viable == []: #happens when all blocks are filled, just reset to blank world and start again
            self.current_state = 0
        else: #gets the action, and updates the states and other properties respectively
            choice = random.choice(viable)
            self.next_state = choice[0]
            self.action = int(choice[1])
            self.waiting = True
            act = self.actions[self.action]
            action_msg = RobotMoveDBToBlock()
            action_msg.robot_db = act['dumbbell']
            action_msg.block_id = act['block']
            #print(action_msg)
            self.execute_pub.publish(action_msg)      

    def run(self):
        ''' runs infinitely. If we are not converged, test an action if we arent waiting for
        a reard message'''
        rate = rospy.Rate(3)        
        while (not rospy.is_shutdown()):
            if self.converged:
                self.save_q_matrix()
                exit()
            else:
                if not self.waiting:
                    # if ready to try another action, do another
                    self.test_an_action()
            rate.sleep()

    def save_q_matrix(self):
        # Saving q matrix to csv file, row by row
        rows = []
        for i in self.Q.q_matrix:
            rows.append(i.q_matrix_row)

        #csv transcription code from stackoverflow
        with open('qmatrix_saved.csv', 'w') as f:
        # using csv.writer method from CSV package
            write = csv.writer(f)
            write.writerows(rows)

if __name__ == "__main__":
    node = QLearning()
    rospy.sleep(3)
    node.run()
