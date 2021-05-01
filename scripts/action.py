#!/usr/bin/env python3

import rospy
from q_learning_project.msg import RobotMoveDBToBlock
import numpy as np
import moveit_commander

'''
This node will subscribe to robot_action and listen for commands to pick up or 
drop a dumbbell at a specific location. It will publish messages to cmd_vel
and moveit to move turtlebot and arm according.
'''

class ActioRobotnNode(object):
    def __init__(self):
        # Set up traffic status publisher
        self.action_sub = rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.signal_received)
        self.movement_status_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        rospy.sleep(1)

    def signal_received(self, data):
        

    def run(self):
        # Once every 10 seconds
        rate = rospy.Rate(0.1)
        while (not rospy.is_shutdown()):
            self.traffic_status_pub.publish(trafficMsg)
            rate.sleep()

if __name__ == '__main__':
    robot = Robot()
    robot.run()