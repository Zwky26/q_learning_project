#!/usr/bin/env python3

import rospy
from q_learning_project.msg import RobotMoveDBToBlock
import numpy as np

'''
This node will subscribe to robot_action and listen for commands to pick up or 
drop a dumbbell at a specific location. It will publish messages to cmd_vel
and moveit to move turtlebot and arm according.
'''

class ActionNode(object):
    def __init__(self):
        # Set up traffic status publisher
        self.traffic_status_pub = rospy.Publisher("/traffic_status", Traffic)

        # Counter to loop publishing direction with
        self.direction_counter = 0
        rospy.sleep(1)

    def run(self):
        # Once every 10 seconds
        rate = rospy.Rate(0.1)
        while (not rospy.is_shutdown()):
            trafficMsg = Traffic()
            trafficMsg.direction = self.direction_counter % 3
            self.direction_counter += 1
            self.traffic_status_pub.publish(trafficMsg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('movement_controller')
    action_node = ActionNode()
    action_node.run()