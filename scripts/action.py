#!/usr/bin/env python3
import cv2, cv_bridge
import os
import rospy
import random
from q_learning_project.msg import RobotMoveDBToBlock
import numpy as np
import moveit_commander
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import QMatrix, QMatrixRow
import keras_ocr
from csv import reader

'''
This node will subscribe to robot_action and listen for commands to pick up or 
drop a dumbbell at a specific location. It will publish messages to cmd_vel
and moveit to move turtlebot and arm according.
'''

PI = 3.1415926535897
right_angle = 90*2*PI/360
angular_speed = 15*2*PI/360
path_prefix = os.path.dirname(__file__) + "/action_states/"

def next_state_calc(r,g,b):
    return r + (16 * b) + (4 * g)

class ActionRobotNode(object):
    def __init__(self):
     # Set up traffic status publisher
        # set up ROS  / OpenCV bridge
        rospy.init_node("action")

        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)

        #self.action_sub = rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.signal_received)
        self.robot_movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        self.rest_pos = [0, .7, -.3, -.3]
        self.lift_pos = [0, .3, -.8, -.3]
        self.turn_pos = [1.5, .3, -.8, -.3]
        self.lower_pos = [1.5, .7, -.3, -.3]
        self.open_grip = [0.010, 0.010]
        self.close_grip = [0.007, 0.007]
        self.move_group_arm.go(self.rest_pos, wait=True)
        self.move_group_gripper.go(self.open_grip, wait=True)

        self.laser_scan = rospy.Subscriber("/scan", LaserScan, self.object_identify)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
        self.my_twist = Twist(linear=Vector3(0, 0, 0),angular=Vector3(0, 0, 0))
        self.robot_movement_pub.publish(self.my_twist)

        self.color = "blue"
        self.block_id = 1
        self.laser_data = 0.5
        self.holding = 0
        self.w = 0 
        self.q = []

        # open file in read mode
        with open('qmatrix.csv', 'r') as read_obj:
            # pass the file object to reader() to get the reader object
            csv_reader = reader(read_obj)
            # Iterate over each row in the csv using reader object
            for row in csv_reader:
                # row variable is a list that represents a row in csv
                self.q.append(row)
        
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))

        current_state = 0
        to_do = []
        r = 0
        g = 0
        b = 0
        while (int(max(self.q[current_state])) > 0):
            biggest = int(max(self.q[current_state]))
            # print(biggest)
            viable = []
            for i in range(len(self.q[current_state])):
                if int(self.q[current_state][i]) == biggest:
                    a = self.actions[i]
                    d = a['dumbbell']
                    #i = a['block']
                    if ((r != d) and (b != d) and (g != d)):
                        viable.append(i)
            action = random.choice(viable)
            act = self.actions[action]
            robot_db = act['dumbbell']
            block_id = act['block']
            print(robot_db, block_id)
            to_do.append((robot_db, block_id))
            if robot_db == "red":
                r = block_id
            elif robot_db == "green":
                g = block_id
            elif robot_db == "blue":
                b = block_id
            current_state = next_state_calc(r,g,b)
            print(current_state)
        print(to_do)

        #green = np.uint8([[[0,255,0 ]]])
        #hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
        #print(hsv_green)
        self.pipeline = keras_ocr.pipeline.Pipeline()
        
        rospy.sleep(1)

    def pickup_dumbbell(self):
        self.move_group_gripper.go(self.close_grip, wait=True)
        self.move_group_arm.go(self.lift_pos, wait=True)

    def lower_dumbbell(self):
        self.move_group_arm.go(self.rest_pos, wait=True)
        self.move_group_gripper.go(self.open_grip, wait=True)


    def object_identify(self,msg):
    
        self.laser_data = msg.ranges[0]

    def image_callback(self,msg):
        
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        h, w, d = self.image.shape
        self.w = w

    def dumbel_rec(self):

        if self.holding == 0:
            #print("here1")
            hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
            # TODO: define the upper and lower bounds for what should be considered 'green'
            if self.color == "green":
                lower_color= np.array([60, 60, 60])
                upper_color = np.array([65, 255, 250])
            elif self.color == "red":
                lower_color = np.array([161, 155, 84])
                upper_color = np.array([179, 255, 255])
            else: # "blue"
                lower_color = np.array([94, 80, 2])
                upper_color = np.array([126, 255, 255])
            mask = cv2.inRange(hsv, lower_color, upper_color)
            # using moments() function, the center of the yellow pixels is determined
            M = cv2.moments(mask)
                # if there are any yellow pixels found
            ##print ("here")
            if M['m00'] > 0:
                # center of the pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                #print("cx and cy:", cx, cy)
                if self.laser_data > 3.5:
                    self.laser_data = 3.5
                print("lzr", self.laser_data)
                self.my_twist.linear.x = max (.0075,((self.laser_data - 0.24)*.085))
                self.my_twist.angular.z = (self.w/2 - cx) * 0.001 
                if (self.laser_data) < 0.24:
                    self.my_twist.linear.x = 0
                    self.robot_movement_pub.publish(self.my_twist)
                    self.move_group_gripper.go(self.close_grip, wait=True)
                    self.move_group_arm.go(self.lift_pos, wait=True)
                    self.move_group_arm.go(self.turn_pos, wait=True)
                    self.holding = 1
                    self.move_group_arm.stop()
                    self.move_group_gripper.stop()
                self.robot_movement_pub.publish(self.my_twist)

    def move_backwards(self):
        self.my_twist.linear.x = -.1
        t0 = rospy.Time.now().to_sec()
        time_count = 0
        while (time_count <= 6): # The turtlebt moves backwards
            self.robot_movement_pub.publish(self.my_twist)
            t1 = rospy.Time.now().to_sec()
            time_count = t1 -t0
        self.my_twist.linear.x = 0
        self.robot_movement_pub.publish(self.my_twist)

    def turn_right(self):
        current_angle = 0
        self.my_twist.angular.z = angular_speed
        t0 = rospy.Time.now().to_sec()    
        while (current_angle <= right_angle): # The turtlebot turns left until it has made a 90 degree turn
            self.robot_movement_pub.publish(self.my_twist)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1-t0) 

    def image_rec(self):
        if self.holding == 1:
            #print("id", self.block_id)

            threes = ["3","s","e","5"]
            twos = ["2"]
            ones = ["1","l", "1l"]

            
            #self.robot_movement_pub.publish(self.my_twist)
            self.move_backwards()
            self.turn_right()
            
            block_count = 0
            while block_count < 5:
                if self.laser_data < 3.5: 
                    self.my_twist.angular.z = 0
                    self.robot_movement_pub.publish(self.my_twist)
                    images = [self.image]
                    p = self.pipeline.recognize(images)
                    print(p[0])
                    id = ""
                    id2 = 0
                    left = 1000
                    cx_dict = {}
                    for b,a in p[0]:
                        #print("prediction?", b )
                        cx = 0
                        for i in a:
                            if i[0] < left: 
                                id = b
                                left = i[0]
                            print(i) 
                            cx += i[0]
                        #print("batch")
                        #cx = cx /4
                        #cx_dict[b] = cx
                    #cx_final = cx_dict[id]
                    print("prediction" , id)
                    if id in ones:
                        id2 = 1
                    elif id in twos:
                        id2 = 2
                        print("id is twooooo")
                    elif id in threes:
                        id2 = 3
                    print("id2", id2)
                    print("block_id", self.block_id)
                    print("bool?", self.block_id - id2)
                    if id2 == self.block_id:
                        print("going forward")
                        #print("half width", self.w/2)
                        while self.laser_data > 1:
                            # self.robot_movement_pub.publish(self.my_twist)
                            print("lzr", self.laser_data)
                            self.my_twist.linear.x = max(.05, (self.laser_data - 0.5)*.08)
                            self.robot_movement_pub.publish(self.my_twist)
                            # self.my_twist.angular.z = (self.w/2 - cx_final) * 0.005 
                            # images = [self.image]
                            # p = self.pipeline.recognize(images)
                            # cx_final,junk_id = self.pipeline_helper(p)
                        self.move_group_arm.go(self.lift_pos, wait=True)
                        self.lower_dumbbell()
                        self.holding = 0
                        self.move_backwards()
                        
                        self.turn_right()
                        self.turn_right()
                        self.move_group_gripper.go(self.open_grip, wait=True)
                        self.color = "green"
                        self.block_id = 2
                        break
                    block_count += 1
                    self.my_twist.angular.z = angular_speed
                    self.robot_movement_pub.publish(self.my_twist)
                    #print(self.laser_data)
                    while self.laser_data < 3.5: 
                        #print(self.laser_data)
                        pass
            print("hit three blocks")

    def run(self):
        while (not rospy.is_shutdown()):
            #self.dumbel_rec()
            #print("switch modes")
            #self.image_rec()
            pass
        #rospy.spin()

if __name__ == '__main__':
    robot = ActionRobotNode()
    robot.run()