#!/usr/bin/env python3
import cv2, cv_bridge
import rospy
from q_learning_project.msg import RobotMoveDBToBlock
import numpy as np
import moveit_commander
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import keras_ocr


'''
This node will subscribe to robot_action and listen for commands to pick up or 
drop a dumbbell at a specific location. It will publish messages to cmd_vel
and moveit to move turtlebot and arm according.
'''

PI = 3.1415926535897
right_angle = 90*2*PI/360
angular_speed = 15*2*PI/360

class ActionRobotNode(object):
    def __init__(self):
     # Set up traffic status publisher
        # set up ROS / OpenCV bridge
        rospy.init_node("action")

        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)

        #self.action_sub = rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.signal_received)
        self.robot_movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        self.rest_pos = [0, .7, -.3, -.3]
        self.lift_pos = [0, .4, -.9, -.3]
        self.open_grip = [0.010, 0.010]
        self.close_grip = [0.007, 0.007]
        self.move_group_arm.go(self.rest_pos, wait=True)
        self.move_group_gripper.go(self.open_grip, wait=True)

        self.laser_scan = rospy.Subscriber("/scan", LaserScan, self.object_identify)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
        self.my_twist = Twist(linear=Vector3(0, 0, 0),angular=Vector3(0, 0, 0))
        self.robot_movement_pub.publish(self.my_twist)

        self.color = "green"
        self.block_id = 1
        self.laser_data = 0.5
        self.holding = 1
        self.w = 0 

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
        if self.holding == 0:
            
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
                self.my_twist.linear.x = (self.laser_data - 0.24)*.08
                
                self.my_twist.angular.z = (w/2 - cx) * 0.001 

                if (self.laser_data) < 0.24:
                    self.my_twist.linear.x = 0
                    self.robot_movement_pub.publish(self.my_twist)
                    self.move_group_gripper.go(self.close_grip, wait=True)
                    self.move_group_arm.go(self.lift_pos, wait=True)
                    self.holding = 1
                    self.move_group_arm.stop()
                    self.move_group_gripper.stop()
                self.robot_movement_pub.publish(self.my_twist)
       
   # def signal_received(self, data):
    def pipeline_helper(self,p):
        id = ""
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
            cx = cx /4
            cx_dict[b] = cx
        x_final = cx_dict[id]
        return x_final, id       

    def run(self):
        if self.holding == 1:

            threes = ["3","s","e","5"]
            twos = ["2"]
            ones = ["1","l", "1l"]

            self.my_twist.linear.x = -.1
            #self.robot_movement_pub.publish(self.my_twist)

            t0 = rospy.Time.now().to_sec()
            time_count = 0
            while (time_count <= 6): # The turtlebot turns left until it has made a 90 degree turn
                self.robot_movement_pub.publish(self.my_twist)
                t1 = rospy.Time.now().to_sec()
                time_count = t1 -t0
            self.my_twist.linear.x = 0
            self.robot_movement_pub.publish(self.my_twist) 

            current_angle = 0
            self.my_twist.angular.z = angular_speed
            block_count = 0
            t0 = rospy.Time.now().to_sec()    
            while (current_angle <= right_angle): # The turtlebot turns left until it has made a 90 degree turn
                self.robot_movement_pub.publish(self.my_twist)
                t1 = rospy.Time.now().to_sec()
                current_angle = angular_speed * (t1-t0)
            while block_count < 5:
                if self.laser_data < 3.5: 
                    self.my_twist.angular.z = 0
                    self.robot_movement_pub.publish(self.my_twist)
                    images = [self.image]
                    p = self.pipeline.recognize(images)
                    #print(p[0])
                    id = ""
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
                        cx = cx /4
                        cx_dict[b] = cx
                    cx_final = cx_dict[id]
                    print("prediction" , id)
                    if id in ones:
                        id2 = 1
                    elif id in twos:
                        id2 = 2
                    elif id in threes:
                        id2 = 3
                    if id2 == self.block_id:
                        print("cx_final" , cx_final)
                        print("half width", self.w/2)
                        while self.laser_data > 0.5:
                            self.robot_movement_pub.publish(self.my_twist)
                            self.my_twist.linear.x = (self.laser_data - 0.5)*.1
                            # self.my_twist.angular.z = (self.w/2 - cx_final) * 0.005 
                            # images = [self.image]
                            # p = self.pipeline.recognize(images)
                            # cx_final,junk_id = self.pipeline_helper(p)
                        break
                    block_count += 1
                    self.my_twist.angular.z = angular_speed
                    self.robot_movement_pub.publish(self.my_twist)
                    #print(self.laser_data)
                    while self.laser_data < 3.5: 
                        #print(self.laser_data)
                        pass
            print("hit three blocks")
        rospy.spin()

if __name__ == '__main__':
    robot = ActionRobotNode()
    robot.run()