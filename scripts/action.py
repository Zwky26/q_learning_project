#!/usr/bin/env python3
import cv2, cv_bridge
import rospy
from q_learning_project.msg import RobotMoveDBToBlock
import numpy as np
import moveit_commander
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


'''
This node will subscribe to robot_action and listen for commands to pick up or 
drop a dumbbell at a specific location. It will publish messages to cmd_vel
and moveit to move turtlebot and arm according.
'''

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
        self.move_group_arm.go([0,0,0,0], wait=True)

        self.laser_scan = rospy.Subscriber("/scan", LaserScan, self.object_identify)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
        self.my_twist = Twist(linear=Vector3(0, 0, 0),angular=Vector3(0, 0, 0))
        self.robot_movement_pub.publish(self.my_twist)

        self.color = "green"
        self.laser_data = 0.5

        #green = np.uint8([[[0,255,0 ]]])
        #hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
        #print(hsv_green)

        rospy.sleep(1)

    def object_identify(self,msg):
    
        self.laser_data = msg.ranges[0]

    def image_callback(self,msg):
        
        #print("color scan")

        #if self.want != 1:
            #print("turning")
            #self.my_twist.angular.z = .05
            #self.robot_movement_pub.publish(self.my_twist)

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # TODO: define the upper and lower bounds for what should be considered 'green'

        if self.color == "green":
            lower_color= np.array([60, 60, 60])
            upper_color = np.array([65, 255, 250])
        else if self.color = "red":
            lower_color = np.array([161, 155, 84])
            higher_color = np.array([179, 255, 255])
        else # "blue" 
            lower_color = np.array([94, 80, 2])
            higher_color = np.array([126, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        # this erases all pixels that aren't yellow
        h, w, d = image.shape
        

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
            self.my_twist.linear.x = (self.laser_data - 0.5)*.1
            
            self.my_twist.angular.z = (w/2 - cx) * 0.001 

            gripper_joint_close = [-0.01, -0.01]
            gripper_joint_open = [0, 0]
            if (w/2 - cx) < 0.5:
                self.move_group_gripper.go(gripper_joint_close)
            
            self.robot_movement_pub.publish(self.my_twist)
       
   # def signal_received(self, data):

    def run(self):
        # Once every 10 seconds
        #rate = rospy.Rate(0.1)
        #while (not rospy.is_shutdown()):
            ### self.traffic_status_pub.publish(trafficMsg)
            #rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    robot = ActionRobotNode()
    robot.run()