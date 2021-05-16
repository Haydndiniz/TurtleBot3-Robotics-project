#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import actionlib

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image, LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

# Import some image processing modules:

import cv2
from cv_bridge import CvBridge

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
import math
from math import sqrt, pow, pi
import numpy as np
import time

# You need to know the coordinates that the map you are working
# in is. I experimented with these numbers and the turtlebot3_stage_4
# map from Turtlebot3. The first array is x,y,z location. The second one
# is a "quaternion" defining an orientation. Quaternions are a different
# mathematical represetnation for "euler angles", yaw, pitch and roll.

waypoints = [[(1.4231, -1.451, 0.0), (0.0, 0.0, 0.3189, -0.947783)],
            [(-0.368,  -0.004, 0.0), (0.0, 0.0, -0.56205,  -0.82719)],
            [(-1.31, -0.3, 0.0), (-0.00158, 0.0,  0.9854, -0.1698)],
            [(0.597, 1.37, 0.0), (0.0, 0.0011, -0.66712, 0.7449)],
            [(-0.06, 1.79, 0.0), (0.0, 0.0,  0.9334,  0.3586)]]


class search_and_beacon(object):

    def __init__(self):
        rospy.init_node('detect_colour')
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)
        self.mb_client = actionlib.SimpleActionClient('move_base',
                MoveBaseAction)
        self.mb_pub = rospy.Publisher('/initialpose',
                PoseWithCovarianceStamped, queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = \
            rospy.Subscriber('/camera/rgb/image_raw', Image,
                             self.camera_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom',  Odometry, self.odom_callback)
        self.vel = Twist()

        self.cvbridge_interface = CvBridge()
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.goal_pose = MoveBaseGoal()

        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = ''  # fast, slow or stop
        self.rate = rospy.Rate(5)
        self.m00 = 0
        self.m00_min = 10000

        self.mask = np.zeros((1080, 1920, 1), np.uint8)
        self.hsv_img = np.zeros((1080, 1920, 3), np.uint8)

        self.find_init_color = False
        self.target_colour = ''
        self.lower_bound = []
        self.upper_bound = []

        self.find_target = False

        self.colours = {
            'Red': ([0, 185, 100], [10, 255, 255]),
            'Yellow': ([25, 210, 100], [32, 255, 255]),
            'Green': ([25, 205, 100], [70, 255, 255]),
            'Turquoise': ([80, 150, 100], [100, 255, 255]),
            'Blue': ([115, 224, 100], [130, 255, 255]),
            'Purple': ([140, 160, 100], [160, 255, 255]),
            }

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def laser_callback(self,scan_data):
        # front obstacles, 20 degrees (10 left, 10 right)
        front_left_arc = scan_data.ranges[0:10]
        front_right_arc = scan_data.ranges[-10:]
        front_arc = np.array(front_left_arc[::-1] + front_right_arc[::-1])
        front_arc_angle = np.arange(-10, 10)
        # Set distance and bearing of closest obstacle in front of bot
        self.front_distance = front_arc.min()
        self.front_angle = front_arc_angle[np.argmin(front_arc)]
   
    def odom_callback(self, topic_data):            
        self.ori_x = topic_data.pose.pose.orientation.x
        self.ori_y = topic_data.pose.pose.orientation.y
        self.ori_z = topic_data.pose.pose.orientation.z
        self.ori_w = topic_data.pose.pose.orientation.w

        self.pos_x = topic_data.pose.pose.position.x
        self.pos_y = topic_data.pose.pose.position.y
        self.pos_z = topic_data.pose.pose.position.z
        
    # Set 2D Pose Estimate
    def set_initial_pose(self):
        checkpoint = PoseWithCovarianceStamped()
        checkpoint.pose.pose.position.x = self.pos_x
        checkpoint.pose.pose.position.y = self.pos_y
        checkpoint.pose.pose.position.z = 0.0
        checkpoint.pose.pose.orientation.x = self.ori_x
        checkpoint.pose.pose.orientation.y = self.ori_y 
        checkpoint.pose.pose.orientation.z = self.ori_z 
        checkpoint.pose.pose.orientation.w = self.ori_w
        self.mb_pub.publish(checkpoint)

    def go_to_waypoint(self, pose):
        self.goal_pose.target_pose.header.frame_id = 'map'
        self.goal_pose.target_pose.pose.position.x = pose[0][0]
        self.goal_pose.target_pose.pose.position.y = pose[0][1]
        self.goal_pose.target_pose.pose.position.z = pose[0][2]
        self.goal_pose.target_pose.pose.orientation.x = pose[1][0]
        self.goal_pose.target_pose.pose.orientation.y = pose[1][1]
        self.goal_pose.target_pose.pose.orientation.z = pose[1][2]
        self.goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return self.goal_pose

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data,
                    desired_encoding='bgr8')
        except CvBridgeError, e:
            print e

        (height, width, channels) = cv_img.shape
        crop_width = width - 300
        crop_height = 300
        crop_x = int(width / 2 - crop_width / 2)
        crop_y = int(height / 2 - crop_height / 2)

        crop_img = cv_img[crop_y:crop_y + crop_height, crop_x:crop_x
                          + crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.hsv_img = hsv_img

        if len(self.lower_bound) > 0 and len(self.upper_bound) > 0:
            self.mask = cv2.inRange(hsv_img, self.lower_bound,
                                    self.upper_bound)

        m = cv2.moments(self.mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255),
                       2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def get_colour(self):
        self.rotate(90, 0.3)
        for (colour, (lower, upper)) in self.colours.items():
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            mask = cv2.inRange(self.hsv_img, lower_bound, upper_bound)
            if mask.any():
                self.target_colour = colour
                self.lower_bound = lower_bound
                self.upper_bound = upper_bound
                self.rotate(90, -0.3)
                print 'SEARCH INITIATED: The target colour is {}'.format(self.target_colour)
                return

    def rotate(self, deg, speed):
        rospy.sleep(1)
        seconds = math.radians(deg) / speed
        self.robot_controller.set_move_cmd(0.0, speed)
        self.robot_controller.publish()
        time = abs(seconds)
        rospy.sleep(time)
        self.robot_controller.stop()

    def check_colour(self):
        if self.m00 > self.m00_min and self.find_target == False:
            # blob detected
            if self.cy >= 560 - 100 and self.cy <= 560 + 100:
                if self.move_rate == 'slow':
                    self.move_rate = 'stop'
                    print 'SEARCH COMPLETE: The robot is now facing the target pillar.'
                    self.find_target = True
                else:
                    self.move_rate = 'slow'
            elif self.find_target == True:
                self.robot_controller.stop()
            else:
                self.move_rate = 'fast'

            if self.find_target == False:
                if self.move_rate == 'fast':
                    self.robot_controller.set_move_cmd(0.0,
                            self.turn_vel_fast)
                elif self.move_rate == 'slow':
                    self.robot_controller.set_move_cmd(0.0,
                            self.turn_vel_slow)
                elif self.move_rate == 'stop':
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                else:
                    self.robot_controller.set_move_cmd(0.0,
                            self.turn_vel_slow)
            self.robot_controller.publish()
            self.rate.sleep()

    def move_towards(self):
        self.robot_controller.set_move_cmd(0.0, 0.0)
        while self.front_distance > 0.2:
            print(self.front_distance)
            self.robot_controller.set_move_cmd(0.2, 0.0)

    def main(self):
        self.mb_client.wait_for_server()
        rospy.sleep(5)
        self.set_initial_pose()
        self.get_colour()
        while not self.ctrl_c and (self.find_target == False):
            for pose in waypoints:
                goal = self.go_to_waypoint(pose)
                self.mb_client.send_goal(goal)
                self.mb_client.wait_for_result()
                for i in range(100):
                    # rospy.sleep(5)
                    self.check_colour()
               
                print(self.find_target)
                if self.find_target == True:
                    break    
        # while self.front_distance > 0.25:
        print(self.front_distance)
        self.robot_controller.set_move_cmd(0.2, 0.0)
        print("BEACON DETECTED: Beaconing initiated.")
        print("BEACONING COMPLETE: The robot has now stopped.")
            

if __name__ == '__main__':
    search_ob = search_and_beacon()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
