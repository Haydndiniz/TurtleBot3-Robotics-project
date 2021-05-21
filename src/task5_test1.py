#! /usr/bin/python

import rospy
import actionlib
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

from task5_helper import search_and_beacon

import math

STATES = ["Stage 1", "Stage 2", "Goal Reached"]

class task5:
    
    def __init__(self):
        # rospy.init_node('task5_node')
        self.robot_colour = search_and_beacon()

        self.stage_flag = False
        self.initial_odom_callback = True
        self.beacon_detected = False
        self.state = STATES[0]
        self.kp = 0.6
        self.kpl = 0.4
        self.ki = 1/50
        self.kd = 1/50
        self.linear_vel = 0.25
        self.integral = 0
        self.last_error = 0
        self.derivative = 0
        self.dt = 0.01
        self.move = Twist()
        self.distance_left = 0
        self.distance_right = 0
        self.min_distance_front = 0

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.arc_angles = np.arange(-96, 96)
        self.all_distance = 0.0
        self.all_angle = 0.0
        self.front_distance = 0.0
        self.front_angle = 0.0
        self.right_distance = 0.0
        self.right_angle = 0.0
        self.small_front_distance = 0
        self.small_front_angle = 0
        self.raw_data = []

        
        rospy.on_shutdown(self.shutdownhook)
        self.mb_client = actionlib.SimpleActionClient('move_base',
                MoveBaseAction)

        self.mb_pub = rospy.Publisher('/initialpose',
                PoseWithCovarianceStamped, queue_size=10)

        self.goal_pose = MoveBaseGoal()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        self.odom_sub = rospy.Subscriber('/odom',  Odometry, self.odom_callback)


        self.rate = rospy.Rate(50)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("the task5 node has been initialised...")

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        # publish an empty twist message to stop the robot (by default all 
        # velocities within this will be zero):
        print("stopping task4 node at: {}".format(rospy.get_time()))
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.pub.publish(self.move)

    def laser_callback(self, scan_data):
        close_left_arc = scan_data.ranges[0:3]
        close_right_arc = scan_data.ranges[-3:] 
        close_front_arc = np.array(close_left_arc[::-1] + close_right_arc[::-1])
        self.close_front_distance = close_front_arc.min()

        # front detection
        front_left_arc = scan_data.ranges[0:16]
        front_right_arc = scan_data.ranges[-15:]
        front_arc = np.array(front_left_arc[::-1] + front_right_arc[::-1])
        front_arc_angle = np.arange(-15, 16)

        # find the miniumum object distance within the frontal laserscan arc:
        self.front_distance = front_arc.min()
        self.front_angle = front_arc_angle[np.argmin(front_arc)]
        
        #right detection
        right_arc = scan_data.ranges[-70:-15]
        right_side_arc = np.array(right_arc[::1])
        right_arc_angle = np.arange(-70,-15)

        # find the miniumum object distance within the right laserscan arc:
        self.right_distance = right_side_arc.min()
        self.right_angle = right_arc_angle[np.argmin(right_side_arc)]

        #left detection
        left_arc = scan_data.ranges[16:60]
        left_side_arc = np.array(left_arc[::1])
        left_arc_angle = np.arange(16,60)

        # find the miniumum object distance within the left laserscan arc:
        self.left_distance = left_side_arc.min()
        self.left_angle = left_arc_angle[np.argmin(left_side_arc)]

        left_arc40 = scan_data.ranges[0:40]
        right_arc40 = scan_data.ranges[-40:]
        self.left40 = left_arc40
        self.right40 = right_arc40
        front_arc40 = np.array(left_arc40[::-1] + right_arc40[::-1])
        self.front_distance40 = front_arc40
        self.min_distance40 = front_arc40.min()

    def odom_callback(self, topic_data):            
        self.ori_x = topic_data.pose.pose.orientation.x
        self.ori_y = topic_data.pose.pose.orientation.y
        self.ori_z = topic_data.pose.pose.orientation.z
        self.ori_w = topic_data.pose.pose.orientation.w

        self.pos_x = topic_data.pose.pose.position.x
        self.pos_y = topic_data.pose.pose.position.y
        self.pos_z = topic_data.pose.pose.position.z

        if self.initial_odom_callback:
            self.initial_pos_x = self.pos_x
            self.initial_pos_y = self.pos_y
            self.initial_odom_callback = False

    def safe_to_check_colour(self):
        self.displacement = math.sqrt((self.pos_x - self.initial_pos_x)**2 + (self.pos_y - self.initial_pos_y)**2)
        if self.displacement > 1.6:
            return True
        else:
            return False

    def object_avoidance(self, distance):
        
        if self.front_distance <= 0.25 or self.left_distance <= 0.25 or self.right_distance <= 0.25:
            print("avoiding obstacles")
            self.robot_controller.set_move_cmd(-0.2, 0.1)
            self.robot_controller.publish()
        elif self.front_distance > distance and self.left_distance > distance and self.right_distance > distance:
            self.robot_controller.set_move_cmd(0.2, 0)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.left_distance > distance and self.right_distance > distance: 
            if self.left_distance > self.right_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(-0.2, 0.4)
            elif self.left_distance < self.right_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(-0.2, -0.4)#
            self.robot_controller.publish()
        elif self.front_distance > distance and self.left_distance < distance and self.right_distance < distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0.2, 0)#
            self.robot_controller.publish()
        elif self.front_distance > distance and self.left_distance > distance and self.right_distance < distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0, 0.4)
            self.robot_controller.publish()
        elif self.front_distance > distance and self.left_distance < distance and self.right_distance > distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0, -0.4)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.left_distance < distance and self.right_distance > distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0, -0.4)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.left_distance > distance and self.right_distance < distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0, 0.4)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.left_distance < distance and self.right_distance < distance:
            if self.left_distance > self.right_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0, 2)
                self.robot_controller.publish()
            elif self.left_distance < self.right_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0, -2)
                self.robot_controller.publish() 

    def wall_follow(self, distance):
        
        if self.front_distance > distance and self.right_distance > distance-0.05 and self.right_distance < distance+0.03:
            self.robot_controller.set_move_cmd(0.3,0)
            self.robot_controller.publish()
        elif self.front_distance > distance and self.right_distance > distance:
            self.robot_controller.set_move_cmd(0.24, -0.8)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.right_distance > distance:
            self.robot_controller.set_move_cmd(0, -0.6)
            self.robot_controller.publish()
        elif self.front_distance > distance and self.right_distance < distance:
            self.robot_controller.set_move_cmd(0.15, 0.5)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.right_distance < distance:
            self.robot_controller.set_move_cmd(0, 0.6)
            self.robot_controller.publish()
        else:
            self.robot_controller.set_move_cmd(0, -0.5)
            self.robot_controller.publish()

    def check_colour(self):
        if self.robot_colour.m00 > self.robot_colour.m00_min and self.safe_to_check_colour() == False:           
            if self.cy >= 560-100 and self.cy <= 560+100:
                if self.move_rate == 'slow':
                    self.move_rate = 'stop'                             
            else:
                self.move_rate = 'slow'
        else:
            self.move_rate = 'fast'                
        if self.move_rate == 'fast':
            self.wall_follow(0.35)
        elif self.move_rate == 'slow' and self.safe_to_check_colour() == True:
            if 0 < self.cy and self.cy <= 560-100:
                self.robot_controller.set_move_cmd(0.1, 0.25)
                self.robot_controller.publish()
            elif self.cy > 560+100:
                self.robot_controller.set_move_cmd(0.1, -0.25)
                self.robot_controller.publish()
        elif self.move_rate == 'stop' and self.safe_to_check_colour() == True:
            if self.close_front_distance < 0.5 and self.safe_to_check_colour() == True:
                self.robot_controller.set_move_cmd(0.1, 0.0)
                self.robot_controller.publish()
                time.sleep(2)
                self.robot_controller.stop()
                print "BEACONING COMPLETE: The robot has now stopped."
                self.beacon_detected = True
            else:
                self.object_avoidance()  
        elif self.safe_to_check_colour() == False:
            self.wall_follow(0.35)                 
        self.robot_controller.publish()

    def main_loop(self):
        print("start time: ", time.time())
        self.robot_colour.rotate(90, 1)
        self.robot_colour.get_colour()
        self.robot_colour.rotate(90, -1)      
        while not self.ctrl_c:
            if self.robot_colour.m00 > self.robot_colour.m00_min and self.safe_to_check_colour(): 
                if self.beacon_detected == False:                    
                    print "BEACON IDENTIFIED: Beaconing initiated." 
                    self.beacon_detected = True   
                self.check_colour()
                if self.robot_colour.move_towards():
                    break
            else:
                self.wall_follow(0.35)
   
            self.rate.sleep()
        print("end time: ", time.time())

if __name__ == '__main__':
    task4_instance = task5()
    try:
        task4_instance.main_loop()
    except rospy.ROSInterruptException:
        pass