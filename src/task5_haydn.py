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

waypoints = [[(1.09, 2.05, 0.0), (0.0, 0.0, -0.0197, -0.9998)],
             [(0.684, -1.487, 0.0), (0.0, 0.0,-0.002, 0.999)],
             [(1.62, 0.436, 0.0), (0.0, 0.0, 0.999, -0.010)],          
             [(1.900, 1.940, 0.0), (0.0, 0.0, -0.999, 0.0033)] 
            ]

STATES = ["Stage 1", "Stage 2", "Goal Reached"]

class task4:
    
    def __init__(self):
        rospy.init_node('task5_node')

        
        self.stage_flag = False
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

        rospy.loginfo("the task4 node has been initialised...")

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

    def wall_follow(self, distance):
        if self.front_distance > distance and self.right_distance > distance:
            self.robot_controller.set_move_cmd(0.2, -0.8)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.right_distance > distance:
            self.robot_controller.set_move_cmd(0, -0.5)
            self.robot_controller.publish()
        elif self.front_distance > distance and self.right_distance < distance:
            self.robot_controller.set_move_cmd(0.1, 0.5)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.right_distance < distance:
            self.robot_controller.set_move_cmd(0, 0.5)
            self.robot_controller.publish()
        else:
            self.robot_controller.set_move_cmd(0, -0.5)
            self.robot_controller.publish()

    def main_loop(self):
        while not self.ctrl_c:
            self.wall_follow(0.33)

           
            self.rate.sleep()

if __name__ == '__main__':
    task4_instance = task4()
    try:
        task4_instance.main_loop()
    except rospy.ROSInterruptException:
        pass