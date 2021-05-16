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

waypoints = [[(-1.16, 2.13, 0.0), (0.0, 0.0, -0.0438, 0.999)],
             [(1.90040443106, 1.94064598664, 0.0), (0.0, 0.0, -0.999993175507, 0.00333086304911)] 
            ]

STATES = ["Stage 1", "Stage 2"]

class task4:
    
    def __init__(self):
        
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
        
        rospy.on_shutdown(self.shutdownhook)
        self.mb_client = actionlib.SimpleActionClient('move_base',
                MoveBaseAction)

        self.mb_pub = rospy.Publisher('/initialpose',
                PoseWithCovarianceStamped, queue_size=10)

        self.goal_pose = MoveBaseGoal()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)

        self.odom_sub = rospy.Subscriber('/odom',  Odometry, self.odom_callback)

        rospy.init_node('task4_node')

        self.rate = rospy.Rate(20)

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

    def callback(self, dt):
        scan_data = np.array(dt.ranges)
        arc_distance_left = scan_data[70:110]
        arc_distance_right = scan_data[250:290]
        arc_front_left = scan_data[:30]
        arc_front_right = scan_data[330:]
        front_arc = np.concatenate((arc_front_right, arc_front_left))
        self.min_distance_front = front_arc.min()
        self.distance_left = arc_distance_left.min()
        self.distance_right = arc_distance_right.min()

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

    def timeout(self, initial_time, cancel):
        self.start_timeout = initial_time
        while not cancel:
            if self.start_timeout >= 10:
                return True

    def main_loop(self):
        while not self.ctrl_c:
            if self.pos_x >= -1.46 and self.pos_y >= 0.89 and self.stage_flag == False:
                print("Changing to stage 2...")
                self.stage_flag = True
                self.move.linear.x = 0
                self.move.angular.z = 0
                self.pub.publish(self.move)
                self.state = STATES[1]

            if self.state == STATES[1] and self.stage_flag == True:
                self.mb_client.wait_for_server()
                self.set_initial_pose()
                for pose in waypoints:
                    the_goal = self.go_to_waypoint(pose)
                    print("Going for goal: ", the_goal)
                    self.mb_client.send_goal(the_goal)
                    self.mb_client.wait_for_result()
                    self.timeout(rospy.get_time(), True)

            elif self.state == STATES[0]:
                self.start = rospy.get_time()

                self.error = self.distance_right - 0.3

                self.lerror = self.min_distance_front - 0.2

                #self.integral = self.integral + self.error

                #self.derivative = self.error - self.last_error

                if self.kp*self.error <= 0.6:
                    self.move.angular.z = self.kp*self.error
                elif self.kp*self.error >= -0.6:
                    self.move.angular.z = self.kp*self.error
                elif self.kp*self.error > 0.6:
                    self.move.angular.z = 0.6
                elif self.kp*self.error < -0.6:
                    self.move.angular.z = -0.6

                self.move.linear.x = self.linear_vel
                self.last_error = self.error

                self.end = rospy.get_time()
                self.dt = self.start - self.end

                self.pub.publish(self.move)

                if self.min_distance_front <= 0.4:
                    self.move.linear.x = 0.05
                    self.move.angular.z = 0
                    self.pub.publish(self.move)
                    if self.distance_left > self.distance_right:
                        while self.min_distance_front <= 0.4:
                            self.move.angular.z = 0.6
                            self.pub.publish(self.move)
                        self.move.angular.z = 0
                        self.pub.publish(self.move)
                    elif self.distance_right > self.distance_left:
                        while self.min_distance_front <= 0.4:
                            self.move.angular.z = -0.6
                            self.pub.publish(self.move)
                        self.move.angular.z = 0
                        self.pub.publish(self.move)

                self.rate.sleep()

if __name__ == '__main__':
    task4_instance = task4()
    try:
        task4_instance.main_loop()
    except rospy.ROSInterruptException:
        pass