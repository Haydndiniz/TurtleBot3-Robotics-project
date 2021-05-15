#! /usr/bin/python

import rospy
import actionlib
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

goal = [(1.90040443106, 1.94064598664, 0.0), (0.0, 0.0, -0.999993175507, 0.00333086304911)]

class task4:
    
    def __init__(self):

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.mb_client = actionlib.SimpleActionClient('move_base',
                MoveBaseAction)

        self.mb_pub = rospy.Publisher('/initialpose',
                PoseWithCovarianceStamped, queue_size=10)

        self.goal_pose = MoveBaseGoal()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

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

    """def set_initial_pose(self):
        checkpoint = PoseWithCovarianceStamped()
        checkpoint.pose.pose.position.x = -2.088
        checkpoint.pose.pose.position.y = -2.027
        checkpoint.pose.pose.position.z = 0.0
        [x, y, z, w] = quaternion_from_euler(0.0, 0.0, 1.572)
        checkpoint.pose.pose.orientation.x = x
        checkpoint.pose.pose.orientation.y = y
        checkpoint.pose.pose.orientation.z = z
        checkpoint.pose.pose.orientation.w = w
        self.mb_pub.publish(checkpoint)"""

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

    def main_loop(self):
        while not self.ctrl_c:
            self.mb_client.wait_for_server()
            #self.set_initial_pose()
            the_goal = self.go_to_waypoint(goal)
            print("Going for goal: ", the_goal)
            self.mb_client.send_goal(the_goal)
            self.mb_client.wait_for_result()

if __name__ == '__main__':
    task4_instance = task4()
    try:
        task4_instance.main_loop()
    except rospy.ROSInterruptException:
        pass