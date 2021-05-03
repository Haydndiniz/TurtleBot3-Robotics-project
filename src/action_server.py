#! /usr/bin/python

# Import the core modules
import rospy
import actionlib
# Import ROS message types:
from com2009_actions.msg import SearchFeedback, SearchResult, SearchAction
from sensor_msgs.msg import LaserScan

from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
import math
import numpy as np
import sys

class ActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/move_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()
        self.rate = rospy.Rate(100)

        self.scanSub = rospy.Subscriber('/scan', LaserScan, self.callback_function)

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        # Define start pose 
        self.x0 = 0.0
        self.y0 = 0.0

        # init twist params
        # Distance in m
        self.front_distance = 0.0
        self.front_wide_distance = 0.0
        self.right_distance = 0.0
        self.left_distance = 0.0
        self.back_distance = 0.0
        self.front_round_distance = 1.0
        self.back_round_distance = 1.0

        # angles in rad/s
        self.front_angle = 0.0
        self.front_wide_angle = 0.0
        self.right_angle = 0.0
        self.left_angle = 0.0
        self.back_angle = 0.0
      
        # Initial start time 
        self.startTime = rospy.get_rostime()
        print(self.startTime.secs)

    def callback_function(self, scan_data):
        
        back_arc = scan_data.ranges[90:270]
        back_side_arc = np.array(back_arc[::1])
        back_round_angle = np.arange(90,270)
        # Set distance to closest rear obstacle(rear 180 degrees)
        self.back_round_distance = back_side_arc.min()

        front_round_left = scan_data.ranges[0:90]
        front_round_right = scan_data.ranges[-90:]
        front_round_arc = np.array(front_round_right[::-1] + front_round_left[::-1])
        # Set distance to closest front obstacle(front 180 degrees)
        self.front_round_distance = front_round_arc.min()

        # front obstacles, 30 degrees (15 left, 15 right)
        front_left_arc = scan_data.ranges[0:15]
        front_right_arc = scan_data.ranges[-15:]
        front_arc = np.array(front_left_arc[::-1] + front_right_arc[::-1])
        front_arc_angle = np.arange(-15, 15)
        # Set distance and bearing of closest obstacle in front of bot
        self.front_distance = front_arc.min()
        self.front_angle = front_arc_angle[np.argmin(front_arc)]

        # front obstacles, 90 degrees (15 left, 15 right)
        front_left_arc = scan_data.ranges[0:30]
        front_right_arc = scan_data.ranges[-30:]
        front_arc = np.array(front_left_arc[::-1] + front_right_arc[::-1])
        front_arc_angle = np.arange(-30, 30)
        # Set distance and bearing of closest obstacle in front of bot
        self.front_wide_distance = front_arc.min()
        self.front_wide_angle = front_arc_angle[np.argmin(front_arc)]
        
        # Right obstacles, 30 degrees 
        right_arc = scan_data.ranges[316:346]
        right_side_arc = np.array(right_arc[::1])
        right_arc_angle = np.arange(316,346)

        # Set distance and bearing of closest obstacle to the right of bot
        self.right_distance = right_side_arc.min()
        self.right_angle = right_arc_angle[np.argmin(right_side_arc)]

        # Left obstacles, 30 degrees 
        left_arc = scan_data.ranges[16:46]
        left_side_arc = np.array(left_arc[::1])
        left_arc_angle = np.arange(16,46)

        # Set distance and bearing of closest obstacle to the left of bot
        self.left_distance = left_side_arc.min()
        self.left_angle = left_arc_angle[np.argmin(left_side_arc)]

    def collision_detect(self, min_d):
        global go
        if self.front_round_distance <= min_d:
            print("Front collision")
            rospy.loginfo('Cancelling the move.')
            self.actionserver.set_preempted()
            # stop the robot:
            self.robot_controller.stop()
            success = False
            go = False
            sys.exit()
        elif self.back_round_distance <= min_d:
            print("Rear collision")
            rospy.loginfo('Cancelling the move.')
            self.actionserver.set_preempted()
            # stop the robot:
            self.robot_controller.stop()
            success = False
            go = False
            sys.exit()

    def can_turn(self):
        if self.front_round_distance <= 0.2:
            print("No space in front, moving...")
           
        elif self.back_round_distance <= 0.2:
            print("No space in back, moving...")
        else
         go:
          

    def action_server_launcher(self, goal):
        success = True
        x = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))

        # check goal
        if goal.fwd_velocity <= 0 or goal.fwd_velocity >= 5:
            success = False
            print("Invalid velocity.  Select a value between 0 and 0.4 m/s.")
        if goal.approach_distance <= 0.11 or goal.approach_distance > 0.9:
            success = False
            print("Invalid approach distance: Select a value between 0.12 and 0.9")

        if not success:
            self.actionserver.set_aborted()
            return

        # set minimum distance
        min_d = 0.2

        go = True
        while go:
            self.collision_detect(min_d)

            if self.front_distance > goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance > goal.approach_distance:
                # Turn left if disance to obstacle on left is greater
                if self.left_distance > self.right_distance:
                    self.robot_controller.set_move_cmd(0.4, 1) # (velocity, rad/s)
                    self.robot_controller.publish()
                    self.collision_detect(min_d)
                # Turn right if disance to obstacle on right is greater
                elif self.left_distance < self.right_distance:
                    self.robot_controller.set_move_cmd(0.4, -1)
                    self.robot_controller.publish()
                    self.collision_detect(min_d)
                # populate the feedback message and publish it:
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
                self.collision_detect(min_d)
           
            # obstacle within min goal distance(defined in action client), so turn left or right
            elif self.front_distance < goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance > goal.approach_distance: 
                # Turn left if disance to obstacle on left is greater
                if self.left_distance > self.right_distance:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0.1, 2)
                    self.robot_controller.publish()
                    self.collision_detect(min_d)
                    print("----Turning Left")
                # Turn right if disance to obstacle on right is greater
                elif self.left_distance < self.right_distance:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0.1, -2)
                    self.robot_controller.publish()
                    self.collision_detect(min_d)
                    print("----Turning right")
                # populate the feedback message and publish it:
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
            # there is space infront, but obstacles on the left and right, move forward
            elif self.front_distance > goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance < goal.approach_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.4, 0)
                self.robot_controller.publish()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
                self.collision_detect(min_d)
                print("----Move forward, obstacles left and right")
            # obstacle on right, so turn left
            elif self.front_distance > goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance < goal.approach_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.4, 0.1)
                self.robot_controller.publish()
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
                self.collision_detect(min_d)
                print("----turn left and move forward")
           # obstacle on left, so turn right
            elif self.front_distance > goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance > goal.approach_distance:
                #self.robot_controller.set_move_cmd(-1, 0)
                #self.robot_controller.publish()
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.4, -0.1)
                self.robot_controller.publish()
                self.collision_detect(min_d)
                print("----turn right and move forward")
            
            # obstacle in front and on left, so turn right
            elif self.front_distance < goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance > goal.approach_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.1, -0.8)
                self.robot_controller.publish()
                self.collision_detect(min_d)
                print("----turn right and slow down")
            
            # obstacle in front and on right, so turn left
            elif self.front_distance < goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance < goal.approach_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.1, 0.8)
                self.robot_controller.publish()
                self.collision_detect(min_d)
                print("----turn left and slow down")
            # obstacle in front and on both sides
            elif self.front_distance < goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance < goal.approach_distance:
                # stop, move back slowly, then turn around to the left
                if self.left_distance > self.right_distance:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(-0.4, 0)
                    self.robot_controller.publish()
                    self.robot_controller.set_move_cmd(0, 2)
                    self.robot_controller.publish()
                    self.collision_detect(min_d)
                # stop, move back slowly, then turn around to the right
                elif self.left_distance < self.right_distance:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(-0.4, 0)
                    self.robot_controller.publish()
                    self.robot_controller.set_move_cmd(0, -2)
                    self.robot_controller.publish() 
                    self.collision_detect(min_d)
                # populate the feedback message and publish it:
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
                self.collision_detect(min_d)
                print("----obstacles on all sides, turning around")
            else:
                print("----unknown scenario")
               
        if success:
            rospy.loginfo('Stop sucessfully.')
            self.result.total_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
            self.result.closest_object_distance = self.front_distance
            self.result.closest_object_angle = self.front_angle
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
            
                
if __name__ == '__main__':
    rospy.init_node('move_action_server')
    ActionServer()
    rospy.spin()