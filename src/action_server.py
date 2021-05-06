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
        self.rate = rospy.Rate(200)

        self.scanSub = rospy.Subscriber('/scan', LaserScan, self.callback_function)

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        # Define zones
        # [x1,x2,y1,y2]
        # self.zones = [[-1.0,-2.3,-1.0,-2.3],[-1.0,-2.3,1.0,-1.0],[-1.0,-2.3,2.3,1.0],
        #                 [1.0,-1.0,-1.0,-2.3], [1.0, -1.0,1.0,-1.0],[1.0,-1.0,2.3,1.0],
        #                 [2.3,1.0,-1.0,-2.3 ],[2.3,1.0,1.0,-1.0],[2.3,1,2.3,1.0]]

        # self.visited_zones =[]

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
        self.closest_distance = 1.0

        # angles in rad/s
        self.front_angle = 0.0
        self.front_wide_angle = 0.0
        self.right_angle = 0.0
        self.left_angle = 0.0
        self.back_angle = 0.0
        self.closest_angle = 0.0
      
        # Initial start time 
        self.startTime = rospy.get_rostime()
        print(self.startTime.secs)

    def callback_function(self, scan_data):
         
        # pano_arc = scan_data.ranges[0:359]
        # closest_arc = np.arange(pano_arc[::1])
        # # Set distance to closest obstacle(360 degrees)
        # self.closest_distance = closest_arc.min()
        
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
        front_left_arc = scan_data.ranges[0:16]
        front_right_arc = scan_data.ranges[-15:]
        front_arc = np.array(front_left_arc[::-1] + front_right_arc[::-1])
        front_arc_angle = np.arange(-15, 16)
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
        right_arc = scan_data.ranges[314:345]
        right_side_arc = np.array(right_arc[::1])
        right_arc_angle = np.arange(314,345)

        # Set distance and bearing of closest obstacle to the right of bot
        self.right_distance = right_side_arc.min()
        self.right_angle = right_arc_angle[np.argmin(right_side_arc)]

        # Left obstacles, 30 degrees 
        left_arc = scan_data.ranges[14:44]
        left_side_arc = np.array(left_arc[::1])
        left_arc_angle = np.arange(14,44)

        # Set distance and bearing of closest obstacle to the left of bot
        self.left_distance = left_side_arc.min()
        self.left_angle = left_arc_angle[np.argmin(left_side_arc)]

    def collision_detect(self, min_d_side):
        global goal_accomplished
        if self.front_round_distance <= min_d_side or self.back_round_distance <= min_d_side:
            self.robot_controller.stop()
            success = False
            goal_accomplished = False
            print(self.closest_distance)
            print("---------------------------------Collision detected")
            rospy.loginfo('Cancelling the move.')
            self.actionserver.set_preempted()
            sys.exit()

    def move(self, lin_v, ang_v):
        
        # set minimum distance
        min_d_side = 0.14
        min_d_corner = 0.2

        self.robot_controller.set_move_cmd(lin_v, ang_v) # (velocity, rad/s)
        self.robot_controller.publish()
        self.collision_detect(min_d_side)

    # def check_zone(self):
        
    #     self

    # def check_stuck(self):
    #     old_x = 0   
    #     old_y = 0   

    #     if (old_x-0.05) < self.robot_odom.posx < (old_x+0.05) and (old_y-0.05) < self.robot_odom.posy < (old_y+0.051):
    #         self.move(-0.3, 0.2)
    #         self.sleep(1)
    #         print("stuck turning around")
    #     old_x = self.robot_odom.posx
    #     old_y = self.robot_odom.posy


    def action_server_launcher(self, goal):
        success = True

        lin_v1 = 0.4
        lin_v2 = 0.2

        if goal.fwd_velocity <= 0 or goal.fwd_velocity >= 5:
            success = False
            print("Invalid velocity.  Select a value between 0 and 0.4 m/s.")
        if goal.approach_distance <= 0.11 or goal.approach_distance > 0.9:
            success = False
            print("Invalid approach distance: Select a value between 0.12 and 0.9")

        if not success:
            self.actionserver.set_aborted()
            return

        goal_accomplished = False
        max_time =  rospy.Time.now() + rospy.Duration(95)
        run_time = rospy.Time.now()
        while not goal_accomplished and ( run_time <= max_time):
            run_time = rospy.Time.now() 
            # self.check_stuck    

            if self.front_distance > goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance > goal.approach_distance:
                # Turn left if disance to obstacle on left is greater
                if self.left_distance > self.right_distance:
                   # Turn left if disance to obstacle on left is greater 
                    if self.left_distance > self.right_distance:
                        self.robot_controller.stop()
                        self.move(lin_v1, 0.2)
                        print("----Veering Left")
                    # Turn right if disance to obstacle on right is greater
                    elif self.left_distance < self.right_distance:
                        self.robot_controller.stop()
                        self.move(lin_v1, -0.2)
                        print("----Veering right")
              
            # obstacle within min goal distance(defined in action client), so turn left or right
            elif self.front_distance < goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance > goal.approach_distance: 
                # Turn left if disance to obstacle on left is greater 
                if self.left_distance > self.right_distance:
                    self.robot_controller.stop()
                    self.move(0.2, 2)
                    print("----Turning Left")
                # Turn right if disance to obstacle on right is greater
                elif self.left_distance < self.right_distance:
                    self.robot_controller.stop()
                    self.move(0.2, -2)
                    print("----Turning right")

            # there is space infront, but obstacles on the left and right, move forward
            elif self.front_distance > goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance < goal.approach_distance:
                self.robot_controller.stop()
                self.move(lin_v1, 0)
                print("----Move forward, obstacles left and right")

            # obstacle on right, so turn left
            elif self.front_distance > goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance < goal.approach_distance:
                self.robot_controller.stop()
                self.move(lin_v2, 0.4)
                self.feedback.current_distance_travelled = (math.sqrt((self.robot_odom.posx - self.x0)**2 + (self.robot_odom.posy- self.y0)**2))
                self.actionserver.publish_feedback(self.feedback)
                print("----turn left and move forward")

           # obstacle on left, so turn right
            elif self.front_distance > goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance > goal.approach_distance:
                self.robot_controller.stop()
                self.move(lin_v2, -0.4)
                print("----turn right and move forward")
            
            # obstacle in front and on left, so turn right
            elif self.front_distance < goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance > goal.approach_distance:
                self.robot_controller.stop()
                self.move(0.1, -0.9)
                print("----turn right and slow down")
            
            # obstacle in front and on right, so turn left
            elif self.front_distance < goal.approach_distance and self.left_distance > goal.approach_distance and self.right_distance < goal.approach_distance:
                self.robot_controller.stop()
                self.move(0.1, 0.9)
                print("----turn left and slow down")

            # obstacle in front and on both sides
            elif self.front_distance < goal.approach_distance and self.left_distance < goal.approach_distance and self.right_distance < goal.approach_distance:
                # stop, move back slowly, then turn around to the left
                if self.left_distance > self.right_distance:
                    self.robot_controller.stop()
                    self.move(-lin_v1 ,0)
                    self.move(0, 2)

                # stop, move back slowly, then turn around to the right
                elif self.left_distance < self.right_distance:
                    self.robot_controller.stop()
                    self.move(-lin_v1, 0)
                    self.move(0, -2)
                print("----obstacles on all sides, turning around")
            # Stop and turn
            else:
                self.move(0, -2)
               
        if success:
            if goal_accomplished:
                print("Goal accomplished, stopping...")
            else:
                print("time limit reached, stopping")
            rospy.loginfo('Stop successfully.')
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
            
                
if __name__ == '__main__':
    rospy.init_node('move_action_server')
    ActionServer()
    rospy.spin()
