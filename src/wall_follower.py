#! /usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf import transformations
from nav_msgs.msg import Odometry
import math

class wall_follower:

    def __init__(self):

        self.regions_right = 0
        self.regions_fright = 0
        self.regions_front = 0
        self.regions_fleft = 0
        self.regions_left = 0

        self.state = 0

        self.state_dict = {
            0: "Finding wall",
            1: "Turning left",
            2: "Following wall"
        }

        self.move = Twist()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        rospy.init_node('wall_follower_node')

        self.rate = rospy.Rate(50)

        self.ctrl_c = False

        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("the wall_follower node has been initialised...")

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        print("stopping wall_follower node at: {}".format(rospy.get_time()))
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.pub.publish(self.move)

    def laser_callback(self, data):
        scan_data = np.array(data.ranges)
        self.regions_right = scan_data[248:293].min()
        self.regions_fright = scan_data[293:338].min()
        self.regions_front = np.concatenate((scan_data[338:359], scan_data[0:23])).min()
        self.regions_fleft = scan_data[23:67].min()
        self.regions_left = scan_data[67:112].min()

        self.take_action()

    def change_state(self, new_state):
        if new_state is not self.state:
            print('Wall follower - [%s] - %s' % (new_state, self.state_dict[new_state]))
            self.state = new_state


    def take_action(self):
        
        state_description = ''
        
        distance = 0.425
        
        if self.regions_front > distance and self.regions_fleft > distance and self.regions_fright > distance:
            state_description = 'case 1 - nothing'
            self.change_state(0)
        elif self.regions_front < distance and self.regions_fleft > distance and self.regions_fright > distance:
            state_description = 'case 2 - front'
            self.change_state(1)
        elif self.regions_front > distance and self.regions_fleft > distance and self.regions_fright < distance:
            state_description = 'case 3 - fright'
            self.change_state(2)
        elif self.regions_front > distance and self.regions_fleft < distance and self.regions_fright > distance:
            state_description = 'case 4 - fleft'
            self.change_state(0)
        elif self.regions_front < distance and self.regions_fleft > distance and self.regions_fright < distance:
            state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif self.regions_front < distance and self.regions_fleft < distance and self.regions_fright > distance:
            state_description = 'case 6 - front and fleft'
            self.change_state(1)
        elif self.regions_front < distance and self.regions_fleft < distance and self.regions_fright < distance:
            state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
        elif self.regions_front > distance and self.regions_fleft < distance and self.regions_fright < distance:
            state_description = 'case 8 - fleft and fright'
            self.change_state(0)
        else:
            state_description = 'unknown case'

        print(state_description)

    def find_wall(self):
        self.move.linear.x = 0.15
        self.move.angular.z = -0.0
        return self.move

    def turn_left(self):
        self.move.angular.z = 1.0
        return self.move

    def follow_wall(self):
        self.move.linear.x = 0.15
        return self.move

    def main_loop(self):

        while not self.ctrl_c:

            if self.state == 0:
                self.move = self.find_wall()
            elif self.state == 1:
                self.move = self.turn_left()
            elif self.state == 2:
                self.move = self.follow_wall()
                pass
            else:
                print("Unknown state.")

            self.pub.publish(self.move)

            self.rate.sleep()

if __name__ == '__main__':
    wall_follower_instance = wall_follower()
    try:
        wall_follower_instance.main_loop()
    except rospy.ROSInterruptException:
        pass




