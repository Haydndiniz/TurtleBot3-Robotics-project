#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class task1:

    def callback(dt):

        if odom_in:
            reset_old_pos()
            odom_in = False

        rospy.Timer(rospy.Duration(2), reset_old_pos())

        min_permitted_distance = 0.4
        scan_data = np.array(dt.ranges)
        scan_arc_left = scan_data[:30]
        scan_arc_right = scan_data[330:]
        min_distance_left = scan_arc_left.min()
        min_distance_right = scan_arc_right.min()

        if trapped():

            move.linear.x = 0.0 # go forward (linear velocity) m/s
            move.angular.z = 0.0 # rotate (angular velocity)

            move.linear.x = -0.2 # go forward (linear velocity) m/s

            rospy.sleep(3)

            move.linear.x = 0.0 # go forward (linear velocity) m/s

            move.angular.z = 0.2 # rotate (angular velocity)

            rospy.sleep(7.85)

            move.angular.z = 0.0 # rotate (angular velocity)

        else:    
            
            if min_distance_left <= min_permitted_distance:
                # turns right
                print("Object detected to the left. Turning right...")
                move.linear.x = 0.0 # go forward (linear velocity) m/s
                move.angular.z = -0.2 # rotate (angular velocity)
            else:
                # moves forward
                print("Moving forward...")
                move.linear.x = 0.2 # go forward (linear velocity) m/s
                move.angular.z = 0.0

            if min_distance_right <= min_permitted_distance:
                # turns left
                print("Object detected to the right. Turning left...")
                move.linear.x = 0.0 # go forward (linear velocity) m/s
                move.angular.z = 0.2 # rotate (angular velocity)
            else:
                # moves forward
                print("Moving forward...")
                move.linear.x = 0.2 # go forward (linear velocity) m/s
                move.angular.z = 0.0

            pub.publish(move)


    def reset_old_pos():
        old_pos_x = position_x
        old_pos_y = position_y

    def trapped():

        box_area.x1 = old_pos_x-0.1
        box_area.x2 = old_pos_x+0.1
        box_area.y1 = old_pos_y-0.1
        box_area.y2 = old_pos_y+0.1

        if position_x >= box_area.x1 and position_x <= box_area.x2 and position_y >= box_area.y1 and  position_y <= box_area.y2:
            seconds_trapped += 0.2
        else:
            seconds_trapped = 0
            old_pos_x = position_x
            old_pos_y = position_y

        if seconds_trapped >= 5:
            print("Robot trapped. Peforming untrapping manoeuvre...")
            return True
        else:
            return False

    def odom_callback(odom_data):

        position_x = odom_data.pose.pose.position.x
        position_y = odom_data.pose.pose.position.y

        odom_in = True

        orientation_x = odom_data.pose.pose.orientation.x
        orientation_y = odom_data.pose.pose.orientation.y
        orientation_z = odom_data.pose.pose.orientation.z
        orientation_w = odom_data.pose.pose.orientation.w

        # global (roll, pitch, yaw) = euler_from_quaternion([orientation_x, orientation_y, orientation_z, orientation_w],'sxyz')

    

    rospy.spin() # Loops infinitely until someone stops the program execution

    def __init__(self):

        move = Twist()
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        sub = rospy.Subscriber("/scan", LaserScan, callback)
        odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)

        rospy.init_node('task1_node')

        seconds_trapped = 0

        position_x = 0.0
        position_y = 0.0

        old_pos_x = 0.0
        old_pos_y = 0.0

        odom_in = False

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:
            self.pub.publish(move)

if __name__ == '__main__':
    main_instance = task1()
    try:
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass