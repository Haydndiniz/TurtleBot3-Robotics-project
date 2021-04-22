#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(dt):

    distance_1 = 0.5
    distance_2 = 0.5

    if dt.ranges[0]>distance_1 and dt.ranges[20]>distance_2 and dt.ranges[340]>distance_2:
        # move forward
        move.linear.x = 0.2 # go forward (linear velocity) m/s
        move.angular.z = 0.0 # rotate (angular velocity)
    else:
        # turns left
        move.linear.x = 0.0 # go forward (linear velocity) m/s
        move.angular.z = 0.2

    pub.publish(move)

# hi this is a test run for pushing to the remote repository (Marvelous Jibogu)

move = Twist()
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rospy.init_node('task1_node')
sub = rospy.Subscriber("/scan", LaserScan, callback)

rospy.spin() # Loops infinitely until someone stops the program execution
