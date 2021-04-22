#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(dt):

    move.linear.x = 0.2 # go forward (linear velocity)
    move.angular.z = 0.0 # rotate (angular velocity)

    pub.publish(move)


move = Twist()
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rospy.init_node('task1_node')
sub = rospy.Subscriber("/scan", LaserScan, callback)

rospy.spin() # Loops infinitely until someone stops the program execution
