#! /usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class task4:
    
    def __init__(self):
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

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)

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

    def main_loop(self):
        while not self.ctrl_c:

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