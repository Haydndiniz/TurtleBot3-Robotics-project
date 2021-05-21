#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import time
import math

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
import numpy as np



class task5(object):

    def __init__(self):
        rospy.init_node('object_search')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5000)

        self.m00 = 0
        self.m00_min = 10000000

        self.mask = np.zeros((1080,1920,1), np.uint8)
        self.hsv_img = np.zeros((1080,1920,3), np.uint8)
        self.lowerbound = []
        self.upperbound = []

        self.color_boundaries = {
            "red":    ([-1.8, 217, 100], [3.3, 255, 255]),
            "blue":   ([115, 224, 100],   [130, 255, 255]),
            "yellow": ([28, 180, 100], [32, 255, 255]),
            "green":   ([58, 50, 100], [61, 256, 255]),
            "Turquoise":   ([75, 150, 100], [100, 255, 255]),
            "purple":   ([145, 150, 100], [150, 250, 255])
        }

        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)

        self.robot_odom = TB3Odometry()
        self.arc_angles = np.arange(-96, 96)
        self.all_distance = 0.0
        self.all_angle = 0.0
        self.front_distance = 0.0
        self.front_angle = 0.0
        self.right_distance = 0.0
        self.right_angle = 0.0

        self.initial_pos_x = self.robot_odom.pos_x
        self.initial_pos_y = self.robot_odom.pos_y

        self.init_x = 0.0
        self.init_y = 0.0

        self.init_search = False
        self.start_nav = False

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def scan_callback(self, scan_data):
        close_left_arc = scan_data.ranges[0:3]
        close_right_arc = scan_data.ranges[-3:]
        close_front_arc = np.array(close_left_arc[::-1] + close_right_arc[::-1])
        self.close_front_distance = close_front_arc.min()
        front_left_arc1 = scan_data.ranges[0:5]
        front_right_arc1 = scan_data.ranges[-5:]
        front_arc1 = np.array(front_left_arc1[::-1] + front_right_arc1[::-1])
        front_arc_angle1 = np.arange(-5, 5)

        # find the miniumum object distance within the frontal laserscan arc:
        self.front_distance1 = front_arc1.min()
        self.front_angle1 = front_arc_angle1[np.argmin(front_arc1)]

        left_arc16 = scan_data.ranges[0:16]
        right_arc16 = scan_data.ranges[-15:]
        front_arc16 = np.array(left_arc16[::-1] + right_arc16[::-1])
        front_arc_angle = np.arange(-15, 16)

        self.front_distance = front_arc16.min()
        self.front_angle = front_arc_angle[np.argmin(front_arc16)]

        right_arc15_70 = scan_data.ranges[-70:-15]
        right_side_arc15_70 = np.array(right_arc15_70[::1])
        right_arc_angle15_70 = np.arange(-70,-15)

        self.right_distance = right_side_arc15_70.min()
        self.right_angle15_70 = right_arc_angle15_70[np.argmin(right_side_arc15_70)]


        left_arc40 = scan_data.ranges[0:40]
        right_arc40 = scan_data.ranges[-40:]
        self.left40 = left_arc40
        self.right40 = right_arc40
        front_arc40 = np.array(left_arc40[::-1] + right_arc40[::-1])
        self.front_distance40 = front_arc40
        self.min_distance40 = front_arc40.min()

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.hsv_img = hsv_img

        if len(self.lowerbound) != 0 and len(self.upperbound) != 0:
            self.mask = cv2.inRange(hsv_img, self.lowerbound, self.upperbound)

        m = cv2.moments(self.mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        #if self.m00 > self.m00_min:
        cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(5)

    def get_colour(self):
        for color_name, (lower, upper) in self.color_boundaries.items():
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            mask = cv2.inRange(self.hsv_img, lower_bound, upper_bound)
            if mask.any():
                print("TARGET COLOUR DETECTED: The target beacon is {}".format (color_name))
                self.lowerbound = lower_bound
                self.upperbound = upper_bound

    def rotate(self, deg, speed):
        rospy.sleep(1)
        seconds = math.radians(deg) / speed
        self.robot_controller.set_move_cmd(0.0, speed)
        self.robot_controller.publish()
        time = abs(seconds)
        rospy.sleep(time)
        self.robot_controller.stop()

    def object_avoidance(self, distance):
        
        if self.front_distance <= 0.25 or self.left_distance <= 0.25 or self.right_distance <= 0.25:
            print("avoiding obstacles")
            self.robot_controller.set_move_cmd(-0.2, 0.1)
            self.robot_controller.publish()
        elif self.front_distance > distance and self.left_distance > distance and self.right_distance > distance:
            self.robot_controller.set_move_cmd(0.2, 0)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.left_distance > distance and self.right_distance > distance: 
            if self.left_distance > self.right_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(-0.2, 0.4)
            elif self.left_distance < self.right_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(-0.2, -0.4)#
            self.robot_controller.publish()
        elif self.front_distance > distance and self.left_distance < distance and self.right_distance < distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0.2, 0)#
            self.robot_controller.publish()
        elif self.front_distance > distance and self.left_distance > distance and self.right_distance < distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0, 0.4)
            self.robot_controller.publish()
        elif self.front_distance > distance and self.left_distance < distance and self.right_distance > distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0, -0.4)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.left_distance < distance and self.right_distance > distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0, -0.4)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.left_distance > distance and self.right_distance < distance:
            self.robot_controller.stop()
            self.robot_controller.set_move_cmd(0, 0.4)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.left_distance < distance and self.right_distance < distance:
            if self.left_distance > self.right_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0, 2)
                self.robot_controller.publish()
            elif self.left_distance < self.right_distance:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0, -2)
                self.robot_controller.publish() 

    def safe_to_check_colour(self):
        self.displacement = math.sqrt((self.robot_odom.pos_x - self.initial_pos_x)**2 + (self.robot_odom.pos_y - self.initial_pos_y)**2)
        print(self.robot_odom.pos_x)
        print(self.initial_pos_x)
        print("here")
        if self.displacement > 1.6:
            return True
        else:
            return False

    def check_colour(self):
        if self.m00 > self.m00_min and self.safe_to_check_colour():
            if self.cy >= 560-100 and self.cy <= 560+100:
                if self.move_rate == 'slow':
                    self.move_rate = 'stop'
            else:
                self.move_rate = 'slow'
        else:
            self.move_rate = 'fast'
        if self.move_rate == 'fast':
            self.wall_follow(0.35)
            #print "Turn fast"
        elif self.move_rate == 'slow' and self.safe_to_check_colour() == False:
            if 0 < self.cy and self.cy <= 560-100 and self.safe_to_check_colour() == False:
                self.robot_controller.set_move_cmd(0.1, 0.25)
                self.robot_controller.publish()
                #print "Adjust left"
            elif self.cy > 560+100:
                self.robot_controller.set_move_cmd(0.1, -0.25)
                self.robot_controller.publish()
                #print "Adjust right"
        elif self.move_rate == 'stop' and self.safe_to_check_colour() == False:
            if self.close_front_distance < 0.5 and self.safe_to_check_colour() == False:
                #print "enter zone"
                self.robot_controller.set_move_cmd(0.1, 0.0)
                self.robot_controller.publish()
                time.sleep(2)
                self.robot_controller.stop()
                print "FINAL CHALLENGE COMPLETE: The robot has now stopped."
                self.distance_status = True
            else:
                self.object_avoidance()

        elif self.safe_to_check_colour() == False:
            self.wall_follow(0.35)

        self.robot_controller.publish()

    def wall_follow(self, distance):
        if self.front_distance > distance and self.right_distance > distance-0.05 and self.right_distance < distance+0.03:
            self.robot_controller.set_move_cmd(0.3,0)
            self.robot_controller.publish()
        elif self.front_distance > distance and self.right_distance > distance:
            self.robot_controller.set_move_cmd(0.24, -0.8)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.right_distance > distance:
            self.robot_controller.set_move_cmd(0, -0.6)
            self.robot_controller.publish()
        elif self.front_distance > distance and self.right_distance < distance:
            self.robot_controller.set_move_cmd(0.15, 0.5)
            self.robot_controller.publish()
        elif self.front_distance < distance and self.right_distance < distance:
            self.robot_controller.set_move_cmd(0, 0.6)
            self.robot_controller.publish()
        else:
            self.robot_controller.set_move_cmd(0, -0.5)
            self.robot_controller.publish()

    def move_towards(self):
        while self.front_distance > 0.3:
            self.robot_controller.set_move_cmd(0.2, 0.0)
            self.robot_controller.publish()
        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.robot_controller.publish()

    def main(self):
        print("start time: ", time.time())
        self.rotate(90, -1)  
        self.get_colour() 
        self.rotate(90, 1)   
        while not self.ctrl_c:
            if self.m00 > self.m00_min and self.safe_to_check_colour() == True:
                if self.init_search == False:
                    print "TARGET BEACON IDENTIFIED: Beaconing initiated."
                    self.init_search = True
               
                if self.safe_to_check_colour():
                    self.check_colour()
                    self.move_towards()
                    if self.front_distance1<= 0.4:
                        print "FINAL CHALLENGE COMPLETE: The robot has now stopped."
                        self.ctrl_c = True
            else:
                self.wall_follow(0.35)

        self.rate.sleep()
        print("end time: ", time.time())
if __name__ == '__main__':
    search_ob = task5()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass