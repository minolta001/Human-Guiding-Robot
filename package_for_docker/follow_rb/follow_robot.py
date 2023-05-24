#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('follow_rb')
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import sys
import time
import numpy as np
import math
import cv2
import pyrealsense2.pyrealsense2 as rs

from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

min_val = np.inf  # minimum reading value from scaner
front_min = np.inf
desire_theta = -np.inf
ang_diff_to_wall = -np.inf

depth_dist = np.inf
x_drift = np.inf
y_drift = np.inf
detect_state = False    # if detect a target color, True. Else, False

TwistMsg = Pose2D

states = ['collide', 'follow', 'search', 'calibrate']



class Robot(threading.Thread):
    def __init__(self, rate):
        super(Robot, self).__init__()
        self.publisher = rospy.Publisher('/drive_cmd', TwistMsg, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()

        self.done = False

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")
        
    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn

        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()


    def stop(self):
        self.done = True
        self.update(0,0,0,0,0,0)
        self.join()
    
    def run(self):
        twist_msg = TwistMsg()
        twist = twist_msg
        

        while not self.done:
            self.condition.acquire()

            self.condition.wait(None)

            twist.x = self.x * self.speed
            twist.y = self.y * self.speed
            twist.theta = self.th * self.turn

            self.condition.release()

            # publish
            self.publisher.publish(twist_msg)

        # stop 
        twist.x = 0
        twist.y = 0
        twist.theta = 0
        self.publisher.publish(twist_msg)

# Vision using D435
# Return depth distance toward target color
# Return drift pixel distance toward the center
def vision():
    global depth_dist, x_drift, y_drift, detect_state

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    pipeline.start(config)

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Convert color image to HSV color space
            hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

            # Define lower and upper bounds for yellow color
            lower_red = np.array([20, 100, 100])
            upper_red = np.array([40, 255, 255])

            yel_mask = cv2.inRange(hsv_image, lower_red, upper_red)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            yel_mask = cv2.morphologyEx(yel_mask, cv2.MORPH_OPEN, kernel)
            yel_mask = cv2.morphologyEx(yel_mask, cv2.MORPH_CLOSE, kernel)

            # Find contours in the red mask
            contours, _ = cv2.findContours(yel_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
             
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 2000:  # Filter small contours

                    detect_state = True

                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)

                    # Get depth at the center of the circle
                    depth = depth_image[int(y), int(x)]

                    # Convert depth to meters using the depth scale
                    depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
                    depth = depth * depth_scale 

                    # Display the circle and its distance
                    cv2.circle(color_image, center, radius, (0, 255, 0), 2)
                    #cv2.putText(color_image, f"Distance: {depth:.2f} meters", (int(x) - radius, int(y) - radius - 10),
                                #cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    

                    # calculate pixel distance toward the center of image
                    #M = cv2.moments(contour)
                    #cx = int(M["m10"] / M["m00"])
                    #cy = int(M["m01"] / M["m00"])

                    image_width, image_height = color_image.shape[:2]
                    center_x = image_width // 2
                    center_y = image_height // 2
                    #distance = np.sqrt((center_x - int(x)) ** 2 + (center_y - int(y)) ** 2)
                    x_dist = int(x) - center_x - 85
                    y_dist = int(y) - center_y + 85

                    cv2.putText(color_image, f"Depth: {depth:.2f}, Drift X: {x_dist:.2f}, Drift Y: {y_dist:.2f}", (int(x) - radius, int(y) - radius - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    

                    # update required data
                    x_drift = x_dist
                    y_drift = y_dist
                    depth_dist = depth


            if cv2.countNonZero(yel_mask) > 0:
                detect_state = True
            else:
                detect_state = False
            
            # Display the color image with detected circles and distances
            # cv2.imshow("Color Image", color_image)
            cv2.waitKey(1)

    finally:
        pipeline.stop()
        #cv2.destroyAllWindows()

       
def scan_callback(scan_data):
    global min_val, desire_theta, ang_diff_to_wall, front_min


    desire_dist_to_wall = 0.5
    desire_forward_dist = 0.4

    ranges = scan_data.ranges
    min_val = min(ranges)

    min_idx = ranges.index(min_val)

    front10 = ranges[:10]
    back10 = ranges[-10:]
    
    front_ranges = front10 + back10 # the scan readings from heading 30 degree
    front_min = min(front_ranges)

    # calculate the theta difference to the desire theta (radian)
    ang_to_wall = min_idx * scan_data.angle_increment
    ang_diff_to_wall = 2 * math.pi - ang_to_wall

    # distance to the wall minus the desire distance between wall and robot
    dist_to_wall_with_gap = min_val - desire_dist_to_wall
    # desire angle
    desire_theta = math.atan2(desire_forward_dist, dist_to_wall_with_gap)
    # angle error need to be correct
    err_theta = abs(ang_diff_to_wall - desire_theta)

    if(err_theta > math.pi):
        err_theta = 2 * math.pi - err_theta

def turn_left_or_right(desire_theta, ang_diff_to_wall):
    if(desire_theta < 0):
        desire_theta = 2 * math.pi + desire_theta
    
    if(ang_diff_to_wall < 0):
        ang_diff_to_wall = 2 * math.pi + ang_diff_to_wall
    
    # several scenarios are discussed here. A better desciption should be added
    if(abs(ang_diff_to_wall - desire_theta) < (math.pi / 2)):   # faster to approach desire theta line on the right side, or on the left side? -> right side
        if(ang_diff_to_wall - desire_theta > 0):    # desire theta larger or ang_diff larger?  -> ang_diff_to_wall larger, so turn right
            return -1
        else:
            return 1                                # ang_diff_to_wall smaller. The closest desire_theta line on robot left side. Turn left.
    else:                                                       # faster to approach desire theta line on the left side
        if(ang_diff_to_wall - desire_theta > 0):    # ang_diff_to_wall larger, so turn left
            return 1
        else:
            return -1                               # ang_diff_to_wall smaller, turn right


# determine the robot current state
def current_state():
    if not detect_state:
        return "search"
    else:
        if(abs(x_drift) >= 200):
            return "calibrate"

        if(min_val <= 0.3):
            return "collide"
    
        if(front_min <= 0.3):
            return "collide"
        else:
            return 'follow'
 

# return linear velocity and angular velocity for robot controling
def controller():
    # check if rotation is required
    state = current_state()
    if state == "search":
        return 0.0, 0.0, 0.0, -2.5, state
    elif state == "collide":
        return 0.0, 0.0, 0.0, 0.0, state

    elif(state == "calibrate"):
        return 0.0, 0.0, 0.0, 0.0, state    # calibration might be shaking, so we don't set a single rotate direction
    else:
        return 0.0, 1.5, 0.0, 0.0, state
    


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)
    

if __name__ == "__main__":
    
    rospy.init_node('follow_robot')
    speed = 1.0
    turn = 1.5
    
    TwistMsg = Pose2D

    pub_thread = Robot(0.0)

    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0

    # subscribe laser scanner
    rospy.Subscriber('/scan', LaserScan, scan_callback) 

    led_pub = rospy.Publisher('/color_cmd', Int32, queue_size=1)
    led_msg = Int32()

    try:
        vision_thread = threading.Thread(target=vision)
        vision_thread.start()
   
        pub_thread.wait_for_subscribers()
        pub_thread.update(0,0,0,0,0,0)
        pub_thread.start()

        input("Press Enter to Continue\n")
        while(1):
            x, y, z, th, state = controller()

            if state == "calibrate":
                led_msg = 0x0000FF
                led_pub.publish(led_msg)
                #print("Calibrating......")
                pub_thread.update(0, 0, 0, 0, speed, turn)
                while(abs(x_drift) >= 200 and detect_state == True):
                    if(x_drift < 0):
                        print("Calibrating Right...")
                        pub_thread.update(0, 0, 0, -1.0, speed, turn)
                    else:
                        print("Calibrating Left...")
                        
                        pub_thread.update(0, 0, 0, 1.0, speed, turn)
                    rospy.sleep(0.01)
                pub_thread.update(0,0,0,0, speed, turn)
            
            elif state == "collide":
                led_msg = 0xFF0000
                led_pub.publish(led_msg)
                
                print("Collide!")
                pub_thread.update(0,0,0,0, speed, turn)
                
            elif state == "search":   # search state
                led_msg = 0xFFFF00
                led_pub.publish(led_msg) 

                #print("Searching...")
                start_time = time.time()
                #while(time.time() - start_time < 25):

                while(detect_state != True):
                    print("Searching...")
                    pub_thread.update(x, y, z, th, speed, turn)     # turn right, th = -2.0
                    rospy.sleep(0.02)
                    if(detect_state):
                        break

                start_time = time.time()
                #while(time.time() - start_time < 2):
                pub_thread.update(0,0,0,0,speed,turn)

            else:                   # follow
                led_msg = 0x00FF00
                led_pub.publish(led_msg)
                while(current_state() == "follow"):
                    print("Following...")
                    #pub_thread.update(x, y, z, th, speed, turn)
                    pub_thread.update(0, 1.5, 0, 0, speed, turn)
                    rospy.sleep(0.02)
                
                '''
                if(current_state() == "search"):                # still searching, turn left
                    start_time = time.time() 
                    print("left")
                    while(time.time() - start_time < 25):
                        pub_thread.update(0.0, 0.0, 0.0, 2.0, speed, turn)
                        #if(detect_state):
                            #break
                    while(time.time() - start_time < 27):
                        pub_thread.update(0,0,0,0,0,0)
                     
                    start_time = time.time()                    # go straight, TODO: stanley?
                    print("straight")
                    while(time.time() - start_time < 5):
                        if(front_min >= 1):
                            l_or_right = turn_left_or_right(desire_theta, ang_diff_to_wall)
                            pub_thread.update(0, 1.0, 0, 0, speed, turn)

                        #if(detect_state or front_min < 1):
                            #break
                    while(time.time() - start_time < 7):
                        pub_thread.update(0, 0, 0, 0, speed, turn)
                ''' 
            rospy.sleep(0.02)

    except Exception as e:
        print(e)
    
    finally:
        pub_thread.stop()
        vision_thread.stop()
        pub_thread.done = True
