#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('follow_rb')
import rospy

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import sys
import time

from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

TwistMsg = Pose2D

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

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)
    

if __name__ == "__main__":
    rospy.init_node('follow_robot')
    speed = 1.0
    turn = 3.0
    
    TwistMsg = Pose2D

    pub_thread = Robot(0.0)

    x = 0.0
    y = 0.0
    z = 0.0
    th = -1.0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(0,0,0,0,0,0)
        pub_thread.start()

        input("Press Enter to Continue\n")
        start_time = time.time()
        while(time.time() - start_time < 5):
            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)
    
    finally:
        pub_thread.stop()
        pub_thread.done = True
