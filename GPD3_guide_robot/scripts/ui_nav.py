#!/usr/bin/env python3

#source: http://edu.gaitech.hk/ria_e100/map-navigation.html

#not sure if these should be imported here or in the master
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point


def chooseDest():
    print("Press a key to choose your destination:")
    print("1: Destination 1") #change names later
    print("2: Destination 2")
    dest = int(input())

    while dest != 1 or dest != 2:
        print("No destination found, try again:")
        dest = int(input())

    return dest


def getDest(dest):
    #define these coordinates manually!
    xDest1 = 0
    yDest1 = 0
    xDest2 = 0
    yDest2 = 0
    if dest == 1:
        return xDest1, yDest1
    elif dest == 2:
        return xDest2, yDest2


#need to pass action client as param?
#pass destination too?
def interruptNav(ac, search, dest):
    ac.cancel_goal()
    if search:
        #handle search behavior
    else:
        #handle obstacle/human
    return getDest(dest) #assumes another function will start going to goal


def goToGoal(x, y):
    #not sure if global will work here (define in main???)
    global ac = actionlib.SimpleActionClient("move_base", MoveBaseAction) #move_base is the server where you send goal requests

    while not ac.wait_for_server(rospy.Duration.from_sec(5.0)):
        rospy.loginfo("waiting for move_base server")

    #initialize goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map" #or whatever our map is called
    goal.target_pose.header.stamp = rospy.Time.now()

    #set and send goal
    goal.target_pose.pose.position = Point(x, y, 0)
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 1
    ac.send_goal(goal)

    #wait until we reach destination
    ac.wait_for_result()
    if ac.get_state() == GoalStatus.SUCCEEDED:
        #reached the destination do some work (like change led to green)
        return True
    else:
        return False 
