#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees, dist
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point


def chooseDest():
    xDest1 = 0 #manually define these coordinates
    yDest1 = 0
    xDest2 = 0
    yDest2 = 0

    print("Press a key to choose your destination:")
    print("1: Destination 1") #change names later
    print("2: Destination 2")
    dest = int(input())

    while dest != 1 or dest != 2:
        print("No destination found, try again:")
        dest = int(input())

    if dest == 1:
        return xDest1, yDest1
    elif dest == 2:
        return xDest2, yDest2


def goToNode(x, y, ac):
    #guideToGoal but simpler
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

    #wait until we reach node
    ac.wait_for_result()
    if ac.get_state() == GoalStatus.SUCCEEDED:
        return True
    else:
        return False


def getCurrentPos():
    #returns the estimate of the robots x, y position


def humanSearch(ac):
    nodes = [] #manually define nodes as list of [x, y]
    unsearched_nodes = nodes #need to make a deep copy??

    #***

    #DO AN INITIAL ROTATION AND CAMERA CHECK FOR HUMAN

    #***
    human_found = False

    while not human_found:
        #find nearest node to robot
        x, y = getCurrentPos()
        min_dist = 1000
        for i in range(len(unsearched_nodes)):
            dist = math.dist([x, y], unsearched_nodes[i])
            if dist < min_dist:
                min_dist = dist
                next_node = unsearched_nodes[i]

        #go to that node
        success = goToNode(next_node[0], next_node[1], ac)
        if success:
            unsearched_nodes.remove(next_node)
            #***
            
            #USE CAMERA AND ROTATION HERE TO DETECT HUMANS
            
            #***
        else:
            print("couldn't reach desired position, trying again")
    #doesnt return anything
    #assumes termination when human was found


def guideToGoal(x, y, ac):
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

    #wait 30s then check if human is following
    while ac.get_state() != GoalStatus.SUCCEEDED:
        ac.wait_for_result(rospy.Duration(30))
        ac.cancel_goal()
        humanSearch(ac)
        ac.send_goal(goal)

    if ac.get_state() == GoalStatus.SUCCEEDED:
        return True
    else:
        return False


if __name__ == "__main__":
    try:
        rospy.init_node('robot_controller', anonymous = True)

        #wait for action server
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        while not ac.waut_for_server(rospy.Duration.from_sec(5.0)):
            rospy.loginfo("waiting for move_base server")

        #begin guiding
        xGoal, yGoal = chooseDest()
        success = guideToGoal(xGoal, yGoal, ac)

        if success:
            print("You have reached your destination!")
            #change LED to green?
        else:
            print("Failed to reach your destination :(")

    except rospy.ROSInterruptException:
        print("ending process due to keyboard interrupt")
