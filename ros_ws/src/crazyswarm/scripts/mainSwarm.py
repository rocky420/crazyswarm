#!/usr/bin/env python

################################################################################
# Modules
################################################################################

# ROS
import rospy
from std_msgs.msg import String, Duration, Header
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
import tf.transformations

# Classes
from Crazyswarm import Crazyswarm
from Handtracking import Hand

# Python
import time
from math import sqrt
import threading

# Crazyflie
from crazyflie_driver.srv import *
from crazyflie_driver.msg import Position as PositionMsg


COMMAND_UPDATE_RATE = 10 # Hz
NB_AGENTS = 1

def hand_tracking_threading(rate):
    hand = Hand(rate)

def swarm_threading(rate):
    global crazyswarm
    crazyswarm = Crazyswarm(nb_agents = NB_AGENTS, rate= rate)


if __name__ == '__main__':  

    rospy.init_node('mainSwarm')

    rospy.loginfo("Initializing hand interface node.")
    
    time.sleep(1.5)

    rate = rospy.Rate(COMMAND_UPDATE_RATE)
    frequency = 10


    threading.Thread(target = hand_tracking_threading, args = (rate,)).start()
    swarm_threading(rate)

    # del crazyswarm

    #threading.Thread(target = swarm_threading, args = (rate,)).start()

