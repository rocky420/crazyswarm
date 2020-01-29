#!/usr/bin/env python

import rospy
import numpy as np
from pycrazyswarm import *

# Crazyflie
from crazyflie_driver.srv import *
from crazyflie_driver.msg import Position as PositionMsg

# ROS
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose



class Hand:
    def __init__(self, rate):
        
        
        self.hand_pose = np.zeros(3)
        
        self.raw_hand_pos = np.zeros(3)
        self.prev_raw_hand_pos = np.zeros(3)
        self.delta_hand_pos = np.zeros(3)

        
        cmd_topic = 'hand/pose'


        #Publishers
        self.hand_pos_pub =  rospy.Publisher(cmd_topic, PositionMsg, queue_size=10)
        #Subscribers
        rospy.Subscriber("vrpn_client_node/hand/pose", PoseStamped, self.__vrpn_hand_pose_callback)

        self.__send_hand_pose(rate)


    #########################################

    def __vrpn_hand_pose_callback(self, msg):
        self.hand_pose[0] =  msg.pose.position.x
        self.hand_pose[1] =  msg.pose.position.y
        self.hand_pose[2] =  msg.pose.position.z

    def  __send_hand_pose(self, rate):
        while not rospy.is_shutdown():
            self.prev_raw_hand_pos = np.copy(self.raw_hand_pos)
            self.raw_hand_pos = np.copy(self.hand_pose)

            self.delta_hand_pos = self.raw_hand_pos - self.prev_raw_hand_pos

            if np.linalg.norm(self.delta_hand_pos) > 1 : self.delta_hand_pos /= np.linalg.norm(self.delta_hand_pos)

            posMsg = PositionMsg()
            posMsg.x = self.delta_hand_pos[0]
            posMsg.y = self.delta_hand_pos[1]
            posMsg.z = self.delta_hand_pos[2]

            posMsg.header.seq = rospy.get_rostime()
            
            self.hand_pos_pub.publish(posMsg)

            rate.sleep()