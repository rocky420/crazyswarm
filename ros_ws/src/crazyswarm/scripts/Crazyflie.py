#!/usr/bin/env python

import rospy
import numpy as np
from pycrazyswarm import *
import matplotlib.pyplot as plt

# Crazyflie
from crazyflie_driver.srv import *
from crazyflie_driver.msg import Position as PositionMsg

# ROS
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
import time, copy


class Crazyflie:
    def __init__(self, id_num, debug):
        
        self.id = id_num
        self.init = False
        self.debug = debug
        
        self.secs = 0
        self.prev_secs = 0
        
        self.pos_neu = np.zeros(3)
        self.prev_pos_neu = np.zeros(3)
        self.pos_cmd = np.zeros(3)

        self.previous_position = np.zeros(3)
        self.new_position = np.zeros(3)

        self.vel_neu = np.zeros(3)

        self.target_pos = np.zeros(3)

        self.pos_neu_history =  []
        self.vel_neu_history =  []
        self.dt_history = []
        
        cmd_topic = '/cf' + str(self.id) + '/cmd_position'


        #Publishers
        self.cmd_pos_pub =  rospy.Publisher(cmd_topic, PositionMsg, queue_size=10)
        #Subscribers
        rospy.Subscriber("vrpn_client_node/cf" + str(self.id) + "/pose", PoseStamped, self.__vrpn_pose_callback)

    def set_vel_history(self, vel_neu):
        np.append(self.vel_neu_history, vel_neu)

    def plot_vel_and_pos_history(self):
        plt.figure(1)
        plt.plot(self.vel_neu_history)
        plt.legend(["vel_neu_x", "vel_neu_y", "vel_neu_z"])

        plt.figure(2)
        plt.plot(self.pos_neu_history)
        plt.legend(["pos_neu_x", "pos_neu_y", "pos_neu_z"])

        plt.figure(3)
        plt.plot(self.dt_history)
        plt.legend(["dt"])
        plt.ylim([-0.01, 0.015])
    ################### Private methods ######################

    def __vrpn_pose_callback(self, msg):
        
        if self.init :
            self.prev_secs = self.secs
            for i in range(0,3):
                self.prev_pos_neu[i] = np.copy(self.pos_neu[i])
        
        self.secs =  msg.header.stamp.secs + msg.header.stamp.nsecs * pow(10,-9)
        if self.debug:    
            rospy.loginfo("time : " + str(self.secs))
            rospy.loginfo("previous time : " + str(self.prev_secs))
            rospy.loginfo("")

        self.pos_neu[0] = msg.pose.position.x
        self.pos_neu[1] = msg.pose.position.y
        self.pos_neu[2] = msg.pose.position.z

        dt = self.secs-self.prev_secs
        if dt > 0.008:
            if self.init :
                for i in range(0,3):
                    self.vel_neu[i] = (self.pos_neu[i] - self.prev_pos_neu[i])/(self.secs-self.prev_secs)
            # rospy.loginfo(self.vel_neu)
            self.pos_neu_history.append(np.copy(self.pos_neu))
            self.vel_neu_history.append(np.copy(self.vel_neu))
            self.dt_history.append(np.copy((self.secs - self.prev_secs)))

            # After verification, the velocity computed here is correct        
            if self.debug:
                rospy.loginfo("position : " + str(self.pos_neu))
                rospy.loginfo("previous position : " + str(self.prev_pos_neu))
                rospy.loginfo("delta_t : " + str((self.secs-self.prev_secs)))
                rospy.loginfo("Velocity : " + str(self.vel_neu))
                rospy.loginfo("")
                rospy.loginfo((self.secs - self.prev_secs))

            self.init = True

