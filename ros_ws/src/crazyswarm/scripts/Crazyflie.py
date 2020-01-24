#!/usr/bin/env python

import rospy
import numpy as np
from pycrazyswarm import *

# Crazyflie
from crazyflie_driver.srv import *
from crazyflie_driver.msg import Position as PositionMsg

class Crazyflie:
    def __init__(self, id_num):
        
        self.id = id_num
        self.init = False
        
        # self.secs = secs
        # self.prev_secs = prev_secs
        
        self.pos_neu = np.zeros(3)
        self.prev_pos_neu = np.zeros(3)
        self.pos_cmd = np.zeros(3)

        self.vel_neu = np..zeros(3)

        self.pos_neu_history =  np.array([])
        self.pos_cmd_history =  np.array([])
        self.vel_neu_history =  np.array([])
        
        cmd_topic = '/cf' + str(self.id) + '/cmd_position'


        #Publishers
        self.cmd_pos_pub =  rospy.Publisher(cmd_topic, PositionMsg, queue_size=10)
        #Subscribers
        self.pos_sub = rospy.Subscriber("vrpn_client_node/cf" + str(self.id) + "/pose", PoseStamped, vrpn_pose_callback)

    def set_pos_history(self, position_neu):
        # Set the position history of the drone in the NEU frame

        # Update pos_neu_history
        np.append(pos_neu_history, position_neu)


    def set_pos_cmd_history(self, position_cmd):
        np.append(pos_cmd_history, position_cmd)

    def set_vel_history(self, vel_neu):
        np.append(vel_neu_history, vel_neu)


    #########################################

    def __vrpn_pose_callback(self, msg):
        self.secs = msg.Header.Stamp.Sec + msg.Header.Stamp.Nsec*10^-9
        
        if self.init
            self.prev_secs = self.secs
            for i in range(0,3):
                self.prev_pos_neu[i] = self.pos_neu[i];

        self.pos_neu[0] = msg.Pose.Position.X
        self.pos_neu[1] = msg.Pose.Position.Y
        self.pos_neu[2] = msg.Pose.Position.Z

        if self.init :
            for i in range(0,3):
                self.vel_neu[i] = (self.pos_neu[i] - self.prev_pos_neu[i])/(self.secs-self.prev_secs)

        self.init = True