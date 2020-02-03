#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket, struct
import select
import time
import json
import sys
import os
import serial
import yaml
import signal
import keyboard
import numpy as np
import rospy

from crazyswarm.msg import ExperimentState as ExperimentStateMsg
from Bracelets import Bracelets
from Glove import Glove


REACHING_HEIGHT = 2;
GO_TO_FIRST_WAYPOINT = 5;
EXTENSION = 6;
WAYPOINT_NAV = 7;
CONTRACTION = 8;


# local IP. Do not change that
UDP_IP = "127.0.0.1"
# socket to which data is being received
UDP_PORT_DISTANCES = 8051

allIndexes = ["up", "back", "right", "front", "left", "down"]


class Haptic_interface:
    def __init__(self, device):
        if device =="glove":
            self.device = Glove()
        elif device == "bracelets":
            self.device = Bracelets()
            
        self.height_error = 0
        self.max_height_error = 1
        self.extension_error = 0
        self.max_extension_error = 2
        self.next_waypoint = np.zeros(3)
        self.max_distance_error = 4

        self.experiment_state = 0

        self.emergency_stop = False
        
        cmd_topic = 'swarm/swarm_state'

        #Init connection with feedback system
        self.device.connect()
        
        #Subscribers
        rospy.Subscriber('swarm/experiment_state', ExperimentStateMsg, self.experiment_state_callback)

        rospy.loginfo("Done initiallizing Glove")




    #######################################################3



    def experiment_state_callback(self, msg):
        self.height_error = msg.height_error
        self.extension_error =  msg.extension_error
        self.next_waypoint[0] = msg.position_error.x
        self.next_waypoint[1] = msg.position_error.y
        self.next_waypoint[2] = msg.position_error.z
        self.experiment_state  = msg.experiment_state
        self.emergency_stop = msg.emergency_stop
        rospy.loginfo("received")
        self.control_loop()

    def control_loop(self):
        if not self.emergency_stop:
            if self.experiment_state == EXTENSION or self.experiment_state == CONTRACTION:
                up_time = 3/10
                error = self.extension_error
                max_error = self.max_extension_error
                if error > 0:
                    self.device.turnOnMotors(["up"], error, max_error)
                    time.sleep(up_time)
                    self.device.turnOnMotors(["up"], 0, max_error)
                    self.device.turnOnMotors(["extensionLeft", "extensionRight"], error, max_error)
                    time.sleep(up_time)
                    self.device.turnOnMotors(["extensionLeft", "extensionRight"], 0, max_error)
                else : 
                    self.device.turnOnMotors(["extensionLeft", "extensionRight"], error, max_error)
                    time.sleep(up_time)
                    self.device.turnOnMotors(["extensionLeft", "extensionRight"], 0, max_error)
                    self.device.turnOnMotors(["up"], error, max_error)
                    time.sleep(up_time)
                    self.device.turnOnMotors(["up"], 0, max_error)
                time.sleep(0.5)
            elif self.experiment_state == GO_TO_FIRST_WAYPOINT or self.experiment_state == WAYPOINT_NAV:
                if abs(self.height_error)> 0.1*self.max_height_error:
                    self.sendHeightCue()
                else :
                    self.sendDirectionalCue()

    def sendHeightCue(self):
        if self.height_error < 0:
            self.device.turnOnMotors(["up"], self.height_error, self.max_height_error)
            self.device.turnOnMotors(["down"], 0, self.max_height_error)
        else :
            self.device.turnOnMotors(["down"], self.height_error, self.max_height_error)
            self.device.turnOnMotors(["up"], 0, self.max_height_error)

        self.device.turnOnMotors(["front","back", "left", "right"], 0, self.max_height_error)

    def sendDirectionalCue(self):
        self.send1DirecionalCue(self.position_error[0], "front", "back")
        self.send1DirecionalCue(self.position_error[1], "left", "right")

        self.device.turnOnMotors(["up","down"], 0, self.max_height_error)

    def send1DirecionalCue(self, distance, direction, negative_direction):
        if distance < 0:
            self.device.turnOnMotors([negative_direction], distance, self.max_distance_error)
            self.device.turnOnMotors([direction], 0, self.max_distance_error)

        elif distance > 0:
            self.device.turnOnMotors([negative_direction], 0, self.max_distance_error)
            self.device.turnOnMotors([direction], distance, self.max_distance_error)

    def shutDownAllMotors(self):
        self.device.turnOnMotors(allIndexes,0,  1)





