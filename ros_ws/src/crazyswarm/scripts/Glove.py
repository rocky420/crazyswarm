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

### WARNING this class is not finished !!!! based on https://github.com/hukohli/Bidirectional_Interface/blob/master/Bidirectional_interface/Haptics/API_Calls/main.py ######

class Glove:
    def __init__(self):


        rospy.loginfo("Done initiallizing Glove")


    #######################################################3



    def turnOnMotors(self, list_of_motors, error, max_error):
    	rospy.loginfo("turnOnMotors")

    def connect(self):
    	rospy.loginfo("Connection to device")
    	sys.path.insert(1, os.path.join(sys.path[0], '../../../../../Bidirectional_Interface/Bidirectional_interface/Haptics/Interface/src'))
        from connections.beagleboneGreenWirelessConnection import BeagleboneGreenWirelessConnection

        c = BeagleboneGreenWirelessConnection()
        I2C_interface = "PCA9685@I2C[1]"
