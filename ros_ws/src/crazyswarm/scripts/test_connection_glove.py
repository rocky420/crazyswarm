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



rospy.loginfo("Connection to device")
sys.path.insert(1, os.path.join(sys.path[0], '../../../../../Bidirectional_Interface/Bidirectional_interface/Haptics/Interface/src'))
from connections.beagleboneGreenWirelessConnection import BeagleboneGreenWirelessConnection

c = BeagleboneGreenWirelessConnection()
I2C_interface = "PCA9685@I2C[1]"
c.connect()
print('Status: {}'.format(c.getState()))

time.sleep(3)
c.sendMessages([json.dumps({"type": "Settings", "name": I2C_interface, "scan": False})])
c.sendMessages([json.dumps({"type": "Settings", "name": I2C_interface, "dutyFrequency": '50 Hz'})])


