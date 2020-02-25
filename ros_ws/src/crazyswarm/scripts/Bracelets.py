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


class Bracelets:
    def __init__(self):


        rospy.loginfo("Done initiallizing Glove")


    #######################################################3
