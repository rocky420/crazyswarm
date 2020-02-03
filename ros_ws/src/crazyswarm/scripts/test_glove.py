#!/usr/bin/env python

from Haptic_interface import Haptic_interface
from Crazyswarm import Crazyswarm
import rospy, time


if __name__ == '__main__':  
    rospy.init_node('mainSwarm')
    haptic_interface = Haptic_interface(device = "glove")
    rate = rospy.Rate(10)
    crazyswarm = Crazyswarm(nb_agents = 2, rate= rate, debug = True)

    while not rospy.is_shutdown():
       
        time.sleep(0.1)
