#!/usr/bin/env python

from Crazyswarm import Crazyswarm
import rospy, time


if __name__ == '__main__':  
    rospy.init_node('mainSwarm')

    rate = rospy.Rate(10)
    crazyswarm = Crazyswarm(nb_agents = 2, rate= rate, debug = True)
    while not rospy.is_shutdown():
        crazyswarm.flocking_behaviour()
        time.sleep(0.1)
