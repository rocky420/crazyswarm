#!/usr/bin/env python

import rospy
import numpy as np
from pycrazyswarm import *

# Crazyflie
from crazyflie_driver.srv import *
from crazyflie_driver.msg import Position as PositionMsg

class Crazyflie:
    def __init__(self, nb_agents):
    	self.agents = []
    	self.nb_agents = 0
    	if nb_agents>0
    		self.add_n(nb_agents)
    	self.nb_agents = nb_agents

    def add(self, crazyflie):
    	self.nb_agents +=1
    	self.agents.append(crazyflie)

    def add_n(self, n )
    	for drone_id in range(self.nb_agents + 1, self.nb_agents + n + 1):
    		self.add(Crazyflie(drone_id))

    def get_positions(self):
    	all_positions = np.zeros((self.nb_agents, 3))
    	for drone_id in range(0, self.nb_agents):
    		all_positions[drone_id] = self.agents[drone_id].pos_neu

    	return all_positions

    def get_velocities(self):
    	all_velocities = np.zeros((self.nb_agents, 3))
    	for drone_id in range(0, self.nb_agents):
    		all_velocities[drone_id] = self.agents[drone_id].vel_neu

    	return all_velocities

    def takeoff(self, height, duration, rate)
    	start_positions = self.get_positions()

    	final_positions = np.zeros((self.nb_agents,3))
    	for i in range(0, self.nb_agents):
    		final_positions[i][3] = height

    	final_positions = start_positions + final_positions

    	for i in range(bite):
    		current_positions = self.get_positions()
    		for drone_id in range(0, self.nb_agents):
    			cf = self.agents[drone_id]
    			#declare msg crazyflie_driver/Position
    			x = final_positions[drone_id][0]
    			y = final_positions[drone_id][1]
				z = final_positions[drone_id][2]    			
				yaw = 0
				header_seq = drone_id + 1
				msg_header_stamp = rospy.get_time()
				cf.cmd_pos_pub.publish(PositionMsg(header=header, x=x, y=y, z=z, yaw=0.0))
				cf.set_pos_cmd_history(final_positions[drone_id])
				cf.set_pos_history(current_positions[drone_id])