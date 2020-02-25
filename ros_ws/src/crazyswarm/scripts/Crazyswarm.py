#!/usr/bin/env python

import rospy
import numpy as np
import time
from  std_msgs.msg import Int16

from crazyswarm.msg import ExperimentState as ExperimentStateMsg
from pycrazyswarm import *
from pynput import mouse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import scipy.integrate
from scipy.signal import butter, lfilter, freqz

# Crazyflie
from crazyflie_driver.srv import *
from crazyflie_driver.msg import Position as PositionMsg


from Crazyflie import Crazyflie

LANDED = 0
TAKING_OFF = 1
FLYING = 2
LANDING = 3
WAIT_FOR_TAKE_OFF = 4
WAIT_FOR_LANDING = 5
HAND_ROOM_SCALE = 3




class Crazyswarm:
    def __init__(self, nb_agents, rate, debug= False ):
        self.init = False
        self.show = True
        
        self.agents = []
        self.nb_agents = 0
        if nb_agents>0:
            self.add_n(nb_agents)

        self.nb_agents = nb_agents
        self.target_height = 0
        self.rate = rate
        self.flying = False 
        self.debug = debug


        self.master_target = np.zeros(3)

        #Lists used for plots
        self.acc_array = []
        self.vel_array = []
        self.pos_array =[]
        self.time_array = []

        for i in xrange(self.nb_agents):
            self.acc_array.append([])
            self.vel_array.append([])
            self.pos_array.append([])
            self.time_array.append([])

        self.initial_secs = 0
        self.current_time = 0

        self.state = LANDED

        self.dt = np.zeros(self.nb_agents)*0.1

        cmd_topic = 'swarm/experiment_state'

        #Publishers
        self.experiment_state_pub =  rospy.Publisher(cmd_topic, ExperimentStateMsg, queue_size=10)

        #Subscribers
        rospy.Subscriber("/hand/pose", PositionMsg, self.__get_master_target)

        rospy.loginfo("Done initializing crazyflies ")

        self.clutchActivated = False

        mouseListener = mouse.Listener(on_click=self.onClickCallback)
        mouseListener.start()

        rospy.loginfo("Done initializing mouseListener")
        
        self.__control_loop()



    def onClickCallback(self, x, y, button, pressed):
        msg = ExperimentStateMsg()
        msg.height_error = 13.2
        self.experiment_state_pub.publish(msg)
        if button ==  mouse.Button.right:
            if (pressed):
                self.clutchActivated = True
            else:
                self.clutchActivated = False
        elif button == mouse.Button.left:
            if self.state == LANDED:
                self.state = WAIT_FOR_TAKE_OFF
            elif self.state == FLYING:
                self.state = WAIT_FOR_LANDING


    def add(self, crazyflie):
        self.nb_agents +=1
        self.agents.append(crazyflie)

    def add_n(self, n ):
        for drone_id in range(self.nb_agents + 1, self.nb_agents + n + 1):
            self.add(Crazyflie(drone_id, self.debug))

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

    def land(self, duration):
        self.state = LANDING
        rospy.loginfo("Landing !")
        start_positions = self.get_positions()
        temp_pos = np.zeros((self.nb_agents,3))
        for i in range(0, self.nb_agents):
            temp_pos[i][2] = start_positions[i][2] - 0.05

        land_positions = start_positions - temp_pos
        
        #TODO: use rate instead of 10
        duration_iter = int(np.ceil(duration*10))

        for i in xrange(duration_iter):
            current_positions = self.get_positions()
            for drone_id in range(0, self.nb_agents):
                cf = self.agents[drone_id]
                msg = PositionMsg()
                msg.x = land_positions[drone_id][0]
                msg.y = land_positions[drone_id][1]
                msg.z = land_positions[drone_id][2]
                msg.yaw = 0
                msg.header.seq = drone_id + 1
                msg.header.stamp = rospy.get_rostime()
                if not self.debug: cf.cmd_pos_pub.publish(msg)
                cf.set_pos_cmd_history(land_positions[drone_id])
                cf.set_pos_history(current_positions[drone_id])
            self.rate.sleep()
        self.init = False
        self.state = LANDED
    
#######These plots show the velocity, acceleration and position according to the time, uncomment it for debugging purpose ############

        
        if self.debug: self.agents[0].plot_vel_and_pos_history() #print the positions and velocity obtained from optitrack
        # drone_id = 1
        # plt.figure(1)
        # plt.plot(self.time_array[drone_id], self.acc_array[drone_id])
        # name = "acc"
        # plt.legend(["target_" + name+ "_x", "target_" + name+ "_y","target_" + name+ "_z"])
        # drone_id = 2
        # plt.figure(2)
        # plt.plot(self.time_array[drone_id], self.acc_array[drone_id])
        # name = "acc"
        # plt.legend(["target_" + name+ "_x", "target_" + name+ "_y","target_" + name+ "_z"])

        # drone_id = 1
        # plt.figure(5)
        # plt.plot(self.time_array[drone_id], self.vel_array[drone_id])
        # name = "vel"
        # plt.legend(["target_" + name+ "_x", "target_" + name+ "_y","target_" + name+ "_z"])
        # drone_id = 2
        # plt.figure(6)
        # plt.plot(self.time_array[drone_id], self.vel_array[drone_id])
        # name = "vel"
        # plt.legend(["target_" + name+ "_x", "target_" + name+ "_y","target_" + name+ "_z"])

        # drone_id = 1
        # plt.figure(7)
        # plt.plot(self.time_array[drone_id], self.pos_array[drone_id])
        # name = "pos"
        # plt.legend(["target_" + name+ "_x", "target_" + name+ "_y","target_" + name+ "_z"])

        # plt.show()
        # time.sleep(3)
        # plt.close()
        

    def takeoff(self, height, duration):
        rospy.loginfo("Taking off !")
        
        self.target_height = height
        start_positions = self.get_positions()
        final_positions = np.zeros((self.nb_agents,3))

        for drone_id in range(0, self.nb_agents):
            final_positions[drone_id][2] = height

        final_positions = start_positions + final_positions
        self.master_target = final_positions[0]
        if self.debug : rospy.logwarn("final_positions : " + str(final_positions))
        

        #TODO: use rate instead of 10
        duration_iter = int(np.ceil(duration*10))

        for i in xrange(duration_iter):
            current_positions = self.get_positions()
            for drone_id in range(0, self.nb_agents):
                cf = self.agents[drone_id]
                msg = PositionMsg()
                msg.x = final_positions[drone_id][0]
                msg.y = final_positions[drone_id][1]
                msg.z = final_positions[drone_id][2]
                msg.yaw = 0
                msg.header.seq = drone_id + 1
                msg.header.stamp = rospy.get_rostime()

                if not self.debug: cf.cmd_pos_pub.publish(msg)
                cf.set_pos_cmd_history(final_positions[drone_id])
                cf.set_pos_history(current_positions[drone_id])
            self.rate.sleep()

        self.master_target = final_positions[0]
        self.state = FLYING

    def send_pos_commands(self, cmd_positions):
        for drone_id in xrange(self.nb_agents):
            cf = self.agents[drone_id]
            
            #height and horizontal security at 4m
            for i in xrange(3):
                if cmd_positions[drone_id][i] > 4 : 
                    cmd_positions[drone_id][i] = 4
                    rospy.logwarn("security limitations")

            msg = PositionMsg()
            msg.x = cmd_positions[drone_id][0]
            msg.y = cmd_positions[drone_id][1]
            msg.z = cmd_positions[drone_id][2]
            msg.yaw = 0
            msg.header.seq = drone_id + 1
            msg.header.stamp = rospy.get_rostime()
            if not self.debug: cf.cmd_pos_pub.publish(msg)
            cf.set_pos_cmd_history(cmd_positions[drone_id])

    def flocking_behaviour(self):
        slave_cmd_positions = self.__slave_cmd_positions()

        cmd_positions = slave_cmd_positions
        cmd_positions[0] = self.master_target

        for i in range(1, self.nb_agents):
            cmd_positions[i][2] = self.target_height

        self.send_pos_commands(cmd_positions)

    ############# Private method #############
    def __control_loop(self):
        rospy.loginfo("Entered in the control loop") 
        while not rospy.is_shutdown():

            if self.state == WAIT_FOR_TAKE_OFF:
                self.takeoff(height = 1, duration = 3)
            elif self.state == WAIT_FOR_LANDING:
                self.land(duration = 2)

            elif self.state == FLYING:
                self.flocking_behaviour()

            self.rate.sleep()
        rospy.loginfo("Over")

    def __get_master_target(self, handPosition):
        #send a position target for the master
        if self.state == FLYING:
            #The master is always the drone 0
            cf = self.agents[0]
            if self.clutchActivated:
                current_master_target = np.copy(cf.target_pos)
                deltaHandPosition = np.array([handPosition.x, handPosition.y, handPosition.z])
                self.master_target = current_master_target + deltaHandPosition * HAND_ROOM_SCALE        
                self.target_height = np.copy(self.master_target[2])
                #To avoid ground crashes, lower limit at 20 cm...
                if self.master_target[2] < 0.2: self.master_target[2] = 0.2
            else: 
                self.master_target = np.copy(cf.target_pos)
                self.master_target[2] = self.target_height

            cf.target_pos = self.master_target


    def __get_cog(self):
        #calculate Center of Gravity of the swarm
        cog = np.zeros(3)
        for drone_id in xrange(self.nb_agents):
            cog+= self.agents[drone_id].pos_neu

        cog /= self.nb_agents
        rospy.logwarn("COG : " + str(cog))
        return cog

    def __get_average_velocity(self, dt):
        vel = np.zeros(3)
        for drone_id in xrange(self.nb_agents):
            velocity = self.agents[drone_id].vel_neu
            vel += velocity
        vel/= self.nb_agents
        return vel

    def __cohesion(self, cf):
        #cohesion term of reynold algo
        cog = self.__get_cog()
        coh = np.zeros(3)
        coh = cog - cf.pos_neu
        return coh

    def __separation(self, cf, drone_id):
        #calculate separation term of reynold algo
        sep = np.zeros(3)
        separation = np.zeros(2)
        for drone_id2 in xrange(self.nb_agents):
            if drone_id != drone_id2:
                cf2 = self.agents[drone_id2]
                diff = cf.pos_neu[0:2] - cf2.pos_neu[0:2]
                difflen = np.linalg.norm(diff)
                if not difflen == 0:
                    separation += diff/(difflen*difflen)

        sep[0:2] = separation
        return sep

    def __alignment(self, drone_id, dt):
        #separation term of reynold algo
        average_vel = self.__get_average_velocity(dt)

        align = np.zeros(3)
        align = average_vel - self.agents[drone_id].vel_neu
        return align

    def __slave_cmd_positions(self):
        #Coefficients of the Reynolds algo (P multiplies each coefficient)
        P =50
        k_coh = P*2.1
        k_sep = P*0.7
        k_align = P*0.

        #Get initial time
        if not self.init:
            self.initial_secs = rospy.get_rostime().secs + rospy.get_rostime().nsecs*pow(10,-9)
            self.init = True

        cmd_positions = np.zeros((self.nb_agents, 3))

        #Calculate delta t
        self.previous_time = self.current_time
        self.current_time = rospy.get_rostime().secs + rospy.get_rostime().nsecs*pow(10,-9) - self.initial_secs
        dt = self.current_time - self.previous_time
        if dt > 1 or dt < 0.05: dt = 0.1
       
        for drone_id in range(1, self.nb_agents):

            calc_velocities = np.zeros(3)
            position_error = np.zeros(3)
            cf = self.agents[drone_id]
            self.time_array[drone_id].append(self.current_time)

            #For each drone, calculate the Reynolds terms
            acc_reynold =  k_sep*self.__separation(cf,drone_id)  + k_coh*self.__cohesion(cf) + k_align*self.__alignment(drone_id, dt)
            self.acc_array[drone_id].append(acc_reynold)

            #Integrate the acceleration to have the target velocity
            calc_velocities = cf.vel_neu + acc_reynold*dt
            self.vel_array[drone_id].append(calc_velocities)

            #Integrate the velocity to have the relative target position, which is also the position_error
            position_error = calc_velocities*dt
            self.pos_array[drone_id].append(cf.pos_neu)

            cmd_positions[drone_id] = cf.pos_neu + position_error

            if self.debug: 
                rospy.loginfo("acc_reynold : " + str(drone_id) + str(acc_reynold))
                rospy.loginfo("coh : " + str(drone_id) + str(k_coh*self.__cohesion(cf)))
                rospy.loginfo("separation : " + str(drone_id) + str(k_sep*self.__separation(cf, drone_id)))
                rospy.loginfo("alignement : " + str(drone_id) + str(k_align*self.__alignment(drone_id, dt)))
                rospy.loginfo("Real velocity : " + str(cf.vel_neu))
                rospy.loginfo("")


        return cmd_positions

