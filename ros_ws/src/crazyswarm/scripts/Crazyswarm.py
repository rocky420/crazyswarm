#!/usr/bin/env python

import rospy
import numpy as np
import time
import std_msgs
from pycrazyswarm import *
from pynput import mouse


# Crazyflie
from crazyflie_driver.srv import *
from crazyflie_driver.msg import Position as PositionMsg



from Crazyflie import Crazyflie

class Crazyswarm:
    def __init__(self, nb_agents, rate):
        self.agents = []
        self.nb_agents = 0
        if nb_agents>0:
            self.add_n(nb_agents)
        self.nb_agents = nb_agents
        self.target_height = 0
        self.rate = rate
        self.flying = False

        #Subscribers
        rospy.Subscriber("/hand/pose", PositionMsg, self.__send_relative_pos_commands)

        rospy.loginfo("Done initializing crazyflies ")

        self.clutchActivated = False

        mouseListener = mouse.Listener(on_click=self.onClickCallback)
        mouseListener.start()

        rospy.loginfo("Done initializing mouseListener")

    def onClickCallback(self, x, y, button, pressed):
        if button ==  mouse.Button.left:
            if (pressed):
                self.clutchActivated = True
            else:
                self.clutchActivated = False
# elif button ==  mouse.Button.right:
#     if (pressed):
#         if droneState == LANDED:
#             droneState = TAKING_OFF
#             threading.Thread(target=waitForTakeOff, args=(TAKE_OFF_DURATION,)).start()
#         elif droneState == FLYING:
#             droneState = LANDING
#             threading.Thread(target=waitForLanding, args=(TAKE_OFF_DURATION,)).start()



    def add(self, crazyflie):
        self.nb_agents +=1
        self.agents.append(crazyflie)

    def add_n(self, n ):
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

    def land(self, duration):
        rospy.loginfo("Landing !")
        start_positions = self.get_positions()
        temp_pos = np.zeros((self.nb_agents,3))
        for i in range(0, self.nb_agents):
            temp_pos[i][2] = start_positions[i][2] - 0.05

        land_positions = start_positions - temp_pos

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
                cf.cmd_pos_pub.publish(msg)
                cf.set_pos_cmd_history(land_positions[drone_id])
                cf.set_pos_history(current_positions[drone_id])
            self.rate.sleep()
            
        self.flying = False



    def takeoff(self, height, duration):
        rospy.loginfo("Taking off !")
        self.target_height = height
        start_positions = self.get_positions()

        final_positions = np.zeros((self.nb_agents,3))
        for drone_id in range(0, self.nb_agents):
            final_positions[drone_id][2] = height

        final_positions = start_positions + final_positions

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

                cf.cmd_pos_pub.publish(msg)
                cf.set_pos_cmd_history(final_positions[drone_id])
                cf.set_pos_history(current_positions[drone_id])
            self.rate.sleep()
        self.agents[0].master_target = self.agents[0].pos_neu
        self.flying = True


    def send_pos_commands(self, cmd_positions):
        for drone_id in range(1,self.nb_agents):
            cf = self.agents[drone_id]
            
            #height security at 3m
            for i in xrange(3):
                if cmd_positions[drone_id][i] > 3 : cmd_positions[drone_id][i] = 3


            #horizontal security
            #TODO

            msg = PositionMsg()
            msg.x = cmd_positions[drone_id][0]
            msg.y = cmd_positions[drone_id][1]
            msg.z = cmd_positions[drone_id][2]
            msg.yaw = 0
            msg.header.seq = drone_id + 1
            msg.header.stamp = rospy.get_rostime()
            cf.cmd_pos_pub.publish(msg)
            cf.set_pos_cmd_history(cmd_positions[drone_id])
        self.rate.sleep()


    def flocking_behaviour(self, rate):
        if(self.nb_agents) > 1:
            cog = self.__get_cog()
            average_vel = self.__get_average_velocity()
            slave_cmd_positions = self.__slave_cmd_positions(cog,average_vel, rate)

            cmd_positions = slave_cmd_positions
            for i in range(1, self.nb_agents):
                cmd_positions[i][2] = self.target_height

            self.send_pos_commands(cmd_positions)


    ############# Private method #############
    def __send_relative_pos_commands(self, handPosition):
        #send a position target relative to the position of the master
        if self.flying:

            drone_id = 0 #The master is always the drone 0
            cf = self.agents[drone_id]
            
            if self.clutchActivated:
                current_master_target = cf.master_target
                
                deltaHandPosition = np.array([handPosition.x, handPosition.y, handPosition.z])

                new_master_target = current_master_target + deltaHandPosition

                #distance security at 3m
                for i in xrange(3):
                    if new_master_target[i] > 3: new_master_target[i] = 3
        
                self.target_height = new_master_target[2]
            
                #To avoid ground crashes, lower limit at 20 cm...
                if new_master_target[2] < 0.2: new_master_target[2] = 0.2
            
            else: 
                new_master_target = np.copy(cf.master_target)
                new_master_target[2] = self.target_height



            msg = PositionMsg()
            msg.x = new_master_target[0]
            msg.y = new_master_target[1]
            msg.z = new_master_target[2]
            msg.yaw = 0
            msg.header.seq = drone_id + 1
            msg.header.stamp = rospy.get_rostime()

            cf.cmd_pos_pub.publish(msg)
            cf.set_pos_cmd_history(new_master_target)
            cf.master_target = new_master_target
            self.rate.sleep()



    def __get_cog(self):
        #calculate Center Of Gravity of the swarm
        cog = np.zeros(3)
        for drone_id in xrange(self.nb_agents):
            cf = self.agents[drone_id]
            cog += cf.pos_neu
        cog /= self.nb_agents
        return cog

    def __get_average_velocity(self):
        vel = np.zeros(3)
        for drone_id in xrange(self.nb_agents):
            cf = self.agents[drone_id]
            vel += cf.vel_neu
        vel/= self.nb_agents
        return vel

    def __cohesion(self, cog):
        #cohesion term of reynold algo
        coh = np.zeros((self.nb_agents,3))
        #the master is not affected:        
        coh[0] = np.zeros(3)
        for drone_id in range(1,self.nb_agents):
            cf = self.agents[drone_id]
            coh[drone_id] = cog - cf.pos_neu

        return coh

    def __separation(self):
        #calculate separation term of reynold algo
        sep = np.zeros((self.nb_agents, 3))
        #the master is not affected:
        sep[0] = np.zeros(3)
        for drone_id1 in range(1,self.nb_agents):
            separation = np.zeros(2)
            for drone_id2 in xrange(self.nb_agents):
                if drone_id1 != drone_id2:
                    cf1 = self.agents[drone_id1]
                    cf2 = self.agents[drone_id2]
                    diff = cf1.pos_neu[0:2] - cf2.pos_neu[0:2]
                    difflen = np.linalg.norm(diff)
                    separation = separation + diff/(difflen*difflen)
            sep[drone_id1][0:2] = separation
        return sep

    def __alignment(self, average_vel):
        align = np.zeros((self.nb_agents,3))
        align[0] = np.zeros(3)
        for drone_id in range(1, self.nb_agents):
            cf = self.agents[drone_id]
            align[drone_id] = average_vel - cf.vel_neu
        return align


    def __slave_cmd_positions(self, cog, average_vel, rate):
        dt = 0.1
        k_coh = 0.02
        k_sep = 0.008
        k_align = 0.001
        P = 0.4
        acc_reynold = k_coh*self.__cohesion(cog) + k_sep*self.__separation() + k_align*self.__alignment(average_vel)
        desired_velocities = acc_reynold/dt
        desired_positions = desired_velocities/dt
        cmd_positions = self.get_positions() + P*desired_positions

        return cmd_positions












        