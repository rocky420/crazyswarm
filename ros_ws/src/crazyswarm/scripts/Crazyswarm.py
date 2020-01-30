#!/usr/bin/env python

import rospy
import numpy as np
import time
from  std_msgs.msg import Int16
from pycrazyswarm import *
from pynput import mouse


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
HAND_ROOM_SCALE = 5




class Crazyswarm:
    def __init__(self, nb_agents, rate, debug = False):
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
        self.desired_positions = np.zeros((self.nb_agents, 3))
        self.old_desired_positions = np.zeros((self.nb_agents, 3))

        self.desired_velocities = np.zeros((self.nb_agents, 3))
        self.rel_desired_positions = np.zeros((self.nb_agents, 3))

        self.secs = 0
        self.prev_secs = self.secs
        self.log_time = []

        self.state = LANDED

        self.dt = np.zeros(self.nb_agents)*0.1

        cmd_topic = 'swarm/state'


        #Publishers
        self.swarm_state_pub =  rospy.Publisher(cmd_topic, Int16, queue_size=10)

        #Subscribers
        rospy.Subscriber("/hand/pose", PositionMsg, self.__get_master_target)

        rospy.loginfo("Done initializing crazyflies ")

        self.clutchActivated = False

        mouseListener = mouse.Listener(on_click=self.onClickCallback)
        mouseListener.start()

        rospy.loginfo("Done initializing mouseListener")
        if not self.debug: self.__control_loop()


    def onClickCallback(self, x, y, button, pressed):
        self.swarm_state_pub.publish(2)
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
        self.state = LANDING
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
        rospy.logwarn(self.log_time)
        self.state = LANDED

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

        self.desired_positions = self.get_positions()
        self.old_desired_positions = self.desired_positions
        self.master_target = final_positions[0]
        self.state = FLYING

    def send_pos_commands(self, cmd_positions):
        for drone_id in xrange(self.nb_agents):
            cf = self.agents[drone_id]
            
            #height security at 3m
            for i in xrange(3):
                if cmd_positions[drone_id][i] > 4 : 
                    cmd_positions[drone_id][i] = 4
                    rospy.logwarn("security limitations")

            #horizontal security
            #TODO

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


    def __get_master_target(self, handPosition):
        #send a position target relative to the position of the master
        if self.state == FLYING:
            drone_id = 0 #The master is always the drone 0
            cf = self.agents[drone_id]
            if self.clutchActivated:
                current_master_target = np.copy(cf.target_pos)
                
                deltaHandPosition = np.array([handPosition.x, handPosition.y, handPosition.z])
                new_master_target = current_master_target + deltaHandPosition * HAND_ROOM_SCALE        
                self.target_height = np.copy(new_master_target[2])
                #To avoid ground crashes, lower limit at 20 cm...
                if new_master_target[2] < 0.2: new_master_target[2] = 0.2
            else: 
                new_master_target = np.copy(cf.target_pos)
                new_master_target[2] = self.target_height

            self.master_target = new_master_target
            cf.target_pos = self.master_target


    def __get_cog(self):
        #calculate Center Of Gravity of the swarm
        cog = np.zeros(3)
        for drone_id in xrange(self.nb_agents):
            cog+= self.agents[drone_id].pos_neu
        cog /= self.nb_agents

        return cog

    def __get_average_velocity(self):
        vel = np.zeros(3)
        for drone_id in xrange(self.nb_agents):
            cf = self.agents[drone_id]
            vel += cf.vel_neu
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

    def __alignment(self, cf):
        average_vel = self.__get_average_velocity()

        align = np.zeros(3)
        align = average_vel - cf.vel_neu
        return align


    def __slave_cmd_positions(self):
        k_coh = 2
        k_sep = 0.8
        k_align = 0.1

        dt = 0.1
        P = 2
        D = 2

        cmd_positions = np.zeros((self.nb_agents, 3))
        self.prev_secs = self.secs
        self.secs = rospy.get_rostime().secs + rospy.get_rostime().nsecs*pow(10,-9)
        self.log_time.append(self.secs - self.prev_secs)
        
        self.old_desired_positions = self.desired_positions

        for drone_id in range(1, self.nb_agents):
            cf = self.agents[drone_id]
            #dt = (cf.secs - cf.prev_secs)*10^-9
            
            acc_reynold = k_coh*self.__cohesion(cf) + k_sep*self.__separation(cf,drone_id)
            
            max_acc = 2
            for i in xrange(3):
                if acc_reynold[i] > max_acc: acc_reynold[i] = max_acc
                if acc_reynold[i] < -max_acc: acc_reynold[i] = -max_acc


            if self.debug: rospy.loginfo("acc_reynold : " + str(drone_id) + str(acc_reynold))
            if self.debug: rospy.loginfo("coh : " + str(drone_id) + str(k_coh*self.__cohesion(cf)))
            if self.debug: rospy.loginfo("separation : " + str(drone_id) + str(k_sep*self.__separation(cf, drone_id)))
            if self.debug: rospy.loginfo("alignement : " + str(drone_id) + str(k_align*self.__alignment(cf)))

            self.desired_velocities[drone_id] += acc_reynold*dt 

            self.rel_desired_positions[drone_id] += self.desired_velocities[drone_id]*dt


            max_relative_dist = 0.1

            for i in xrange(3):
                if self.rel_desired_positions[drone_id][i] > max_relative_dist: self.rel_desired_positions[drone_id][i] = max_relative_dist
                if self.rel_desired_positions[drone_id][i] < -max_relative_dist: self.rel_desired_positions[drone_id][i] = -max_relative_dist

            rospy.loginfo("self.rel_desired_positions" + str(self.rel_desired_positions[drone_id]))

            self.desired_positions[drone_id] = cf.pos_neu + self.rel_desired_positions[drone_id]

            #PID CONTROL

            u = P*(self.desired_positions[drone_id] - cf.pos_neu) + D*(self.desired_positions[drone_id] - self.old_desired_positions[drone_id])/dt

            cmd_positions[drone_id] = cf.pos_neu + u

        #rospy.loginfo(cmd_positions)
            #rospy.loginfo(cmd_positions[drone_id])
        #rospy.loginfo()

        return cmd_positions

