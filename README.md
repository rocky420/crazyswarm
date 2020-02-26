
[![Build Status](https://travis-ci.org/USC-ACTLab/crazyswarm.svg?branch=master)](https://travis-ci.org/USC-ACTLab/crazyswarm)


<p align="center">
  <img src=https://github.com/AntoineWeber/Bidirectional_Interface/blob/master/readme_images/epfl_logo.png>
</p>


# Crazyswarm
This repository contains the real world implementation of the Master project entitled "Evaluation of swarm information transmission with haptic and visual feedback systems" performed at the LIS (EPFL).

More specifically, a swarm of Crazyflies (up to 5 drones) maintains a flocking behaviour and can be controlled by the hand movement. 

The position of the drones, as well as the position of the hand is captured using Optitrack. Some markers are placed on the hand and on the drones to make them traceable, and a mouse is used as a clutch mechanism.

The original documentation of the Crazyswarm is available here: http://crazyswarm.readthedocs.io/en/latest/.

## Author

* **Hugo Kohli**

## Installation
The communication between the drones, the optitrack PC and the control PC is done using ROS Kinetic, which requires to have Ubuntu 16.04 installed.

The first step is to clone the repository and the submodules, and checkout to the correct branch 

    git clone --recurse-submodules https://github.com/lis-epfl/crazyswarm.git
    cd crazyswarm
    git checkout hugo-master
    ./build.sh
Once this done, go to the ros catking workspace, source the setup file and initialize the workspace:

    cd ros_ws
    source devel/setup.bash
    catkin_make

The installation is done, don't forget to source the setup file for each new terminal you're opening !

## Setup
The full description of the setup for the drones, the radio and the Optitrack is described in this [documentation sheet](https://docs.google.com/document/d/1Y48amTBw6Qvj5wFzh-YaJSl-eiJA-wuTf_qebyt7qNo/edit#heading=h.erk4gy2wz4zw) (an authorization has to be granted). It is important to **read entirely and carefully this document** ! Skip the first part about cloning the repository.

## Structure 

### Python classes
The control of the swarm is done using python classes, located in `ros_ws/src/crazyswarm/scripts` 

 - `Crazyflie.py` For each element of the swarm, an object Crazyflie is created. It contains mainly a method getting the position of the drone from the Optitrack system
 - `Crazyswarm.py` This class contains method to control the swarm (takeoff, land, goto, etc) and to adopt the flocking behaviour
 - `Handtracking.py` This class is used to track the motion of the hand to control the swarm
 - `Haptic_interface.py`, `Bracelets.py` and `Glove.py` These methods are meant to be used to transmit information from the swarm to the pilot, as previously done in simulation ([see this repo](https://github.com/hukohli/Bidirectional_Interface), and more specifically are based on [this python script](https://github.com/hukohli/Bidirectional_Interface/blob/master/Bidirectional_interface/Haptics/API_Calls/main.py) ). WARNING, these classes are not finished

## Fly the swarm

As described in the provided documentation, the first step to fly the swarm is to launch the `ros_ws/src/crazyswarm/launch/hover_swarm.launch` file, to initialize the server for the swarm of crazyflies (make sure you have adjusted the parameters as explained in the doc):

    roslaunch crazyswarm hover_swarm.launch
  Then, a node, `ros_ws/src/crazyswarm/scripts/mainSwarm.py` has to be launched using rosrun. Before launching it, make sure to modify the variable `NB_AGENTS` in `mainSwarm.py` with the number of drones composing your swarm:
  

    rosrun crazyswarm mainSwarm.py
 
Once these two files launched, the swarm will take off if the left click of the mouse is pressed.
Then, the Master drone will follow the movement of the hand if the right click is pressed and the other drone will maintain connectivity and follow the master.

Make sure that the position of the crazyflies is correctly streamed from Optitrack (correct names and id, see the crazyswarm doc for more details). Also, a rigid body called 'hand' created from the markers has to be streamed from Motive. 

