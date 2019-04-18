#!/bin/bash

# SBD Metallic Debris Collector Project
# Stanley Black and Decker
# Georgia Institute of Technology
# created by Tyler Brown

#script to run in /etc/rc.local which will run the launch
#files to start the algorithm

#we can also start other packages in here if we need to
rosservice call /fiducial_slam/clear_map
roslaunch state_machine state_machine.launch