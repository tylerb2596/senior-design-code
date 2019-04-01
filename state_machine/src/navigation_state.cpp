//SBD Metallic Debris Collector Project
//Stanley Black and Decker
//Georgia Institute of Technology
//created by Tyler Brown

//Conceptually this file defines the navigation state for out robot. Practically
//this file defiens a collection of functions which are used to execute
//all of the system commands needed for successful navigation of the robot.

#ifndef NAVIGATION_STATE
#define NAVIGATION_STATE

#include "ros/ros.h"
#include "include/rmdc_mover.h"



int main(int argc, char ** argv) {

	//intitialize ROS
    ros::init(argc, argv, "navigation_state");

    // NodeHandle is the main access point to communications with the ROS
    // system. The first NodeHandle constructed will fully initialize this node,
    // and the last NodeHandle destructed will close down the node.
    ros::NodeHandle n;

    while(true) {

    }

    return(0);
}


#endif