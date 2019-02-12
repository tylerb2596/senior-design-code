//SBD Metallic Debris Collector Project
//Stanley Black and Decker
//Georgia Institute of Technology
//created by Tyler Brown

//Conceptually this file defines the home state for out robot. Practically
//this file defines a collection of functions which are used to execute
//all of the system commands needed for successful navigation back to the
//deignated home location for the robot.

#ifndef HOME_STATE
#define HOME_STATE

#include "ros/ros.h"


int main(int argc, char ** argv) {

	//intitialize ROS
    ros::init(argc, argv, "home_state");

    // NodeHandle is the main access point to communications with the ROS system.
    // The first NodeHandle constructed will fully initialize this node, and the last
    // NodeHandle destructed will close down the node.
    ros::NodeHandle n;

    return(0);
}


#endif
