//SBD Metallic Debris Collector Project
//Stanley Black and Decker
//Georgia Institute of Technology
//created by Tyler Brown

//Conceptually this file defines the decision state for out robot. Practically
//this file defiens a class that will hold functions which are used to execute
//all of the system commands needed for successful navigation decisions in the
//robot.

#ifndef DECISION_STATE
#define DECISION_STATE

#include "ros/ros.h"
#include "state_machine/control.h"
#include <iostream>


void start_stop_callback(const state_machine::control& control);


int main(int argc, char ** argv) {

	//intitialize ROS
    ros::init(argc, argv, "decision_state");

    // NodeHandle is the main access point to communications with the ROS system.
    // The first NodeHandle constructed will fully initialize this node, and the last
    // NodeHandle destructed will close down the node.
    ros::NodeHandle n;

    //subscribe to start messages
    ros::Subscriber subscribe_control = n.subscribe("control", 1000, start_stop_callback);

    //wait for messages to come in
    ros::spin();

    return(0);
}

void start_stop_callback(const state_machine::control& control) {

	std::cout << "I got the message " << control.message << std::endl;

}

#endif