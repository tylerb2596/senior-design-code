//SBD Metallic Debris Collector Project
//Stanley Black and Decker
//Georgia Institute of Technology
//created by Tyler Brown

//This file contains an abstract class that all of the states in the state
//machine will inherit from. It contains methods and data members tbhat should
//be common to all states.

#include "ros/ros.h"

class rmdc_state {

//public members
public:

	//these functions must be implemented in any subclasses
	virtual void run() = 0;
	virtual void init(int argc, char** argv) = 0;

	//node handler object
	 ros::NodeHandle nh;

};