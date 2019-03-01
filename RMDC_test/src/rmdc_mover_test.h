//SBD Metallic Debris Collector Project
//Stanley Black and Decker
//Georgia Institute of Technology
//created by Tyler Brown


//This file defines a class that will allow for easy motion of the robot.

#ifndef RMDC_MOVER
#define RMDC_MOVER

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "turtlesim/Pose.h"
#include <cmath>

class rmdc_mover {

//private member functions and variables
private:

	//constant speed of a turn for now
	const float max_rotation_speed;

	//constant speed of travel for now
	const float forward_speed;

	//node object needed to subscribe and publish
	ros::NodeHandle node;

	//publisher object
	ros::Publisher pub;

	//subscriber onject
	ros::Subscriber sub;

	//linear and angular components of the velocity
	geometry_msgs::Vector3 lin;
	geometry_msgs::Vector3 ang;

	//current orientation (angle in radians relative
	//to the starting orientation)
	double current_orientation;

	//variable to stop movement in the middle of a loop
	bool stop;

	//private function for converting degrees to radians
	double degs_to_rads(double ang);

	//private fucntion to convert quarternian position to yaw
	double quar_to_yaw(const geometry_msgs::Quaternion & quart);

	//private function to handle incoming odometry messages
	void handle_odometry(const turtlesim::Pose & pose);

	//private function to do mod on doubles
	double double_mod(double num, double mod);

	//private function to make current_orientation settle
	void settle();

//public member functions
public:

	//constructor for the class
	rmdc_mover(const ros::NodeHandle & nh);

	//rotate by deg degrees counterclockwises
	void rotate(double deg);

	//move forward for a given distance
	void move_forward(double distance);

	//move forward until told to stop
	void move_forward();

	//stop moving altogether
	void stop_moving();


};

#endif
