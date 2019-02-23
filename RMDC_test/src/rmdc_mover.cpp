//SBD Metallic Debris Collector Project
//Stanley Black and Decker
//Georgia Institute of Technology
//created by Tyler Brown


//Implementation of the class members in rmdc_move class

#include "rmdc_move.h"

//public members

//make the rmdc_move object
rmdc_move::rmdc_move(const NodeHandle & nh) : node(nh) {

	//subscribe to topic sending out odometry messages
	this -> sub = node.subscribe("odom", 1, this -> handle_odometry);

	//make the publisher object
	this -> pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 0);

	//start current orientation is always zero
	this -> current_orientation;

	//set the initial values to zero for the velocities
	this -> lin.x = 0.0;
	this -> lin.y = 0.0;
	this -> lin.z = 0.0;

	this -> ang.x = 0.0;
	this -> ang.y = 0.0;
	this -> ang.z = 0.0;


}

//fucntion to rotate counterclockwise a given angle
void rmdc_move::rotate(double angle) {

	//make stop false
	this -> stop = false;

	//if the angle is higher than 360 then reduce it to 360
	angle = angle > 360 ? 360 : angle;

	//first make sure the orientation is up to date
	ros::spinOnce();

	//find out how many radians we need to rotate
	double rad_angle = this -> degs_to_rads(angle);

	//determine goal angle to reach
	double goal_angle = this -> current_orientation + rad_angle;

	//correct for the angular periodicity
	goal_angle = goal_angle > 2*M_PI ? goal_angle - 2*M_PI : goal_angle;

	//make a message to send
	geometry_msgs::Twist msg;

	//fill the message
	this -> lin.x = 0.0;
	this -> ang.z = this -> rotation_speed;

	msg.linear = this -> lin;
	msg.angular = this -> ang;

	//monitor the loop rate
	ros::Rate loop_rate(2);

	while (abs(this -> current_orientation - goal_angle) > 0.10 &&
		!(this -> stop)) {
		if (goal_angle < this -> current_orientation) {
			msg.ang.z = - (this -> rotation_speed);
			pub.publish(msg);
		} else {
			publish(msg);
		}

		//listen for interrupts
		ros::spinOnce();

		//slow down the loop
		loop_rate.sleep();
	}


}





//stop the robot completely
void rmdc_move::stop() {

	//fill the linear and angular components with all zeros
	this -> lin.x = 0.0;
	this -> ang.z = 0.0;

	//make the comd_vel message to send
	geometry_msgs::Twist msg;

	//fill up the message
	msg.linear = this -> lin;
	msg.angular = this -> ang;

	//publish the message to stop 50 times
	for (int i = 0; i < 50; ++i) {

		pub.publish(msg);
	}
}


//private members
double degs_to_rads(double angle) {

	//1 radian = PI/180 degrees
	return angle * (M_PI / 180);
}