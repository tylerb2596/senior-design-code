//SBD Metallic Debris Collector Project
//Stanley Black and Decker
//Georgia Institute of Technology
//created by Tyler Brown


//Implementation of the class members in rmdc_move class

#include "rmdc_mover_test.h"

//public members

//make the rmdc_move object
rmdc_mover::rmdc_mover(const ros::NodeHandle & nh) : node(nh), max_rotation_speed(M_PI/2), forward_speed(1.0), stop(false) {

	//subscribe to topic sending out odometry messages
	this -> sub = node.subscribe("turtle1/pose", 1, &rmdc_mover::handle_odometry, this);

	//make the publisher object
	this -> pub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);

	//start current orientation is always zero
	this -> current_orientation = 0.0;

	//set the initial values to zero for the velocities
	this -> lin.x = 0.0;
	this -> lin.y = 0.0;
	this -> lin.z = 0.0;

	this -> ang.x = 0.0;
	this -> ang.y = 0.0;
	this -> ang.z = 0.0;


}

//fucntion to rotate given an angle
//if the value is negative it will rotate clockwise,
//if its positive it will rotate counterclockwise
void rmdc_mover::rotate(double angle) {

	//if the angle is higher than 360 then reduce it to 360
	angle =  double_mod(angle, 360);

	//find out how many radians we need to rotate
	double rad_angle = this -> degs_to_rads(angle);

	//determine goal angle to reach
	double goal_angle = this -> current_orientation + rad_angle;

	//make a message to send
	geometry_msgs::Twist msg;

	//set the zero values
	this -> lin.x = 0.0;
	this -> lin.y = 0.0;
	this -> lin.z = 0.0;

	this -> ang.x = 0.0;
	this -> ang.y = 0.0;

	//slow down the loop
	ros::Rate loop_rate(10);

	//correct for the angular periodicity
	goal_angle = goal_angle > 2*M_PI ? goal_angle - 2*M_PI : goal_angle;

	while (rad_angle > 0.03) {

		if (rad_angle > max_rotation_speed) {
			this -> ang.z = max_rotation_speed;
			msg.angular = this -> ang;
			msg.linear = this -> lin;
			pub.publish(msg);
		} else {
			this -> ang.z = rad_angle;
			msg.angular = this -> ang;
			msg.linear = this -> lin;
			pub.publish(msg);
		}

		loop_rate.sleep();
		this -> settle();

		rad_angle = std::abs(goal_angle - this -> current_orientation);

	}

}



//stop the robot completely
void rmdc_mover::stop_moving() {

	//fill the linear and angular components with all zeros
	this -> lin.x = 0.0;
	this -> ang.z = 0.0;

	//make the comd_vel message to send
	geometry_msgs::Twist msg;

	//fill up the message
	msg.linear = this -> lin;
	msg.angular = this -> ang;

	//monitor the loop rate
	ros::Rate loop_rate(10);

	//publish the message to stop 50 times
	for (int i = 0; i < 50; ++i) {

		pub.publish(msg);

		//slow down the loop
		loop_rate.sleep();
	}
}


//private members

//convert degrees to radians
double rmdc_mover::degs_to_rads(double angle) {

	//1 radian = PI/180 degrees
	return angle * (M_PI / 180);
}

//convert quarternian coordinates to yaw
double rmdc_mover::quar_to_yaw(const geometry_msgs::Quaternion & quart) {

	double yaw;

	//convert quarternian orientation to yaw
	double siny_cosp = +2.0 * (quart.w * quart.z + quart.x * quart.y);
	double cosy_cosp = +1.0 - 2.0 * (quart.y * quart.y + quart.z * quart.z);
	yaw = atan2(siny_cosp, cosy_cosp);

	return yaw;
}

//TODO: change this function to handle ying yings messages
//handle messages coming from the odometry
void rmdc_mover::handle_odometry(const turtlesim::Pose & pose) {

	//set the current orientation to the odometry's estimation of it
	this -> current_orientation = std::abs(pose.theta);

}


//function to do mod on doubles
double rmdc_mover::double_mod(double num, double mod) {

	if (num > 0 && num < mod) {
		return num;
	}

	double how_many = num / mod;

	if (num < 0) {

		while(num < 0) {
			num += mod;
		}

		return num;
	}

	while (how_many > 1) {
		how_many -= 1;
	}


	return how_many * std::abs(num);

}

//function to make sure the orientation is up to date
void rmdc_mover::settle() {

	double beginning_orientation = this -> current_orientation;
	int count = 0;

	while(count < 10) {
		if (beginning_orientation == current_orientation) {
			count++;
		} else {
			count = 0;
		}

		ros::spinOnce();

		beginning_orientation = this -> current_orientation;
	}

}