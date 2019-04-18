//SBD Metallic Debris Collector Project
//Stanley Black and Decker
//Georgia Institute of Technology
//created by Tyler Brown

//this file is used to simply keep track of the two time intervals need to make
//the algorithm function properly

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include <chrono>
#include <string>

//start time variable
std::chrono::high_resolution_clock::time_point start_time;

//elapsed time since the interval started
std::chrono::duration<double> elapsed_time;

//boolean to know if the program has been paused, stopped or started
bool paused = false;

//function used to handle the user_control messages coming in
void user_control_callback(const std_msgs::String& message);


//main function
int main(int argc, char** argv) {

	//initialize ros
	ros::init(argc, argv, "state_machine_timer");

	//ros node used for main entry point into ros
	ros::NodeHandle node;

	//subscribe to user_control messages to know when to start, pause and stop
	//the timer
	ros::Subscriber user_control_subscriber = node.subscribe(
		"/state_machine/user_control", 10, user_control_callback);

	//make a publisher to send out timer messages that contains the elapsed time
	ros::Publisher timer_publisher = node.advertise<std_msgs::Int16>(
		"/state_machine/timer", 0);

	//slow down the loop
	ros::Rate loop_rate(10);

	//get the start time
	start_time = std::chrono::high_resolution_clock::now();

	while(ros::ok()) {

		if(!paused) {
			elapsed_time = (std::chrono::high_resolution_clock::now() -
				start_time);
		}

		//setup the message
		std_msgs::Int16 msg;
		msg.data = (int) (std::chrono::duration_cast<std::chrono::milliseconds>
			(elapsed_time).count() / 1000);

		//publish the message
		timer_publisher.publish(msg);

		loop_rate.sleep();

		//listen for callbacks
		ros::spinOnce();

	}

	return 0;
}

void user_control_callback(const std_msgs::String& message) {

	if(std::string(message.data).compare("start") == 0 && paused) {

		start_time = std::chrono::high_resolution_clock::now();
		paused = false;

	} else if(std::string(message.data).compare("stop") == 0) {

		paused = true;

	}
}
