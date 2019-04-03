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
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Range.h"
#include "../include/rmdc_state.h"
#include <string>
#include <iostream>

//class to be used as a container for the decision state methods, callbacks and
//data

class decision_state : public rmdc_state {

//public members
public:

    //constructor
    decision_state(int how_many) : num_sonars(how_many),
        dump_interval(60), return_to_home_interval(300), radius(10) {}

    //constructor
    decision_state(int how_many, double rad) : num_sonars(how_many),
        dump_interval(60), return_to_home_interval(300), radius(rad) {}

    //initialize the node
    virtual void init(int argc, char** argv) {

        //initialize ros with the command line arguments
        ros::init(argc, argv, "decision_state");

        //setup publisher and subscriber

        //subscribe to sonar messages based on the number of sonars used
        for(int i = 0; i < this -> num_sonars; ++i) {

            std::string temp_string = "/pi_sonar/sonar_";
            temp_string.append(std::string(std::to_string(i)));

            this -> sonar_subscribers.push_back(
                (this -> nh).subscribe(temp_string.c_str(), 10,
                    &decision_state::sonar_callback, this));

        }

        //subscribe to timer messages
        this -> timer_subscriber = (this -> nh).subscribe(
            "/state_machine/timer", 10, &decision_state::timer_callback, this);

        //subscribe to user_control messages
        this -> user_control_subscriber = (this -> nh).subscribe(
            "/state_machine/user_control", 10,
            &decision_state::user_control_callback, this);

        //subscribe to the distance messages
        this -> distance_subscriber = (this -> nh).subscribe(
            "/state_machine/distance", 10,
            &decision_state::distance_callback, this);

        //set up the publisher to send control messages
        this -> decision_publisher = (this -> nh)
            .advertise<state_machine::control>("/state_machine/control", 0);

    }

    virtual void run () {

        //slow down the loop
        ros::Rate loop_rate(10);

        while(ros::ok()) {

            //slow down the loop
            loop_rate.sleep();

            //wait for callback messages
            ros::spin();
        }
    }

//private members
private:

    //constant parameters
    const int dump_interval;
    const int return_to_home_interval;

    //subscribe to sonar messages
    std::vector<ros::Subscriber> sonar_subscribers;

    //subscribe to timer messages
    ros::Subscriber timer_subscriber;

    //subscribe to user_control messages
    ros::Subscriber user_control_subscriber;

    //subscribe to the distance messages
    ros::Subscriber distance_subscriber;

    //publisher for control messages
    ros::Publisher decision_publisher;

    //how many sonars being used
    int num_sonars;

    //elapsed time
    int elapsed_time;

    //radius the robot is allowed to stay within in meters
    const double radius;

    //callback function for the sonar array
    void sonar_callback(const sensor_msgs::Range& message) {

    }

    //callback function for timer interrupts
    void timer_callback(const std_msgs::Int16& message) {
        this -> elapsed_time = message.data;
    }

    //callback function for the user_control messages
    void user_control_callback(const std_msgs::String& message) {

        //convert the message to a string
        std::string string_message(message.data);

        //create the message to send to the control topic
        state_machine::control msg;

        //determine the correct message to send
        if (string_message.compare("stop") == 0) {

            msg.message = "stop";

        } else if(string_message.compare("start") ==0) {

            msg.message = "start";
        }

        msg.num = 0;

        //publish the message
        this -> decision_publisher.publish(msg);

    }

    //callback function for the distance messages
    void distance_callback(const std_msgs::Float32& message) {

        //extract the distance from the message
        double distance_traveled = message.data;

        //make sure the robot stays within the given radius
        if (distance_traveled > this -> radius) {

            //create control message
            state_machine::control msg;

            //first stop the robot
            msg.message = "start";
            msg.num = 0;

            this -> decision_publisher.publish(msg);

            //determine a random angle to turn to and turn that way
            msg.message = "turn";
            msg.num = this -> determine_random_angle();

            this -> decision_publisher.publish(msg);


            //move forward
            msg.message = "forward";
            msg.num = 0;

            this -> decision_publisher.publish(msg);

        }
    }

    double determine_random_angle() {

    }

};


//main function to setup and run the decision state
int main(int argc, char ** argv) {

    //create a decision_state object
    decision_state d_state(6);

    //intialize the decision state
    d_state.init(argc, argv);

    //run the d_state
    d_state.run();

    return(0);
}

#endif