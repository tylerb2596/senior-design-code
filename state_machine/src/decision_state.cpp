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
#include "sensor_msgs/Range.h"
#include "rmdc_class.h"
#include <string>
#include <iostream>

//class to be used as a container for the decision state methods, callbacks and
//data

class decision_state : public rmdc_state {

//public members
public:

    //constructor
    decision_state(int how_many) : num_sonars(how_many),
        dump_interval(60), return_to_home_interval(300) {}

    //initialize the node
    virtual void init(int argc, char** argv) {

        //initialize ros with the command line arguments
        ros::init(argc, argv, "decision_state");

        //setup publisher and subscriber

        //subscribe to sonar messages based on the number of sonars used
        for(int i = 0; i < this -> num_sonars; ++i) {

            std::string temp_string = "/pi_sonar/sonar_";
            temp_string.append(std::string((char)i));

            this -> sonar_subscribers.push_back(
                (this -> nh).subscribe(temp_string.c_str(), 10,
                    &sonar_callback, this));

        }

        //subscribe to timer messages
        this -> timer_subscriber = (this -> nh).subscribe(
            "/state_machine/timer", 10, &timer_callback, this);

        //set up the publisher to send control messages
        this -> publisher = (this -> nh)
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

    //publisher for control messages
    ros::Publisher decision_publisher;

    //how many sonars being used
    int num_sonars;

    //elapsed time
    int elapsed_time;

    //callback function for the sonar array
    void sonar_callback(const sensor_msgs::Range& message) {

    }

    //callback function for timer interrupts
    void timer_callback(const std_msgs::Int16& message) {
        this -> elapsed_time = message.data;
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