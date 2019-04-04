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
#include <time.h>
#include <stdlib.h>

//class to be used as a container for the decision state methods, callbacks and
//data

class decision_state : public rmdc_state {

//public members
public:

    //constructor
    decision_state(int how_many) : num_sonars(how_many),
        dump_interval(60), return_to_home_interval(300), radius(10),
        correcting(false) {
            srand(time(NULL));
        }

    //constructor
    decision_state(int how_many, double rad) : num_sonars(how_many),
        dump_interval(60), return_to_home_interval(300), radius(rad),
        correcting(false) {
            srand(time(NULL));
        }

    //initialize the node
    virtual void init() {

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

        //subscribe to confirmation messages
        this -> confirmation_subscriber = (this -> nh).subscribe(
            "/state_machine/distance", 10,
            &decision_state::confirmation_callback, this);

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

    //subscribe to confirmation messages
    ros::Subscriber confirmation_subscriber;

    //publisher for control messages
    ros::Publisher decision_publisher;

    //how many sonars being used
    int num_sonars;

    //elapsed time
    int elapsed_time;

    //radius the robot is allowed to stay within in meters
    const double radius;

    //variable to act as a sort of mutex lock to prevent the sonars from
    //interruoting an adjustment in progress
    bool correcting;

    //callback function for the sonar array
    void sonar_callback(const sensor_msgs::Range& message) {

        //if a correction is in progress then dont
        if (this -> correcting) {
            return;
        }

        //first get the frame_id of sonar that this message is from
        std::string frame_id = message.header.frame_id;

        int sonar_num = std::atoi(frame_id.substr(6, 1).c_str());

        //now switch on the sonar_num
        switch(sonar_num) {
            case 0:

                //sonar on the right side of the robot
                //if it gets too close to something turn 15 degrees
                if (message.range <= 1 && message.range > 0.2) {

                    this -> sonar_adjustment(15);

                }

                break;

            case 1:

                //sonar on the front of the robot facing 45 degrees to the
                //robots left
                //if something gets too close then turn between 90 and 270
                //degrees
                if (message.range <= 1 && message.range > 0.2) {

                    this -> sonar_adjustment(this -> determine_random_angle());

                }

                break;

            case 2:

                //sonar on the front of the robot facing 45 degrees to the
                //robots right
                //if something gets too close then turn between 90 and 270
                //degrees
                if (message.range <= 1 && message.range > 0.2) {

                    this -> sonar_adjustment(this -> determine_random_angle());

                }

                break;

            case 3:

                //sonar on the front of the robot
                //if it gets too close to something turn to a random angle
                //between 90 and 270 degrees
                if(message.range <= 1 && message.range > 0.2) {

                    this -> sonar_adjustment(this -> determine_random_angle());

                }

                break;

            case 4:

                //sonar on the left side of the robot
                //if it gets too close to something turn to an angle between 180
                //and 270 degrees
                if(message.range <= 1 && message.range > 0.2) {

                    this -> sonar_adjustment((rand() % 90) + 180);

                }

                break;
        }
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

            this -> start_stop_message("stop");

        } else if(string_message.compare("start") ==0) {

            this -> start_stop_message("start");
        }

    }

    //callback function for the distance messages
    void distance_callback(const std_msgs::Float32& message) {

        //extract the distance from the message
        double distance_traveled = message.data;

        //make sure the robot stays within the given radius
        if (distance_traveled > this -> radius) {

            //first stop the robot
            this -> start_stop_message("start");

            //determine a random angle to turn to and turn that way
            this -> turn_message(this -> determine_random_angle());

            //move forward
            this -> forward_message();

        }
    }

    //callback function for the confirmation messages
    void confirmation_callback(const std_msgs::String& message) {

        //get the message as a string
        std::string con_message(message.data);

        //if the message is turn then we are done correcting
        if (con_message.compare("turn") == 0) {

            this -> correcting = false;

        }
    }

    //generate a random angle to turn to between 90 and 270
    int determine_random_angle() {

        //generate the number using rand and return it
        return (rand() % 80) + 190;

    }

    //function to send a turn message more easily
    void turn_message(int angle) {

        //make a message
        state_machine::control msg;

        //fill up the message
        msg.message = "turn";
        msg.num = angle;

        //send the message
        this -> decision_publisher.publish(msg);


    }

    //function to make sending a forward messafe easier
    void forward_message(int how_far = 0) {

        //make a message
        state_machine::control msg;

        //fill up the message
        msg.message = "forward";
        msg.num = how_far;

        //send the message
        this -> decision_publisher.publish(msg);
    }

    //function to make sending start/stop messages easier
    void start_stop_message(const char* star_sto) {

        //make a message
        state_machine::control msg;

        //fill up the message
        msg.message = star_sto;
        msg.num = 0;

        //send the message
        this -> decision_publisher.publish(msg);
    }

    //function to abstract out sonar adjustment code
    void sonar_adjustment(int angle) {

        //tell the state that we are currently correcting
        this -> correcting = true;

        //stop the robot from doing what its doing
        this -> start_stop_message("stop");

        //enable the movement api again
        this -> start_stop_message("start");

        //turn to the specified angle
        this -> turn_message(angle);

        //continue moving forward
        this -> forward_message();

    }

};


//main function to setup and run the decision state
int main(int argc, char ** argv) {

    //initialize ros with the command line arguments
    ros::init(argc, argv, "decision_state");

    //create a decision_state object
    decision_state d_state(5);

    //intialize the decision state
    d_state.init();

    //run the d_state
    d_state.run();

    return(0);
}

#endif