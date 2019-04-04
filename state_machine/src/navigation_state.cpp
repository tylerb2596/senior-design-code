//SBD Metallic Debris Collector Project
//Stanley Black and Decker
//Georgia Institute of Technology
//created by Tyler Brown

//Conceptually this file defines the navigation state for out robot. Practically
//this file defiens a class which is used to execute
//all of the system commands needed for successful navigation of the robot.

#ifndef NAVIGATION_STATE
#define NAVIGATION_STATE

#include "ros/ros.h"
#include "../include/rmdc_mover.h"
#include "../include/rmdc_state.h"
#include "state_machine/control.h"
#include "std_msgs/Float32.h"

class navigation_state : public rmdc_state {

//public members of the class
public:

    //constructor
    navigation_state() : mover(this -> nh) {}

    //function that is used to run the main loop of the node
    virtual void run() {

        //slow down the loop
        ros::Rate loop_rate(10);

        while(ros::ok()) {

            //slow down the loop
            loop_rate.sleep();

            //wait for callbacks
            ros::spinOnce();

            //check the distance travelled and publish it
            this -> check_distance();

        }

    }

    //function used to initialize the node, subscribe to any topics and set up
    //publishers
    virtual void init() {

        //setup publishers and subscribers

        //initialize the control subscriber
        this -> control_subscriber = (this -> nh).subscribe(
            "/state_machine/control", 10,
            &navigation_state::control_callback, this);

        //intitialize the distance publisher
        this -> distance_publisher = (this -> nh).advertise<std_msgs::Float32>(
            "/state_machine/distance", 0);

    }

private:

    //subscribe to control messages
    ros::Subscriber control_subscriber;

    //publisher to send out distance updates
    ros::Publisher distance_publisher;

    //mover object to make the robot move using the rmdc_mover class
    rmdc_mover mover;

    //distance traveled away from origin
    double distance;

    //check the distance from the origin
    void check_distance() {

        //update the distance traveled away from the origin
        this -> distance = mover.distance_from_origin();

        //publish the distance to the topic
        std_msgs::Float32 msg;
        msg.data = this -> distance;
        this -> distance_publisher.publish(msg);

    }

    //control subscriber callback
    void control_callback(const state_machine::control& message) {

        //convert the string par tof th message to a string
        std::string string_message(message.message);

        //retrieve the number part of the message
        double number_message = message.num;

        //handle the message
        if (string_message.compare("forward") == 0) {

            //move the robot forward
            //this will block this node until its done
            this -> mover.move_forward();

        } else if(string_message.compare("forward") == 0
            && number_message != 0) {

            //move the robot forward the specified distance
            //this will block this node until its done
            this -> mover.move_forward(number_message);

        } else if(string_message.compare("turn") == 0) {

            //turn the robot the specified amount
            //this will block until its finished
            this -> mover.rotate(number_message);

        }
    }



};

//main function to setup and run the navigation state
int main(int argc, char** argv) {

    //initialize ros with the command line arguments
    ros::init(argc, argv, "decision_state");

    //create a navigation state object
    navigation_state n_state;

    //initialize the navigation state
    n_state.init();

    //run the navigation state
    n_state.run();

    return(0);
}


#endif