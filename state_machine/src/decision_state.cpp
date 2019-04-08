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
        correcting(false), returning_home(false), dumping(false),
        user_stopped(true) {
            srand(time(NULL));
        }

    //constructor
    decision_state(int how_many, double rad) : num_sonars(how_many),
        dump_interval(60), return_to_home_interval(300), radius(rad),
        correcting(false), returning_home(false), dumping(false),
        user_stopped(true){
            srand(time(NULL));
        }

    //initialize the node
    virtual void init() {

        //setup publisher and subscriber

        //subscribe to the sonar_handler node messages
        this -> sonar_subscriber = (this -> nh). subscribe (
            "/state_machine/sonar", 10, &decision_state::sonar_callback, this);

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
            "/state_machine/confirmation", 10,
            &decision_state::confirmation_callback, this);

        //set up the publisher to send control messages
        this -> decision_publisher = (this -> nh)
            .advertise<state_machine::control>("/state_machine/control", 0);

    }

    virtual void run () {

        //slow down the loop
        ros::Rate loop_rate(100);

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
    // std::vector<ros::Subscriber> sonar_subscribers;
    ros::Subscriber sonar_subscriber;

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
    //interrupting an adjustment in progress
    bool correcting;

    //variable to act as a sort of mutex lock to prevent the sonars from
    //interrupting an the return home algorithm
    bool returning_home;

    //variable to act as a sort of mutex lock to prevent the sonars from
    //interrupting an the dumping algorithm
    bool dumping;

    //variable so the robot can know when it has been stopped by the user
    bool user_stopped;

    //callback function for the sonar array
    void sonar_callback(const std_msgs::Float32& message) {

        //if a correction is in progress then dont
        if (this -> user_stopped || this -> correcting || this -> dumping
            || this -> returning_home) {
            return;
        } else {

            this -> sonar_adjustment(message.data);
        }
    }

    //callback function for timer interrupts
    void timer_callback(const std_msgs::Int16& message) {
        this -> elapsed_time = message.data;

        state_machine::control msg;
        msg.message = "home";
        msg.num = 0;

        //if the algorithm is finished send a message to return home and
        //wait to start over again
        if (this -> elapsed_time >= this -> return_to_home_interval) {

            //avoid sonar interrupts
            this -> returning_home = true;
            //dont let the movement API to interfere
            this -> start_stop_message("stop");
            //when done dont let the sonars affect anything
            this -> user_stopped = true;
            //tell the robot to return home
            this -> decision_publisher.publish(msg);

            //slow down the loop
            ros::Rate loop_rate(10);

            //wait until the robot is home
            while(this -> returning_home) {
                loop_rate.sleep();
                ros::spinOnce();
            }

            //tell the robot to dump its load
            msg.message = "drop";
            this -> decision_publisher.publish(msg);
            this -> dumping = true;

        } else if (this -> elapsed_time >= this -> dump_interval) {

            //avoid sonar interrupts
            this -> returning_home = true;
            //dont let the movement API to interfere
            this -> start_stop_message("stop");
            //tell the robot to go home
            this -> decision_publisher.publish(msg);

            //slow down the loop
            ros::Rate loop_rate(10);

            //wait until the robot is home
            while(this -> returning_home) {
                loop_rate.sleep();
                ros::spinOnce();
            }

            //tell the robot to dump its load
            msg.message = "drop";
            this -> decision_publisher.publish(msg);
            this -> dumping = true;

            //wait until the robot has dumped its load
            while(this -> dumping) {
                loop_rate.sleep();
                ros::spinOnce();
            }

            this -> start_stop_message("start");

        }
    }

    //callback function for the user_control messages
    void user_control_callback(const std_msgs::String& message) {

        //convert the message to a string
        std::string string_message(message.data);

        //determine the correct message to send
        if (string_message.compare("stop") == 0) {

            this -> start_stop_message("stop");
            this -> user_stopped = true;
            this -> correcting = false;

        } else if(string_message.compare("start") ==0) {

            this -> start_stop_message("start");
            this -> forward_message();
            this -> user_stopped = false;
        }

    }

    //callback function for the distance messages
    void distance_callback(const std_msgs::Float32& message) {

        //extract the distance from the message
        double distance_traveled = message.data;

        //make sure the robot stays within the given radius
        if (distance_traveled >= this -> radius) {

            //first stop the robot
            this -> start_stop_message("stop");

            //next start the robot again
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
        //if it is home then we have returned home
        //if it is drop then we are done dumping the load
        if (con_message.compare("turn") == 0) {

            this -> correcting = false;

        } else if (con_message.compare("home") == 0) {

            this -> returning_home = false;

        } else if (con_message.compare("drop") == 0) {

            this -> dumping = false;

        }
    }

    //generate a random angle to turn to between 50 and 270
    //in multiples of 10
    int determine_random_angle() {

        //generate the number using rand and return it
        int rand_num = (rand() % 22) + 5;
        return rand_num * 10;

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

        //wait for a half second to ensure the message is executed
        while (this -> correcting) {
            ros::spinOnce();
        }

    }

    //function to make sending a forward messafe easier
    void forward_message(double how_far = 0) {

        //make a message
        state_machine::control msg;

        //fill up the message
        msg.message = "forward";
        msg.num = how_far;

        //send the message
        this -> decision_publisher.publish(msg);

        //wait for a half second to ensure the message is executed
        ros::Duration(1).sleep();
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

        //wait for a second to ensure the message is executed
        ros::Duration(1).sleep();
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