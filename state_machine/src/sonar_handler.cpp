//SBD Metallic Debris Collector Project
//Stanley Black and Decker
//Georgia Institute of Technology
//created by Tyler Brown

//File to take in all the sonar values and
//decide the best course of action based on their collective
//values.

#ifndef SONAR_HANDLER
#define SONAR_HANDLER

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"
#include "../include/rmdc_state.h"
#include <iostream>
#include <time.h>
#include <stdlib.h>


class sonar_handler : public rmdc_state {

//public members of the class
public:

    //constructor
    sonar_handler() : threshold(1.00) {

        for(int i = 0; i < 5; ++i) {
            (this -> previous_sonar)[i] = 0;
            (this -> previous_sonar)[i] = 0;
        }

        srand(time(NULL));

    }

    //members of rmdc_state

    //used to start this node
    virtual void run() {

         //slow down the loop
        ros::Rate loop_rate(10);

        while(ros::ok()) {

            //slow down the loop
            loop_rate.sleep();

            //wait for callback messages
            ros::spinOnce();

            //analyze the sonar data and recommend action
            analyze_sonar();
        }

    }

    //used to initialize this node
    virtual void init() {

        //setup the publishers and subscribers

        //subscribe to each of the sonar topics
        this -> sonar0_subscriber = (this -> nh).subscribe(
            "/pi_sonar/sonar_0", 0, &sonar_handler::sonar_callback_0, this);

        this -> sonar1_subscriber = (this -> nh).subscribe(
            "/pi_sonar/sonar_1", 0, &sonar_handler::sonar_callback_1, this);

        this -> sonar2_subscriber = (this -> nh).subscribe(
            "/pi_sonar/sonar_2", 0, &sonar_handler::sonar_callback_2, this);

        this -> sonar3_subscriber = (this -> nh).subscribe(
            "/pi_sonar/sonar_3", 0, &sonar_handler::sonar_callback_3, this);

        this -> sonar4_subscriber = (this -> nh).subscribe(
            "/pi_sonar/sonar_4", 0, &sonar_handler::sonar_callback_4, this);

        //publish to a topic to be read by the decision state
        this -> sonar_decision_publisher = (this -> nh).
            advertise<std_msgs::Float32>("/state_machine/sonar", 0);


    }

//private members of the class
private:

    //sonar subscribers
    ros::Subscriber sonar0_subscriber;

    ros::Subscriber sonar1_subscriber;

    ros::Subscriber sonar2_subscriber;

    ros::Subscriber sonar3_subscriber;

    ros::Subscriber sonar4_subscriber;

    //publisher for decision state to read
    ros::Publisher sonar_decision_publisher;

    //array to hold the previous sonar values
    double previous_sonar[5];

    //array to hold the current sonar values
    double current_sonar[5];

    //array to hold booleans indicating if the
    //sonar value is decreasing
    bool decreasing[5];

    //array to hold boolean values indication if the
    //sonar value is within the threshold
    bool within_threshold[5];

    //threshold that is too close for the robot to an obstacle
    const double threshold;

    //handlers for the sonar sensors
    void sonar_callback_0(const sensor_msgs::Range& message) {

        (this -> previous_sonar)[0] = (this -> current_sonar)[0];
        (this -> current_sonar)[0] = message.range;
    }

    void sonar_callback_1(const sensor_msgs::Range& message) {

        (this -> previous_sonar)[1] = (this -> current_sonar)[1];
        (this -> current_sonar)[1] = message.range;

    }

    void sonar_callback_2(const sensor_msgs::Range& message) {

        (this -> previous_sonar)[2] = (this -> current_sonar)[2];
        (this -> current_sonar)[2] = message.range;

    }

    void sonar_callback_3(const sensor_msgs::Range& message) {

        (this -> previous_sonar)[3] = (this -> current_sonar)[3];
        (this -> current_sonar)[3] = message.range;
    }

    void sonar_callback_4(const sensor_msgs::Range& message) {

        (this -> previous_sonar)[4] = (this -> current_sonar)[4];
        (this -> current_sonar)[4] = message.range;

    }

    //function to decide if each distance is decreasing or not
    void is_decreasing() {

        for(int i = 0; i < 5; ++i) {
            if((this -> current_sonar)[i] < (this -> previous_sonar)[i]) {
                (this -> decreasing)[i] = true;
            } else {
                (this -> decreasing)[i] = false;
            }
        }

    }

    //function to decide if each distance is decreasing or not
    void is_below_threshold() {

        for(int i = 0; i < 5; ++i) {
            if (i == 0 || i == 4) {
                if((this -> current_sonar)[i] < 0.75 && (this -> current_sonar)[i] > 0.2) {
                (this -> within_threshold)[i] = true;
                } else {
                    (this -> within_threshold)[i] = false;
                }
                continue;
            }
            if((this -> current_sonar)[i] < this -> threshold && (this -> current_sonar)[i] > 0.2) {
                (this -> within_threshold)[i] = true;
            } else {
                (this -> within_threshold)[i] = false;
            }
        }

    }

    //function to determine a random angle to turn to
    double determine_random_angle(float upper, float lower) {

        //get the difference between the two
        float diff = upper - lower;

        //return a random number that is a multiple of 10
        return ((rand() % (int)diff) + lower)*10;
    }

    //function to analyse the sonar data and recommend action
    //to the decision node
    void analyze_sonar() {

        //first check threshold and decreasing
        this -> is_below_threshold();
        this -> is_decreasing();

        //now analyze this data and decide on action
        float lower_bound = 0.0;
        float upper_bound = 36.0;

        if ((this -> within_threshold)[0] && (this -> decreasing)[0]) {
            upper_bound = 27.0;
        }

        if ((this -> within_threshold)[1] && (this -> decreasing)[1]) {
            if(upper_bound > 31.5) {
                upper_bound = 31.5;
            }
        }

        if ((this -> within_threshold)[2] && (this -> decreasing)[2]) {

            if(lower_bound < 4.5) {
                lower_bound = 4.5;
            }
        }

        if ((this -> within_threshold)[3] && (this -> decreasing)[3]) {
            if(lower_bound < 9.0) {
                lower_bound = 9.0;
            }
        }

        if ((this -> within_threshold)[4] && (this -> decreasing)[4]) {
            if(lower_bound < 18.0) {
                lower_bound = 18.0;
            }
        }

        //if there is no reason to turn then dont send a message
        if (std::abs(upper_bound - 36.0) <= 0.1 && std::abs(0.0 - lower_bound) <= 0.1) {
            return;
        }

        //send a messgae otherwise
        std_msgs::Float32 msg;
        msg.data = this -> determine_random_angle(upper_bound, lower_bound);
        this -> sonar_decision_publisher.publish(msg);

        ros::Duration(1).sleep();
    }

};

//main function to setup and run the sonar handler
int main(int argc, char** argv) {

    //initialize ros with the command line arguments
    ros::init(argc, argv, "sonar_handler");

    //create a sonar_handler object
    sonar_handler sonar;

    //intialize the sonar handler
    sonar.init();

    //run the sonar handler
    sonar.run();

    return(0);
}

#endif

