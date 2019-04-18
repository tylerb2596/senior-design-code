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
#include "nav_msgs/Odometry.h"
#include "state_machine/control.h"
#include "std_msgs/String.h"
#include <math.h>

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

    //publisher for sending confirmation messages
    ros::Publisher confirmation_publisher;

    //subscriber object array
    ros::Subscriber sub[2];

    //linear and angular components of the velocity
    geometry_msgs::Vector3 lin;
    geometry_msgs::Vector3 ang;

    //current orientation (angle in radians relative
    //to the starting orientation)
    double current_orientation;

    //current position
    double x_position;
    double y_position;

    //origin locations
    double x_origin;
    double y_origin;
    double origin_orientation;

    //current covariance for the positions
    double current_x_covariance;
    double current_y_covariance;

    //current covariance with the orientation
    double current_orientation_covariance;

    //variable to stop movement in the middle of a loop
    bool stop;

    //private function for converting degrees to radians
    double degs_to_rads(double ang);

    //private function to calculate distance
    double distance(double x1, double y1, double x2, double y2);

    //private fucntion to convert quarternian position to yaw
    double quar_to_yaw(const geometry_msgs::Quaternion & quart);

    //private function to handle incoming odometry messages
    void handle_odometry(const nav_msgs::Odometry & odom);

    //private function to handle incoming control messages
    void handle_controls(const state_machine::control & control);

    //private function to do mod on doubles
    double double_mod(double num, double mod);

    //private function to make current_orientation settle
    void settle();

    //private function to orient towards the initial orientation
    void original_orientation();

    //function to move the desired angle in return to home
    void return_home();

//public member functions
public:

    //constructor for the class
    rmdc_mover(const ros::NodeHandle & nh);

    //rotate by deg degrees counterclockwise
    void rotate(double deg);

    //move forward for a given distance
    void move_forward(double distance);

    //move forward until told to stop
    void move_forward();

    //function to reset the "origin" of the robot so that it can start the
    //algorithm over again
    void reset();

    //function to get the robots current distance from its percieved origin
    double distance_from_origin();

};

#endif
