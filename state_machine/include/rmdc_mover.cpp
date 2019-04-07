//SBD Metallic Debris Collector Project
//Stanley Black and Decker
//Georgia Institute of Technology
//created by Tyler Brown


//Implementation of the class members in rmdc_mover class

#include "rmdc_mover.h"
#include <iostream>
#include <string>

//public members

//make the rmdc_move object
rmdc_mover::rmdc_mover(const ros::NodeHandle & nh) : node(nh),
max_rotation_speed(M_PI/8), forward_speed(0.3), stop(true) {

    //subscribe to topic sending out odometry messages
    this -> sub[0] = node.subscribe("odom", 1,
        &rmdc_mover::handle_odometry, this);

    //subscribe to topic sending control messages
    this -> sub[1] = node.subscribe("/state_machine/control", 10,
        &rmdc_mover::handle_controls, this);


    //make the publisher object
    this -> pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 0);

    //make the confirmation publisher
    this -> confirmation_publisher = node.advertise<std_msgs::String>(
        "/state_machine/confirmation", 0);

    //start current orientation is always zero
    this -> current_orientation = 0.0;

    //set the initial values to zero for the velocities
    this -> lin.x = 0.0;
    this -> lin.y = 0.0;
    this -> lin.z = 0.0;

    this -> ang.x = 0.0;
    this -> ang.y = 0.0;
    this -> ang.z = 0.0;

    //set the covariance to 0.2 at first
    this -> current_orientation_covariance = 0.2;

}

//function to reset the origin of the robot
void rmdc_mover::reset() {

    //spin ros to get the call back from odometry
    ros::spinOnce();

    //set the origin to the current location
    this -> x_origin = this -> x_position;
    this -> y_origin = this -> y_position;

}

//function to get the current distance of the robot from the origin
double rmdc_mover::distance_from_origin() {

    return this -> distance(this -> x_origin, this -> y_origin,
        this -> x_position, this -> y_position);
}

//fucntion to rotate counterclockwise a given angle
void rmdc_mover::rotate(double angle) {

    //spin once
    ros::spinOnce();

    //if the angle is higher than 360 then reduce it to 360
    angle = double_mod(angle, 360);

    //find out how many radians we need to rotate
    double rad_angle = this -> degs_to_rads(angle);

    //determine goal angle to reach
    double goal_angle = this -> current_orientation + rad_angle;

    double tolerance = std::abs(goal_angle - 2*M_PI) < 0.2 ?
        std::abs(goal_angle - 2*M_PI) : 0.2;

    //make a message to send
    geometry_msgs::Twist msg;

    //set the zero values
    this -> lin.x = 0.0;
    this -> lin.y = 0.0;
    this -> lin.z = 0.0;

    this -> ang.x = 0.0;
    this -> ang.y = 0.0;

    //slow down the loop
    ros::Rate loop_rate(5);

    //correct for the angular periodicity
    goal_angle = goal_angle > 2*M_PI ? goal_angle - 2*M_PI : goal_angle;

    while (rad_angle > tolerance && ros::ok()
        && !(this -> stop)) {

        if (rad_angle > this -> max_rotation_speed) {
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

     //send confirmation
    std_msgs::String con_message;
    con_message.data = "turn";
    this -> confirmation_publisher.publish(con_message);

    //make sure to wait 1 second before doing anything else
    ros::Duration(1).sleep();
}

//functions to make the robot move forward a specified distance in meters
void rmdc_mover::move_forward(double distance_to_move) {

    //spin once
    ros::spinOnce();

    //original position before movement
    double x_pos = this -> x_position;
    double y_pos = this -> y_position;

    //make a message to send
    geometry_msgs::Twist msg;

    //set the zero values
    this -> lin.y = 0.0;
    this -> lin.z = 0.0;

    this -> ang.x = 0.0;
    this -> ang.y = 0.0;
    this -> ang.z = 0.0;

    //slow down the loop
    ros::Rate loop_rate(5);

    while (distance_to_move > 0.01 && ros::ok() && !(this -> stop)) {

        if (distance_to_move > this -> forward_speed) {
            this -> lin.x = forward_speed;
            msg.angular = this -> ang;
            msg.linear = this -> lin;
            pub.publish(msg);
        } else {
            this -> lin.x = distance_to_move;
            msg.angular = this -> ang;
            msg.linear = this -> lin;
            pub.publish(msg);
        }

        loop_rate.sleep();
        this -> settle();

        distance_to_move = distance_to_move - distance(x_pos, y_pos,
            this -> x_position, this -> y_position);

    }

    //make sure to wait 1 second before doing anything else
    ros::Duration(1).sleep();
}


//move forward until told to stop
void rmdc_mover::move_forward() {

    //spin once
    ros::spinOnce();

    //make a message to send
    geometry_msgs::Twist msg;

     //set the zero values
    this -> lin.x = this -> forward_speed;
    this -> lin.y = 0.0;
    this -> lin.z = 0.0;

    this -> ang.x = 0.0;
    this -> ang.y = 0.0;
    this -> ang.z = 0.0;

    msg.linear = this -> lin;
    msg.angular = this -> ang;

    ros::Rate loop_rate(5);

    while (ros::ok() && !(this -> stop)) {

        pub.publish(msg);
        loop_rate.sleep();

        ros::spinOnce();
    }

    //make sure to wait  second before doing anything else
    ros::Duration(1).sleep();

}

//private members

//calculate the distance between two points
double rmdc_mover::distance(double x1, double y1, double x2, double y2) {

    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2)) / 4.0;
}

//convert degrees to radians
double rmdc_mover::degs_to_rads(double angle) {

    //1 radian = PI/180 degrees
    return angle * (M_PI / 180);
}

//convert quarternian coordinates to yaw
double rmdc_mover::quar_to_yaw(const geometry_msgs::Quaternion & quart) {

    //convert quarternian orientation to yaw
    double siny_cosp = 2.0 * (quart.w * quart.z + quart.x * quart.y);
    double cosy_cosp = 1.0 - 2.0 * (quart.y * quart.y + quart.z * quart.z);
    double yaw = atan2(siny_cosp, cosy_cosp);

    return yaw;
}

//TODO: change this function to handle ying yings messages
//handle messages coming from the odometry
void rmdc_mover::handle_odometry(const nav_msgs::Odometry & odom) {

    //set the current orientation to the odometry's estimation of it
    double yaw = quar_to_yaw(odom.pose.pose.orientation);

    if (yaw < 0) {
        yaw = yaw + 2*M_PI;
    }


    //values for the position of the robot in free space
    this -> x_position = odom.pose.pose.position.x;

    this -> y_position = odom.pose.pose.position.y;

    this -> current_x_covariance = odom.pose.covariance[0];

    this -> current_y_covariance = odom.pose.covariance[1];


    //values for the orientation of the robot
    this -> current_orientation = yaw;

    this -> current_orientation_covariance = odom.pose.covariance[35];

}


//handle messages coming from control
void rmdc_mover::handle_controls(const state_machine::control & control) {

    std::string message(control.message);

    if (message.compare("stop") == 0) {

        this -> stop = true;

    } else if (message.compare("start") == 0) {

        this -> stop = false;
    }
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


    return how_many * std::abs(mod);

}

//function to make sure the orientation is up to date
void rmdc_mover::settle() {

    double beginning_orientation = this -> current_orientation;
    double beginning_x = this -> x_position;
    double beginning_y = this -> y_position;

    int count = 0;

    while(count < 20 && ros::ok()) {
        if (beginning_orientation == this -> current_orientation
            && beginning_x == this -> x_position
            && beginning_y == this -> y_position) {
            count++;
        } else {
            count = 0;
        }

        ros::spinOnce();

        beginning_orientation = this -> current_orientation;
        beginning_x = this -> x_position;
        beginning_y = this -> y_position;

    }

}