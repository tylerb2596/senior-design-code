#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

int main(int argc, char** argv) {

   //initialize the ros system
   ros::init(argc, argv, "circle");

   //create the rod node to use later
   ros::NodeHandle nh;

   //make the publisher for the velocity messages
   ros::Publisher circle_commands = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

   //set the ros loop rate
   ros::Rate loop_rate(1);

   //vector 3 variables so that they dont have to be made each loop
   geometry_msgs::Vector3 lin;
   lin.x = 2.0;
   lin.y = 0.0;
   lin.z = 0.0;

   geometry_msgs::Vector3 ang;
   ang.x = 0.0;
   ang.y = 0.0;
   ang.z = 1.8;

   while(ros::ok()) {

      //make a message object to send
      geometry_msgs::Twist message;

      message.linear = lin;
      message.angular = ang;


      circle_commands.publish(message);

      ros::spinOnce();

      loop_rate.sleep();

   }

   return 0;

}
