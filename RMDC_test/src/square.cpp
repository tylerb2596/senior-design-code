#include "ros/ros.h"
#include "rmdc_mover.h"

int main(int argc, char** argv) {

   //initialize the ros node
   ros::init(argc, argv, "square");

   //create the ros node to use later
   ros::NodeHandle nh;

   //make the mover object
   rmdc_mover mover(nh);

   ros::Duration(30).sleep();
   mover.rotate(90);
   ros::Duration(5).sleep();
   mover.rotate(90);

   return 0;

}


