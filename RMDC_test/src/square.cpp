#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "math.h"


class Mover {

private:
    ros::Publisher pub;
    geometry_msgs::Twist msg;
    float x_vel;
    float ang_vel;

public:
    Mover(const ros::Publisher& publisher) : pub(publisher), x_vel(0.0), ang_vel(M_PI / 2) {}

    void rotate(float angle) {

        float radian_rotation = angle * (M_PI / 180);
        float time_rotation = (2 / M_PI) * radian_rotation;

        geometry_msgs::Vector3 lin;
        geometry_msgs::Vector3 ang;

        lin.x = lin.y = lin.z = 0.0;

        ang.x = ang.y = 0.0;

        ang.z = ang_vel;

        msg.linear = lin;
        msg.angular = ang;


        ros::Time beginTime = ros::Time::now();
        ros::Duration send_time = ros::Duration(time_rotation);
        ros::Time endTime = beginTime + send_time;
        ros::Rate loop_rate(2);
        
        while(ros::Time::now() < endTime) {

            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
            //ros::Duration(0.5).sleep();
        }
               
    }

    void move(float distance) {

    }

};


int main(int argc, char** argv) {

   //initialize the ros node
   ros::init(argc, argv, "square");

   //create the ros node to use later
   ros::NodeHandle nh;

   //make the publisher for the velocity messages
   ros::Publisher square_command = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);

   //make the mover object
   Mover mover(square_command);

   //set the ros loop rate
   //ros::Rate loop_rate(1);

//   mover.rotate(90);
   //ros::Duration(10).sleep();
  // mover.rotate(90);
  // mover.rotate(90);
  // mover.rotate(90);
     mover.rotate(360);
    // ros::Duration(5).sleep();
//     mover.rotate(90);

   return 0;

}


