#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <state_machine/control.h>
#include <string>

double posX;
double posY;
double orientX;
double orientY;
double orientZ;
double orientW;

double origPosX;
double origPosY;
double origRotX;
double origRotY;
double origRotZ;
double origRotW;

bool startNow = false;
bool goBack = false;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void handleState(const state_machine::control& msg) {
  std::string str = msg.message;
  if (str.compare("start_home") == 0) {
	  startNow = true;
  }
  if (str.compare("return_home") == 0) {
	  goBack = true;
  }
}

void handleOdometry(const nav_msgs::Odometry::ConstPtr& msg) {
  posX = msg->pose.pose.position.x;
  posY = msg->pose.pose.position.y;
  orientX = msg->pose.pose.orientation.x;
  orientY = msg->pose.pose.orientation.y;
  orientZ = msg->pose.pose.orientation.z;
  orientW = msg->pose.pose.orientation.w;
}

int main(int argc, char** argv){
  //saving initial position
  ros::init(argc, argv, "simple_navigation_goals");

  // Initialize the transform listener:
  //tf::TransformListener listener;
  //tf::StampedTransform transform;

  ros::NodeHandle n;
  
  ros::Subscriber stateSub = n.subscribe("/state_machine/control", 1000, handleState);
  ros::Subscriber odomSub = n.subscribe("odom", 1000, handleOdometry);
  ros::Publisher returnPub = n.advertise<std_msgs::String>("/state_machine/confirmation", 1000);
  ros::Rate loop_rate(10);

  // Initialize costmap
  // costmapcostmap_2d::Costmap2DROS costmap("my_costmap", tf);

  // Tell the action client that we want to spin a thread by default:
  MoveBaseClient ac("move_base", true);

  // Wait for the action server to come up:
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  while(ros::ok()) {
	  if (startNow) {
	  //get original orientation and position
		ros::spinOnce();
		origPosX = posX;
		origPosY = posY;
		origRotX = orientX;
		origRotY = orientY;
		origRotZ = orientZ;
		origRotW = orientW; 

		ros::Duration(3.0).sleep();
		startNow = false;
	  }
	  
	  
	  
	  
	  //return code

	  move_base_msgs::MoveBaseGoal goal;

	  // Goal 1:
	  //goal.target_pose.header.frame_id = "base_link";
	  //goal.target_pose.header.stamp = ros::Time::now();

	  //goal.target_pose.pose.position.x = 1.0;
	  //goal.target_pose.pose.position.y = 0.5;
	  //goal.target_pose.pose.orientation.w = 1.0;

	  //ROS_INFO("Sending goal");
	  //ac.sendGoal(goal);

	  //ac.waitForResult();

	  //if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		//ROS_INFO("Goal 1 successful!!");
	  //else
		//ROS_INFO("Goal 1 FAILED...");

	  
	  // Goal 2:
	  //goal.target_pose.header.frame_id = "base_link";
	  //goal.target_pose.header.stamp = ros::Time::now();

	  //goal.target_pose.pose.position.x = 1.0;
	  //goal.target_pose.pose.position.y = -0.5;
	  //goal.target_pose.pose.orientation.w = 1.0;

	  //ROS_INFO("Sending goal");
	  //ac.sendGoal(goal);

	  //ac.waitForResult();

	  //if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		//ROS_INFO("Goal 2 successful!!");
	  //else
		//ROS_INFO("Goal 2 FAILED...");

	  // Save the initial position and orientation:
	  //ros::Time now = ros::Time::now();
	  //listener.waitForTransform("odom", "base_footprint", now, ros::Duration(10.0));
	  //listener.lookupTransform("odom", "base_footprint", now, transform);
  
  
  
      if (goBack) {
		// Send a goal to the robot to return back "home" (set at beginning of code):
		goal.target_pose.header.frame_id = "odom";
		goal.target_pose.header.stamp = ros::Time::now();

		ros::spinOnce();
		goal.target_pose.pose.position.x = origPosX;
		goal.target_pose.pose.position.y = origPosY;
		//goal.target_pose.pose.position.x = transform.getOrigin().x();
		//goal.target_pose.pose.orientation = original_orientation;
		goal.target_pose.pose.orientation.w = origRotW;
		goal.target_pose.pose.orientation.x = origRotY;
		goal.target_pose.pose.orientation.y = origRotZ;
		goal.target_pose.pose.orientation.z = origRotW;
		//ROS_INFO("Sending goal");
		ac.sendGoal(goal);

		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		  ROS_INFO("Hooray, the base returned to home");
		else
		  ROS_INFO("The base failed to go home for some reason");
	  
		goBack = false;
		
		
		//publishes that it returned home
		std_msgs::String msg;

		std::stringstream ss;
		ss << "return_done";
		msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
		returnPub.publish(msg);
	  }
	  
	  ros::spinOnce();
	  loop_rate.sleep();

    }
  
  return 0;
}
