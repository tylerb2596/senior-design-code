#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  // Initialize the transform listener:
  tf::TransformListener listener;
  tf::StampedTransform transform;

  // Tell the action client that we want to spin a thread by default:
  MoveBaseClient ac("move_base", true);

  // Wait for the action server to come up:
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //get original orientation
  //Quaternion original_orientation = transform.getRotation();

  move_base_msgs::MoveBaseGoal goal;

  // Send a goal to the robot to move 1 meter forward:
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  // Save the initial position and orientation:
  ros::Time now = ros::Time::now();
  listener.waitForTransform("odom", "base_footprint", now, ros::Duration(10.0));
  listener.lookupTransform("odom", "base_footprint", now, transform);

  // Send a goal to the robot to return back "home" (set at beginning of code):
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = transform.getOrigin().x();
  goal.target_pose.pose.orientation.w = 1.0;
  //goal.target_pose.pose.orientation = original_orientation;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base returned to home");
  else
    ROS_INFO("The base failed to go home for some reason");


  return 0;
}

