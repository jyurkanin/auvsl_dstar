#include <iostream>
#include <thread>
#include <vector>

#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "GlobalParams.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <rbdl/rbdl.h>



using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

//using namespace auvsl_planner;

//launch-prefix="gdb -ex run --args

//This function is for later on when other nodes may want to request a global path plan
//bool globalPlannerCallback(GlobalPathPlan::Request &req, GlobalPathPlan::Response &resp){
//This functionality is provided by the move_base/navigation packages.


unsigned got_init_pose = 0;
geometry_msgs::Pose initial_pose;

void get_pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  initial_pose = msg->pose.pose;
  got_init_pose = 1;
}


//<node name="test_cs" pkg="auvsl_planner" type="test_cs_node" output="screen"/>
//1. Solve Global Path Plan
//2. Run Local Path Planner and try to follow global path plan

int main(int argc, char **argv){
  ROS_INFO("Starting up auvsl_planner_node\n");
  
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  ros::Rate loop_rate(10);

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  geometry_msgs::Twist msg;
  
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
  
  cmd_vel_pub.publish(msg);
  
  ros::ServiceClient localization_srv = nh.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_localization");
  std_srvs::Empty empty_srv;
  localization_srv.waitForExistence();
  if(localization_srv.call(empty_srv)){
      ROS_INFO("Localization mode set");
  }
  else{
      ROS_INFO("Failed to set Localization mode set");
  }
  
  ROS_INFO("ABOUT TO WAIT");
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_client("move_base", true);
  ROS_INFO("Waiting for server");
  nav_client.waitForServer();
  
  move_base_msgs::MoveBaseGoal nav_goal;
  nav_goal.target_pose.pose.position.x = 8;
  nav_goal.target_pose.pose.position.y = 0;
  nav_goal.target_pose.pose.position.z = .16;
  nav_goal.target_pose.pose.orientation.x = 0;
  nav_goal.target_pose.pose.orientation.y = 0;
  nav_goal.target_pose.pose.orientation.z = 0;
  nav_goal.target_pose.pose.orientation.w = 1;
  nav_goal.target_pose.header.frame_id = "map"; //These next two probably aren't necessary.
  nav_goal.target_pose.header.stamp = ros::Time::now();
  nav_goal.target_pose.header.seq = 0; 
  
  ROS_INFO("Sending goal");
  nav_client.sendGoal(nav_goal);
  nav_client.waitForResult();
  ROS_INFO("Reached goal?");
  
  actionlib::SimpleClientGoalState state = nav_client.getState();
  ROS_INFO("A message from move_base: %s", state.getText().c_str());
  
  return 0;
}
