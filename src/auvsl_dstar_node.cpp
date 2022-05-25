#include <iostream>
#include <thread>
#include <vector>
#include <string>

#include "DStarPlanner.h"

#include <Eigen/Dense>

//launch-prefix="gdb -ex run --args

using namespace auvsl;

DStarPlanner *l_planner;


//This function is for later on when other nodes may want to request a global path plan
/*
bool globalPlannerCallback(GlobalPathPlan::Request &req, GlobalPathPlan::Response &resp){
  std::vector<Eigen::Vector2f> waypoints;
  float start_state[17];
  for(int i = 0; i < 17; i++){
    start_state[i] = req.start_state[i];
  }

  Eigen::Vector2f goal_pos(req.goal_pos[0], req.goal_pos[1]);

  g_planner->plan(waypoints, start_state, goal_pos, req.goal_tol);

  for(unsigned i = 0; i < waypoints.size(); i++){
    resp.waypoint_x[i] = waypoints[i][0];
    resp.waypoint_y[i] = waypoints[i][1];
  }

  return true;
}
//ros::ServiceServer g_planner_client = nh.advertiseService<GlobalPathPlan::Request, GlobalPathPlan::Response>("global_planner", globalPlannerCallback);
*/



//1. Solve Global Path Plan
//2. Run Local Path Planner and try to follow global path plan
int main(int argc, char **argv){
  ROS_INFO("Starting up auvsl_planner_node\n");
  
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  ros::Rate loop_rate(10);
  
  l_planner = new DStarPlanner();

  std::vector<geometry_msgs::PoseStamped> plan;

  ROS_INFO("Starting Local Planner\n");
  //start point is 0,0; goal is 10,0
  for(int i = 0; i < 2; i++){
    //ignore the header.
    geometry_msgs::PoseStamped wp;
    wp.pose.position.x = 0;
    wp.pose.position.y = (i*10);
    wp.pose.position.z = 0;
    
    wp.pose.orientation.x = 0;
    wp.pose.orientation.y = 0;
    wp.pose.orientation.z = 0;
    wp.pose.orientation.w = 1;
    
    plan.push_back(wp);
  }
    
  l_planner->setPlan(plan);
  
  std::cout << "Press enter to start (open rviz first)\n";
  char ignore;
  std::cin >> ignore;
  
  l_planner->initialize();
  
  std::string cmd_vel_topic;
  nh.getParam("/DStarPlanner/cmd_vel_topic", cmd_vel_topic);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/rexrov/cmd_vel", 100);  
  geometry_msgs::Twist cmd_vel;
  
  ROS_INFO("d*_node entering controller loop");
  while(!l_planner->isGoalReached()){
    l_planner->computeVelocityCommands(cmd_vel);
    vel_pub.publish(cmd_vel);
    
    ros::spinOnce(); //This is actually mega important. Without it, the updateEdgeCostsCallback wont even be called.
    loop_rate.sleep();
  }
  
  ROS_INFO("Local Planner Done");
  
  delete l_planner;
  
  return 0;
}
