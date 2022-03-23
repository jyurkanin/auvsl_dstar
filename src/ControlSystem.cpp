#include "ControlSystem.h"
#include "utils.h"
#include <algorithm>

#include <tf/tf.h>

using namespace auvsl;
using namespace Eigen;

ControlSystem::ControlSystem(){

}

ControlSystem::~ControlSystem(){

}



SimpleControlSystem::SimpleControlSystem(){

}

SimpleControlSystem::~SimpleControlSystem(){

}



int SimpleControlSystem::initialize(){
  return 1;
}

int SimpleControlSystem::computeVelocityCommand(std::vector<Vector2f> waypoints, geometry_msgs::Pose pose, float &v_forward, float &v_angular){
  tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 mat(quat);
  double roll;
  double pitch;
  double yaw;
  mat.getRPY(roll, pitch, yaw);
  float dx = waypoints[1][0] - pose.position.x;
  float dy = waypoints[1][1] - pose.position.y;
  
  float dist = sqrtf(dx*dx + dy*dy);
  
  float target_angle = atan2f(dy, dx);
  
  float ang_disp = target_angle - yaw;
  if(ang_disp < -M_PI){
    ang_disp += (2*M_PI);
  }
  else if(ang_disp > M_PI){
    ang_disp -= (2*M_PI);
  }

  v_angular = std::max(std::min(ang_disp*3.0f, .5f), -.5f);
  v_forward = std::max(std::min((dist*8.0f)/(1+(ang_disp*ang_disp)), 4.0f), 0.0f);
  if(fabs(ang_disp) > .5){
    v_forward *= .01;
  }
  
  
  return 1;
}


AnfisControlSystem::AnfisControlSystem(){
  
}

AnfisControlSystem::~AnfisControlSystem(){
  
}

int AnfisControlSystem::initialize(){
  ros::NodeHandle n;
  client_ = n.serviceClient<auvsl_control::AnfisControl>("anfis_control");
  ROS_INFO("Waiting for anfis service to exist");
  client_.waitForExistence();
  return 1;
}

int AnfisControlSystem::computeVelocityCommand(std::vector<Vector2f> waypoints, geometry_msgs::Pose pose, float &v_forward, float &v_angular){
  auvsl_control::AnfisControl srv;
  srv.request.pose = pose;
  
  srv.request.current_x = waypoints[0][0];
  srv.request.current_y = waypoints[0][1];
  
  srv.request.target_x = waypoints[1][0];
  srv.request.target_y = waypoints[1][1];

  if(waypoints.size() > 2){
    srv.request.future_x = waypoints[2][0];
    srv.request.future_y = waypoints[2][1];
  }
  else{
    srv.request.future_x = waypoints[1][0];
    srv.request.future_y = waypoints[1][1];
  }
  
  
  if(!client_.call(srv)){
    v_forward = 0;
    v_angular = 0;
    return 0;
  }
  
  v_forward = srv.response.cmd_vel.linear.x;
  v_angular = srv.response.cmd_vel.angular.z;
  
  return 1;
}



