#include <Eigen/Dense>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <auvsl_control/AnfisControl.h>


/*
 * Abstract base class for control systems + simple waypoint follower
 * For use by Woojin, Justin, and Marius in implementing control systems
 * Used by local planner function computeVelocityCommand
 */


//using namespace Eigen; //For the love of god, dont use using in a header file. Trust me.
namespace auvsl{

class ControlSystem{
public:
    ControlSystem();
    ~ControlSystem();

 
  virtual int initialize() = 0;
  virtual int computeVelocityCommand(std::vector<Eigen::Vector2f> waypoints, geometry_msgs::Pose pose, float &v_forward, float &v_angular) = 0;
};



class SimpleControlSystem : public ControlSystem{
public:
  SimpleControlSystem();
  ~SimpleControlSystem();
  int initialize() override;
  int computeVelocityCommand(std::vector<Eigen::Vector2f> waypoints, geometry_msgs::Pose pose, float &v_forward, float &v_angular) override;
};
  


class AnfisControlSystem : public ControlSystem{
public:
  AnfisControlSystem();
  ~AnfisControlSystem();
  int initialize() override;
  int computeVelocityCommand(std::vector<Eigen::Vector2f> waypoints, geometry_msgs::Pose pose, float &v_forward, float &v_angular) override;
  ros::ServiceClient client_;
};
  
}
