#include "GlobalParams.h"



float GlobalParams::fuzzy_constant_speed;
float GlobalParams::max_angular_vel;
float GlobalParams::timestep;
int GlobalParams::debug_level;
float GlobalParams::planner_resolution;
float GlobalParams::state_checker_resolution;
float GlobalParams::propagation_step_size;
float GlobalParams::visualize_planner;

float GlobalParams::goal_bias;
float GlobalParams::goal_bias2;

int GlobalParams::seed;
std::vector<float> GlobalParams::distance_weights;
int GlobalParams::num_control_samples;
bool GlobalParams::add_intermediate_states;

float GlobalParams::heading_heuristic;
float GlobalParams::angular_variance;
float GlobalParams::forward_variance;

float GlobalParams::p_gain;
float GlobalParams::goal_tolerance;
float GlobalParams::max_gp_runtime;


void GlobalParams::load_params(ros::NodeHandle *nh){
  nh->getParam("/fuzzy_constant_speed", fuzzy_constant_speed);
  nh->getParam("/max_angular_vel", max_angular_vel);
  nh->getParam("/timestep", timestep);
  nh->getParam("/debug_level", debug_level);
  nh->getParam("/planner_resolution", planner_resolution);
  nh->getParam("/state_checker_resolution", state_checker_resolution);

  nh->getParam("/propagation_step_size", propagation_step_size);
  nh->getParam("/visualize_planner", visualize_planner);

  nh->getParam("/goal_bias", goal_bias);
  nh->getParam("/goal_bias2", goal_bias2);

  nh->getParam("/distance_weights", distance_weights);
  nh->getParam("/add_intermediate_states", add_intermediate_states);
  nh->getParam("/num_control_samples", num_control_samples);

  nh->getParam("/seed", seed);

  nh->getParam("/heading_heuristic", heading_heuristic);
  nh->getParam("/forward_variance", forward_variance);
  nh->getParam("/angular_variance", angular_variance);

  nh->getParam("/p_gain", p_gain);

  nh->getParam("/goal_tolerance", goal_tolerance);
  nh->getParam("/max_gp_runtime", max_gp_runtime);

}




float GlobalParams::get_fuzzy_constant_speed(){return GlobalParams::fuzzy_constant_speed;}
float GlobalParams::get_max_angular_vel(){return GlobalParams::max_angular_vel;}
float GlobalParams::get_timestep(){return GlobalParams::timestep;}
int   GlobalParams::get_debug_level(){return GlobalParams::debug_level;}
float GlobalParams::get_planner_resolution(){return GlobalParams::planner_resolution;}
float GlobalParams::get_state_checker_resolution(){return GlobalParams::state_checker_resolution;}
float GlobalParams::get_propagation_step_size(){return GlobalParams::propagation_step_size;}
float GlobalParams::get_visualize_planner(){return GlobalParams::visualize_planner;}

float GlobalParams::get_goal_bias(){return GlobalParams::goal_bias;}
float GlobalParams::get_goal_bias2(){return GlobalParams::goal_bias2;}


std::vector<float> GlobalParams::get_distance_weights(){return GlobalParams::distance_weights;}

bool GlobalParams::get_add_intermediate_states(){return GlobalParams::add_intermediate_states;}
int  GlobalParams::get_num_control_samples(){return GlobalParams::num_control_samples;}
int  GlobalParams::get_seed(){return GlobalParams::seed;}

float GlobalParams::get_heading_heuristic(){return GlobalParams::heading_heuristic;}
float GlobalParams::get_angular_variance(){return GlobalParams::angular_variance;}
float GlobalParams::get_forward_variance(){return GlobalParams::forward_variance;}

float GlobalParams::get_p_gain(){return GlobalParams::p_gain;}
float GlobalParams::get_goal_tolerance(){return GlobalParams::goal_tolerance;}
float GlobalParams::get_max_gp_runtime(){return GlobalParams::max_gp_runtime;}
