#include "DStarPlanner.h"

#include <memory>
#include <algorithm>
#include <mutex>
#include <limits.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <stdio.h>


//valgrind --track-origins=yes --log-file=/home/justin/temp/log.txt 
using namespace auvsl;


DStarPlanner::DStarPlanner(){
    curr_waypoint_ = 0;
    initialized_ = 0;
    
    private_nh_ = new ros::NodeHandle("~/local_planner");
    private_nh_->getParam("/DStarPlanner/map_res", map_res_); //map_res_ = .05;
    private_nh_->getParam("/DStarPlanner/grid_cols", cols_);
    private_nh_->getParam("/DStarPlanner/grid_rows", rows_);
    state_map_ = 0;
    
    control_system_ = new SimpleControlSystem(); //AnfisControlSystem();
    
    initWindow();    
}

DStarPlanner::~DStarPlanner(){
    if(initialized_){
        delete private_nh_;
        planner_thread_->join();
        delete planner_thread_;
    }

    delete control_system_;
}


void DStarPlanner::initialize(std::string name){
    planner_thread_ = new boost::thread(&DStarPlanner::runPlanner, this);
}

bool DStarPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel){       
    std::vector<Eigen::Vector2f> waypoints;
    {
        std::lock_guard<std::mutex> lock(wp_mu_);
        for(unsigned i = 0; i < local_waypoints_.size(); i++){
            waypoints.push_back(local_waypoints_[i]);
        }
    }
    
    
    cmd_vel.linear.y = 0; //?
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    
    if(waypoints.empty()){
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      return true;
    }
    
    float v_forward;
    float v_angular;
    
    nav_msgs::Odometry pose;

    if(!getROSPose(pose)){
      ROS_INFO("Failed to get robot pose");
    }
    
    //ROS_INFO("D* %f %f   %f %f   %f %f   %f %f", pose.pose.position.x, pose.pose.position.y,  waypoints[0][0], waypoints[0][1], waypoints[1][0], waypoints[1][1], waypoints[2][0], waypoints[2][1]);
    //ROS_INFO("D* computeVelocityCommands: computeVelocityCommand");
    control_system_->computeVelocityCommand(waypoints, pose.pose, v_forward, v_angular);
    
    cmd_vel.linear.x = v_forward;
    cmd_vel.angular.z = v_angular;
    
    ROS_INFO("D* Velocity commands computed: %f %f", v_forward, v_angular);
    return true;
}

//This actually queries the configure localization node and obtains a position estimate.
int getROSPose(nav_msgs::Odometry &odom){
  std::lock_guard<std::mutex> lock(odom_mu_);
  odom = latest_odom_;
  return 1;
}

bool DStarPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan){
    ROS_INFO("D* setPlan");
    Eigen::Vector2f prev_wp(plan[0].pose.position.x, plan[0].pose.position.y);
    const float dist = 1;
    
    global_waypoints_.push_back(prev_wp); 
    
    for(unsigned i = 1; i < plan.size(); i++){
        float dx = prev_wp[0] - plan[i].pose.position.x;
        float dy = prev_wp[1] - plan[i].pose.position.y;
    
        if(sqrtf((dx*dx) + (dy*dy)) > dist){
            global_waypoints_.push_back(Eigen::Vector2f(plan[i].pose.position.x, plan[i].pose.position.y));
            prev_wp[0] = plan[i].pose.position.x;
            prev_wp[1] = plan[i].pose.position.y;
        }
    }

    //make sure start and goal match
    global_waypoints_[global_waypoints_.size()-1] = Eigen::Vector2f(plan[plan.size()-1].pose.position.x, plan[plan.size()-1].pose.position.y);
    
    //testing. Remove following lines when you actually want to run it for real
    global_waypoints_.clear();
    global_waypoints_.push_back(Eigen::Vector2f(plan[0].pose.position.x, plan[0].pose.position.y));
    global_waypoints_.push_back(Eigen::Vector2f(plan[plan.size()-1].pose.position.x, plan[plan.size()-1].pose.position.y));
    
    ROS_INFO("D* Test LP Start point: %f %f", global_waypoints_[0][0], global_waypoints_[0][1]);
    ROS_INFO("D* Test LP Goal point: %f %f", global_waypoints_[1][0], global_waypoints_[1][1]);
    
    
    ROS_INFO("D* num waypoints %lu", global_waypoints_.size());
    return true;
}

bool DStarPlanner::isGoalReached(){
    if(global_waypoints_.empty()){
      return false;
    }
    
    Eigen::Vector2f goal = global_waypoints_[global_waypoints_.size()-1];
    Eigen::Vector2f current_pose = getCurrentPose();
    
    float dx = goal[0] - current_pose[0];
    float dy = goal[1] - current_pose[1];
    return sqrtf(dx*dx + dy*dy) < goal_tol_;
}

int DStarPlanner::isStateValid(float x, float y){
  //ROS_INFO("D* is state valid");
  StateData *temp = readStateMap(x, y);
  return temp->occupancy < occupancy_threshold_;
}

void DStarPlanner::odometryCallback(const nav_msgs::Odometry &odom){
  std::lock_guard<std::mutex> lock(odom_mu_);
  latests_odom_ = odom;
}

//This is going to be a real stupid inefficient function
//Just iterate through new_grid and look for differences
//Between new_grid and the current grid.
void DStarPlanner::updateEdgeCostsCallback(const std_msgs::Float32MultiArray &new_grid){
    ROS_INFO("D* update edge costs callback");
    ros::WallTime start_time = ros::WallTime::now();
    ros::WallDuration exe_time;

    if(!state_map_){
      ROS_INFO("D* Occ grid not yet initialized. Can't update edges in grid.");
      return;
    }
    
    StateData temp_state_data;
    for(int i = 0; i < rows_; i++){
      int offset = i*cols_;
      for(int j = 0; j < cols_; j++){
        if(fabs(new_grid[offset+j] - state_map[offset+j].occupancy) > EPSILON){
          temp_state_data.x = j;
          temp_state_data.y = i;
          temp_state_data.occupancy = new_grid[offset+j];
          
          update_nodes.push_back(temp_state_data);
          drawObstacle(&state_map_[offset+j], 0);
        }
      }
    }
    
    
    
    ROS_INFO("D* updateEdge: Done with detecting new states to update. Entering critical section now");
    
    //CRITICAL SECTION
    {
        std::lock_guard<std::mutex> lock(update_mu_);
        for(unsigned i = 0; i < update_nodes.size(); i++){
          update_nodes_.push_back(update_nodes[i]);
        }
    }
    
    has_init_map_ = 1;
    
    //exe_time = ros::WallTime::now() - start_time;
    //ROS_INFO("D* updateEdge: exiting critical section %u %u", exe_time.sec, exe_time.nsec);
}



//May need to check how getRobotPose works. It could block while it waits to hear from odom.
//Which would really slow things down.
Eigen::Vector2f DStarPlanner::getCurrentPose(){
    Eigen::Vector2f current_pose;
    geometry_msgs::PoseStamped pose;
    costmap_ros_->getRobotPose(pose);
    current_pose[0] = pose.pose.position.x;
    current_pose[1] = pose.pose.position.y;
    return current_pose;
}


/*
//Grid will just be initialized by calling updateEdgeCostsCallback the first time.
void DStarPlanner::initOccupancyGridCallback(const std_msgs::Float32MultiArray &initial_grid){
    ROS_INFO("D* init occupancy grid");

    unsigned cnt = 0;
    for(unsigned y = 0; y < rows_; y++){
        offset = y*cols_;
        for(unsigned x = 0; x < cols_; x++){
            
            state_map_[offset+x].occupancy = sum/(float)max_neighbors; //(float)std::min(sum, 4);
        }
    }
        
    ROS_INFO("D* Done initializing occupancy grid");
}
*/

int DStarPlanner::initPlanner(Eigen::Vector2f start, Eigen::Vector2f goal){
    ROS_INFO("D* init planner");
    open_list_.clear();

    init_curr_wp_ = 1; //this is required to set curr_wp_ to origin for the control system when the robot is at the very start of the path plan
    
    //TODO: Think about this a bit more and talk to Aniketh about it.
    x_range_ = (cols_*map_res_);
    y_range_ = (rows_*map_res_);
    
    x_offset_ = 0;
    y_offset_ = 0;
    
    if(state_map_){
        delete[] state_map_;
    }
    state_map_ = new StateData[cols_*rows_];
    
    unsigned offset;
    for(unsigned i = 0; i < rows_; i++){
        offset = cols_*i;
        for(unsigned j = 0; j < cols_; j++){
            state_map_[offset+j].tag = NEW;
            
            state_map_[offset+j].min_cost = -1;
            state_map_[offset+j].curr_cost = 1;
            state_map_[offset+j].b_ptr = 0;
            state_map_[offset+j].x = j;
            state_map_[offset+j].y = i;
            
            //drawStateTag(&state_map_[offset+j]);
        }
    }
    
    //This is going to get a local obstacle and terrain point clouds.
    //And then build a local occ grid
    //initOccupancyGrid(start, goal);
    
    if(!isStateValid(start[0], start[1])){
      ROS_INFO("D* Starting sTATE IS invalid %f %f", start[0], start[1]);
      //return 1;
    }


    ROS_INFO("Start %f %f", start[0], start[1]);
    ROS_INFO("Goal %f %f", goal[0], goal[1]);
    
    
    StateData *goal_state = readStateMap(goal[0], goal[1]);
    insertState(goal_state, 0);
    goal_state->b_ptr = 0;
    drawGoal(goal_state);
    
    float k_min;
    StateData* state_xc = readStateMap(start[0], start[1]);
    drawGoal(state_xc);

    //pressEnter();
    
    do{
        k_min = processState(0);
    } while((k_min != -1) && (state_xc->tag != CLOSED));

    return k_min;
}

int DStarPlanner::replan(StateData* robot_state){
    ROS_INFO("D* replan");
    float k_min;

    do{
        k_min = processState(1);
    } while(!(k_min >= robot_state->curr_cost) && (k_min != -1));
    
    return k_min;
}


//Main Loop thread
void DStarPlanner::runPlanner(){
    ROS_INFO("D* Initialize DStar");
    
    bool is_disabled = false;
    private_nh_->getParam("disable_lp", is_disabled);
    if(is_disabled) return;
    
    ros::Rate loop_rate(10);
    
    control_system_->initialize();
    
    //This is still useful just for the getRobotPose function
    private_nh_->getParam("/DStarPlanner/goal_tol", goal_tol_);
    //private_nh_->getParam("/LocalMap/occupancy_threshold", occupancy_threshold_);
    
    std::string odom_topic;
    private_nh_->getParam("/DStarPlanner/odometry_topic", odometry_topic);
    odom_callback_ = private_nh_->subscribe<nav_msgs::Odometry>(odometry_topic.c_str(),
                                                                100,
                                                                &DStarPlanner::getROSPose,
                                                                this);
    
    std::string update_topic;
    private_nh_->getParam("/DStarPlanner/grid_update_topic", update_topic);
    update_callback_ = private_nh_->subscribe<std_msgs::Float32MultiArray>(update_topic.c_str(),
                                                                           1,
                                                                           &DStarPlanner::updateEdgeCostsCallback,
                                                                           this);
    
    
    
    has_init_map_ = 0;
    do{
      ros::spinOnce();
      loop_rate.sleep();
    } while(!has_init_map_);
    init_sub.shutdown();
    ROS_INFO("D* Got initial map");
    
    
    planner_failed_ = 0;
    initialized_ = 1;
    ROS_INFO("init over");
    
    ROS_INFO("D* Entering DStar Main Thread");
    Eigen::Vector2f curr_pose;
    Eigen::Vector2f X_pos;
    StateData* X_state;
    StateData* temp_start;
    StateData* temp_goal;
    
    std::vector<StateData*> actual_path;
    
    ROS_INFO("D* Waiting for global plan");
    while(global_waypoints_.empty()){
      loop_rate.sleep();
    }
    ROS_INFO("D* Got global plan");
    
    
    ROS_INFO("D* num gobal waypoints %lu", global_waypoints_.size());
    X_pos = global_waypoints_[0];
    //idx is the current waypoint, idx+1 is the goal
    for(unsigned idx = 0; idx < (global_waypoints_.size()-1); idx++){
        ROS_INFO("D* Start State %f %f", X_pos[0], X_pos[1]);
        ROS_INFO("D* Goal State %f %f", global_waypoints_[idx+1][0], global_waypoints_[idx+1][1]);
                
        if(initPlanner(X_pos, global_waypoints_[idx+1]) == -1){   //This finds a solution. aka sets the backpointers from start to goal
            ROS_INFO("D* Couldn't find an initial path");
            while(1){
              loop_rate.sleep();
            }
            //pressEnter();
            planner_failed_ = 1;
            return;
        }
        
        temp_start = readStateMap(X_pos[0], X_pos[1]);
        temp_goal = readStateMap(global_waypoints_[idx+1][0], global_waypoints_[idx+1][1]);
        
        //drawGoal(temp_start);
        //drawGoal(temp_goal);
        
        X_state = temp_start;
        
        actual_path.clear();
        actual_path.push_back(X_state);

        ROS_INFO("D* starting online part");
        
        do{ //This scans for new obstacles, replans, and follows the backptr
          stepPlanner(X_state, X_pos);
          loop_rate.sleep();
          
          if(!X_state){ //could not find a path to goal.
              planner_failed_ = 1;
              return;
          }

          actual_path.push_back(X_state);
          
          
          
        } while(getDistance(global_waypoints_[idx+1], getCurrentPose()) > .5f);//readStateMap(global_waypoints_[idx+1][0], global_waypoints_[idx+1][1]) != X_state);
        
        ROS_INFO("D* Reached waypoint");
    }
    
    update_callback_.shutdown();
    
    return;
}

int DStarPlanner::stepPlanner(StateData*& robot_state, Eigen::Vector2f &X_robot){
    ROS_INFO("D* stepPlanner");
    //This is going to scan the environment and update the node occupancy/graph edge costs
    //The following is going to be slow as hell. But this is just a demo.
    //Just go over all the nodes in state_map_
    //Make sure all nodes that are inside of obstacles are marked obstacles
    //If the node wasn't marked obstacle and now is, and state was CLOSED
    //Then add node to the open_list_
    //This is like prepare_repair()
    
    //ROS_INFO("D* stepPlanner %f %f", X_robot[0], X_robot[1]);

    //ros::MultiThreadedSpinner spinner(1, );
    //spinner.spin();
    
    
    ROS_INFO("D* stepPlanner: Entering critical section to update edge costs");
    std::vector<StateData> update_nodes; //this threads local copy
    {
        //Critical section for recieving which nodes in the graph are updated
        std::lock_guard<std::mutex> lock(update_mu_);
        for(unsigned i = 0; i < update_nodes_.size(); i++){
            update_nodes.push_back(update_nodes_[i]);
        }
        update_nodes_.clear();
    }
    ROS_INFO("D* stepPlanner: exiting critical section");
    
    ROS_INFO("D* Num Cells To be Processed: %lu", update_nodes.size());
    std::vector<StateData*> neighbors;
    
    for(unsigned i = 0; i < update_nodes.size(); i++){
        unsigned mx = update_nodes[i].x;
        unsigned my = update_nodes[i].y;
        unsigned m_idx = (my*cols_) + mx;
        
        //This condition is given and tested for in the updateEdgeCallback
        //if(update_nodes[i].occupancy != state_map_[m_idx].occupancy){
        getNeighbors(neighbors, &state_map_[m_idx], 1);
        
        state_map_[m_idx].occupancy = update_nodes[i].occupancy;
                
        if(state_map_[m_idx].tag == CLOSED){
          insertState(&state_map_[m_idx], state_map_[m_idx].curr_cost);
        }
        
        for(unsigned k = 0; k < neighbors.size(); k++){
          if(neighbors[k]->tag == CLOSED){
            insertState(neighbors[k], neighbors[k]->curr_cost);
          } 
        }
    }
    
    int k_min = replan(robot_state); //pass current robot position.

    drawPath(robot_state);
    followBackpointer(robot_state);
    X_robot = getRealPosition(robot_state->x, robot_state->y);
    
    return k_min;
}

void DStarPlanner::followBackpointer(StateData*& robot_state){
    static Eigen::Vector2f curr_wp;
    
    ROS_INFO("D* follow back pointer");
    unsigned x_actual; //map physical location to local map index
    unsigned y_actual;
    Eigen::Vector2f current_pose = getCurrentPose();
    getMapIdx(current_pose, x_actual, y_actual);
    drawRobotPos(x_actual, y_actual);
    
    ROS_INFO("D* followBackPointer: setting waypoints for control system");
    //Critical Section. Sets local waypoints for control system to use as lookahead.
    //Need to guard local_waypoints which is being read in the computeVelocityCommands function/move_base thread.
    {
        std::lock_guard<std::mutex> lock(wp_mu_);
        StateData* temp_state = robot_state;
        
        local_waypoints_.clear();
        for(unsigned i = 0; i < lookahead_len_; i++){
          Eigen::Vector2f temp_vec = getRealPosition(temp_state->x, temp_state->y);
          temp_state = temp_state->b_ptr; 
          if(temp_state == NULL){
            break;
          }
          local_waypoints_.push_back(temp_vec); //don't include the current point. Set curr point to prev target point
        }
        
        if(init_curr_wp_){
          init_curr_wp_ = 0;
          curr_wp = current_pose;
        }
        
        local_waypoints_.insert(local_waypoints_.begin(), curr_wp);
    }
    
    
    float dx, dy, dist, best_dist;
    unsigned best_idx;
    
    dx = local_waypoints_[1][0] - current_pose[0];
    dy = local_waypoints_[1][1] - current_pose[1];
    best_dist = sqrtf(dx*dx + dy*dy);
    best_idx = 1;
    
    for(unsigned i = 2; i < local_waypoints_.size(); i++){
      dx = local_waypoints_[i][0] - current_pose[0];
      dy = local_waypoints_[i][1] - current_pose[1];
      dist = sqrtf(dx*dx + dy*dy);
      
      if(dist < best_dist){
        best_dist = dist;
        best_idx = i;
      }
    }

    //Hopefully my logic is correct here.
    if(best_idx != 1){ //this means a closer target is available. set curr_wp[k+1] = target_wp[k], target_wp[k+1] = future_wp[k], future_wp[k] = next waypoint. 
      curr_wp = local_waypoints_[1];
      
      ROS_INFO("D* followBackPointer: Back Pointer = Advanced down the path");
      for(unsigned i = 0; i < best_idx-1; i++){
        robot_state = robot_state->b_ptr;
      }
    }    
    
}


void DStarPlanner::getNeighbors(std::vector<StateData*> &neighbors, StateData* X, int replan){
  //ROS_INFO("D* get neighbors");
  neighbors.clear();
  // each state has 8 Neighbors, top left, top, top right, left, right, etc...
  const int dxdy[8][2] = {
                           {-1,-1},
                           {-1, 0},
                           {-1, 1},
                           {0, -1},
                           {0,  1},
                           {1, -1},
                           {1,  0},
                           {1,  1}
  };
  
  
  
  Eigen::Vector2f pos;
  for(int i = 0; i < 8; i++){
    unsigned nx = X->x + dxdy[i][0]; //It won't go negative, but it will overflow and then be outside the range of the grid
    unsigned ny = X->y + dxdy[i][1];
    //Make sure state is within the grid. Or else a segfault will occur
    if((nx >= 0) && (nx < cols_) && (ny >= 0) && (ny < rows_)){
      pos = getRealPosition(nx, ny);
      //ROS_INFO("D* idx %u %u   pos %f %f", nx, ny, pos[0], pos[1]);
      neighbors.push_back(&state_map_[(ny*cols_)+nx]);
    }
  }
}

float DStarPlanner::processState(int replan){
    if(open_list_.empty()){
        return -1;
    }
    
    StateData *X = open_list_[0];
    float k_old = X->min_cost;
    
    deleteState(X);
    
    //ROS_INFO("D* Current x y   %u %u", X->x, X->y);
    std::vector<StateData*> neighbors;
    getNeighbors(neighbors, X, replan);

    //ROS_INFO("D* Num neighbors %lu", neighbors.size());
    StateData *Y;
    
    //ROS_INFO("D* State curr cost %f,   kmin %f", X->curr_cost, k_old);
    
    //Raise
    // k_old < X->curr_cost
    if(k_old < X->curr_cost){
      //drawStateType(X, RAISE);
        for(unsigned i = 0; i < neighbors.size(); i++){
            Y = neighbors[i];
            if(Y->tag != NEW &&
               Y->curr_cost <= k_old &&
               X->curr_cost > (Y->curr_cost + getEdgeCost(Y, X))){
                
                X->b_ptr = Y;
                X->curr_cost = Y->curr_cost + getEdgeCost(Y, X);
            }
        }
    }

    //Lower bluh
    // k_old == X->curr_cost but for floats.
    if(fabs(k_old - X->curr_cost) < EPSILON){
      //drawStateType(X, LOWER);        
        for(unsigned i = 0; i < neighbors.size(); i++){
            Y = neighbors[i];
            //ROS_INFO("D* Neighbor %u %u", Y->x, Y->y);
            if(Y->tag == NEW ||
              (Y->b_ptr == X && Y->curr_cost != (X->curr_cost + getEdgeCost(X, Y))) ||
              (Y->b_ptr != X && Y->curr_cost > (X->curr_cost + getEdgeCost(X, Y)))){
                Y->b_ptr = X;
                insertState(Y, X->curr_cost + getEdgeCost(X,Y));
            }
        }

    }
    else{ //Nothing?
      //drawStateType(X, NORMAL);
        for(unsigned i = 0; i < neighbors.size(); i++){
            Y = neighbors[i];
            if(Y->tag == NEW || (Y->b_ptr == X && Y->curr_cost != (X->curr_cost + getEdgeCost(X, Y)))){
                Y->b_ptr = X;
                insertState(Y, X->curr_cost + getEdgeCost(X, Y));
            }
            else{
                if(Y->b_ptr != X && Y->curr_cost > (X->curr_cost + getEdgeCost(X, Y))){
                    insertState(X, X->curr_cost);
                }
                else if(Y->b_ptr != X &&
                        X->curr_cost > (Y->curr_cost + getEdgeCost(Y, X)) &&
                        Y->tag == CLOSED &&
                        Y->curr_cost > k_old){
                    insertState(Y, Y->curr_cost);
                }
            }
        }
    }
    
    if(open_list_.empty())
        return -1;
    else
        return open_list_[0]->min_cost;
}

void DStarPlanner::insertState(StateData* state, float path_cost){
  //ROS_INFO("D* insertState   open_lst.size()  %lu", open_list_.size());
    switch(state->tag){
    case NEW:
        state->min_cost = path_cost;
        break;
    case OPEN:
        //ensure no repeats in the open list.
        for(unsigned i = 0; i < open_list_.size(); i++){
            if(open_list_[i] == state){
                open_list_.erase(open_list_.begin() + i);
                break;
            }
        }
        state->min_cost = std::min(state->min_cost, path_cost);
        break;
    case CLOSED:
        state->min_cost = std::min(state->curr_cost, path_cost);
        break;        
    }

    state->curr_cost = path_cost;
    state->tag = OPEN;

    //if(open_list_.empty()){
    //  open_list_.push_back(state);
    //}
    
    //ROS_INFO("D* current state min cost %f", state->min_cost);
    
    for(unsigned i = 0; i < open_list_.size(); i++){
        if(open_list_[i]->min_cost > state->min_cost){
            open_list_.insert(open_list_.begin() + i, state);
            return;
        }
    }
    
    //drawStateTag(state);
    open_list_.push_back(state);
}

void DStarPlanner::deleteState(StateData *state){
    //ROS_INFO("D* deleteState");
    for(unsigned i = 0; i < open_list_.size(); i++){
        if(open_list_[i] == state){
            open_list_.erase(open_list_.begin() + i);
            state->tag = CLOSED;
            return;
        }
    }
}


float DStarPlanner::getEdgeCost(StateData* X, StateData* Y){
  //ROS_INFO("D* getEdgeCost");
  int dx = X->x - Y->x;
  int dy = X->y - Y->y;
  return sqrtf(dx*dx + dy*dy) + Y->occupancy*1000;
}

float DStarPlanner::getPathCost(Eigen::Vector2f X, Eigen::Vector2f G){
  //ROS_INFO("D* getPathCost");
  StateData* state = readStateMap(X[0], X[1]);
  return state->curr_cost;
}

float DStarPlanner::getMinPathCost(Eigen::Vector2f X, Eigen::Vector2f G){
  //ROS_INFO("D* getMinPathCost");
  StateData* state = readStateMap(X[0], X[1]);
  return state->min_cost;
}

void DStarPlanner::getMapIdx(Eigen::Vector2f X, unsigned &x, unsigned &y){
    float x_scale = cols_ / x_range_;
    float y_scale = rows_ / y_range_;
    
    int x_int = floorf((X[0] - x_offset_)*x_scale);
    int y_int = floorf((X[1] - y_offset_)*y_scale);
    
    x = std::min(std::max(x_int, int(0)), int(cols_));
    y = std::min(std::max(y_int, int(0)), int(rows_));
    
    //ROS_INFO("D* getMapIdx    %f  %f      %d  %d   %u %u",   X[0], X[1],   x_int, y_int,  x, y);
}

StateData* DStarPlanner::readStateMap(float rx, float ry){
    unsigned x;
    unsigned y;
    
    getMapIdx(Eigen::Vector2f(rx, ry), x, y);
    return &(state_map_[(y*cols_)+x]);
}

Eigen::Vector2f DStarPlanner::getRealPosition(unsigned x, unsigned y){
    float x_scale = x_range_ / cols_;
    float y_scale = y_range_ / rows_;
    Eigen::Vector2f X;
    
    
    X[0] = ((float)x*x_scale) + x_offset_;// + (x_scale*.5);
    X[1] = ((float)y*y_scale) + y_offset_;// + (y_scale*.5);
    return X;
}






/*
 * Following functions are debug related. Creates a window and shows the search graph
 * as well as the state tags so the waves of RAISE/LOWER should be visible
 */


void DStarPlanner::initWindow(){
  dstar_visual_pub_ = private_nh_->advertise<visualization_msgs::Marker>("dstar_visual", 1000000);
}

void DStarPlanner::drawStateType(StateData *state, STATE_TYPE s_type){
    unsigned color;
    float scalar; //from 1 to zero

    visualization_msgs::Marker rect;
    rect.header.frame_id = "map";
    rect.header.stamp = ros::Time::now();
    rect.ns = "dstar_state";
    rect.action = visualization_msgs::Marker::ADD;
    rect.pose.orientation.w = 1.0;
    rect.pose.orientation.x = 0.0;
    rect.pose.orientation.y = 0.0;
    rect.pose.orientation.z = 0.0;
    rect.id = (state->y*cols_) + state->x;
    rect.type = visualization_msgs::Marker::CUBE;
    rect.scale.x = map_res_;
    rect.scale.y = map_res_;
    rect.scale.z = map_res_;//(map_res_ * state->curr_cost) + .01;
    
    rect.color.r = 0.0;
    rect.color.g = 0.0;
    rect.color.b = 0.0;
    
    scalar = std::min(1.0f, std::max(0.0f, (float)(.1+ .9*state->occupancy)));
    
    switch(s_type){
    case RAISE:
      rect.color.r = scalar;
      break;
    case LOWER:
      rect.color.b = scalar;
      break;
    case NORMAL:
      rect.color.g = scalar;
      break;
    }
    
    Eigen::Vector2f goal = getRealPosition(state->x, state->y);
    
    rect.color.a = .5;
    rect.pose.position.x = goal[0];
    rect.pose.position.y = goal[1];
    rect.pose.position.z = 0;
    
    dstar_visual_pub_.publish(rect);
    
    //ROS_INFO("D* drawStateType %u   %u %u", rect.id, state->x, state->y);
    
    //drawStateTag(state);
    //drawStateBPtr(state);
    
}

void DStarPlanner::drawStateTag(StateData* state){
    float scalar = std::min(1.0f, std::max(0.0f, (float)(.1+ .01*state->curr_cost)));
    unsigned color;
    
    unsigned x = state->x * 2;
    unsigned y = state->y * 2;
    
    switch(state->tag){
    case NEW:
        color = 0x0000FF;//BLUE
        break;
    case CLOSED:
        color = 0x00FF00; //GREEN
        break;
    case OPEN:
        color = 0xFF0000; //RED
        break;
    }
    
    //draw something if you feel like it.
}

void DStarPlanner::drawObstacle(StateData *state, int clear){
  Eigen::Vector2f goal = getRealPosition(state->x, state->y);
  
  visualization_msgs::Marker rect;
  rect.header.frame_id = "map";
  rect.header.stamp = ros::Time::now();
  rect.ns = "dstar_obs";
  rect.action = visualization_msgs::Marker::ADD;
  rect.pose.orientation.w = 1.0;
  rect.pose.orientation.x = 0.0;
  rect.pose.orientation.y = 0.0;
  rect.pose.orientation.z = 0.0;
  rect.id = (state->y*cols_) + state->x;
  rect.type = visualization_msgs::Marker::CUBE;
  rect.scale.x = map_res_*.5;
  rect.scale.y = map_res_*.5;
  rect.scale.z = map_res_;

  if(clear){
    rect.color.r = 0.0;
  }
  else{
    rect.color.r = 1.0;
  }
  rect.color.g = 0.0;
  rect.color.b = 0.0;
  rect.color.a = 0.5;
  rect.pose.position.x = goal[0];
  rect.pose.position.y = goal[1];
  rect.pose.position.z = 0;
  
  dstar_visual_pub_.publish(rect);
}

void DStarPlanner::drawStateBPtr(StateData *state){
  if(!state->b_ptr){
    return;
  }

  std::vector<geometry_msgs::Point> pts;
  
  Eigen::Vector2f start = getRealPosition(state->x, state->y);
  Eigen::Vector2f end = getRealPosition(state->b_ptr->x, state->b_ptr->y);

  geometry_msgs::Point start_pt;
  start_pt.x = start[0] + map_res_*.5f;
  start_pt.y = start[1] + map_res_*.5f;
  start_pt.z = .1f;

  geometry_msgs::Point end_pt;
  end_pt.x = end[0] + map_res_*.5f;
  end_pt.y = end[1] + map_res_*.5f;
  end_pt.z = .1f;
  
  pts.push_back(start_pt);
  pts.push_back(end_pt);
  
  visualization_msgs::Marker point_list;
  point_list.header.frame_id = "map";
  point_list.header.stamp = ros::Time::now();
  point_list.ns = "dstar_bptr";
  point_list.action = visualization_msgs::Marker::ADD;
  point_list.pose.orientation.w = 1.0;
  point_list.pose.orientation.x = 0.0;
  point_list.pose.orientation.y = 0.0;
  point_list.pose.orientation.z = 0.0;
  point_list.id = (state->y*cols_) + state->x;
  point_list.type = visualization_msgs::Marker::POINTS;
  point_list.scale.x = 0.1; //line width
  point_list.scale.y = 0.1; //line width
  point_list.color.r = 1.0;
  point_list.color.g = 1.0;
  point_list.color.b = 1.0;
  point_list.color.a = 1.0;
  point_list.points = pts;
  
  dstar_visual_pub_.publish(point_list);
}

void DStarPlanner::drawRobotPos(unsigned x, unsigned y){
  Eigen::Vector2f goal = getRealPosition(x, y);
  
  visualization_msgs::Marker rect;
  rect.header.frame_id = "map";
  rect.header.stamp = ros::Time::now();
  rect.ns = "dstar_robot_pos";
  rect.action = visualization_msgs::Marker::ADD;
  rect.pose.orientation.w = 1.0;
  rect.pose.orientation.x = 0.0;
  rect.pose.orientation.y = 0.0;
  rect.pose.orientation.z = 0.0;
  rect.id = (y*cols_) + x;
  rect.type = visualization_msgs::Marker::CUBE;
  rect.scale.x = map_res_;
  rect.scale.y = map_res_;
  rect.scale.z = .1;
  
  rect.color.r = 1.0;
  rect.color.g = 0.0;
  rect.color.b = 1.0;
  rect.color.a = 0.5;
  rect.pose.position.x = goal[0];
  rect.pose.position.y = goal[1];
  rect.pose.position.z = 0;
  
  dstar_visual_pub_.publish(rect);
}


void DStarPlanner::drawRobotPos(StateData* state){
  drawRobotPos(state->x, state->y);
}

void DStarPlanner::drawGoal(StateData *state){
  //ROS_INFO("State   %u %u", state->x, state->y);
  //XSetForeground(dpy, gc, 0x00FF00);
  
  Eigen::Vector2f goal = getRealPosition(state->x, state->y);
  
  visualization_msgs::Marker rect;
  rect.header.frame_id = "map";
  rect.header.stamp = ros::Time::now();
  rect.ns = "dstar_goal";
  rect.action = visualization_msgs::Marker::ADD;
  rect.pose.orientation.w = 1.0;
  rect.pose.orientation.x = 0.0;
  rect.pose.orientation.y = 0.0;
  rect.pose.orientation.z = 0.0;
  rect.id = (state->y*cols_) + state->x;
  rect.type = visualization_msgs::Marker::CUBE;
  rect.scale.x = .5;
  rect.scale.y = .5;
  rect.scale.z = .5;
  
  rect.color.r = 1.0;
  rect.color.g = 0.0;
  rect.color.b = 0.0;
  rect.color.a = 1.0;
  rect.pose.position.x = goal[0];
  rect.pose.position.y = goal[1];
  rect.pose.position.z = 0;
  
  dstar_visual_pub_.publish(rect);
}

void DStarPlanner::drawPath(StateData *state){
    std::vector<geometry_msgs::Point> path_pts;
    geometry_msgs::Point pt;
    Eigen::Vector2f vec;
    while(state->b_ptr){
        vec = getRealPosition(state->x, state->y);
        pt.x = vec[0];
        pt.y = vec[1];
        pt.z = 0;
        path_pts.push_back(pt);
        state = state->b_ptr;
    }

    vec = getRealPosition(state->x, state->y);
    pt.x = vec[0];
    pt.y = vec[1];
    pt.z = 0;
    path_pts.push_back(pt);
    
    if(path_pts.empty()){
      return;
    }
    
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "dstar_line";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.pose.orientation.x = 0.0;
    line_list.pose.orientation.y = 0.0;
    line_list.pose.orientation.z = 0.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.scale.x = 0.06; //line width
    line_list.color.r = 1.0;
    line_list.color.g = 1.0;
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;
    line_list.points = path_pts;
    
    dstar_visual_pub_.publish(line_list);
}


void DStarPlanner::drawFinishedGraph(StateData *start, std::vector<StateData*> &actual_path){
  
}


