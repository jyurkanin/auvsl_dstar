#include "DStarPlanner.h"

#include <memory>
#include <algorithm>
#include <mutex>
#include <limits.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <stdio.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

#include <pcl_ros/impl/transforms.hpp> //I do as stack overflow commands.

#define INIT_VALUE 0xDEADBEEF //A random value. Will be useful for debugging and detecting unitialized values
#define DEBUG_WINDOW 1


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(auvsl::DStarPlanner, nav_core::BaseLocalPlanner)


//valgrind --track-origins=yes --log-file=/home/justin/temp/log.txt 
using namespace auvsl;


DStarPlanner::DStarPlanner(){
    curr_waypoint_ = 0;
    initialized_ = 0;

    private_nh_ = new ros::NodeHandle("~/local_planner");
    private_nh_->getParam("/move_base/local_costmap/resolution", map_res_); //map_res_ = .05;
    state_map_ = 0;

    control_system_ = new AnfisControlSystem();
    
    //log_file.open("/home/justin/code/AUVSL_ROS/pose.csv", std::ofstream::out);
    //log_file << "x,y\n";
    /*
    x_range_ = 0;
    x_offset_ = 0;
    y_range_ = 0;
    y_offset_ = 0;
    */
    
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


void DStarPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros){
    costmap_ros_ = costmap_ros; //Ignore this param.
    planner_thread_ = new boost::thread(&DStarPlanner::runPlanner, this);
    
}

bool DStarPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel){       
    std::vector<Vector2f> waypoints;
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
    
    geometry_msgs::PoseStamped pose;

    if(!costmap_ros_->getRobotPose(pose)){
      ROS_INFO("Failed to get robot pose");
    }

    ROS_INFO("D* %f %f   %f %f   %f %f   %f %f", pose.pose.position.x, pose.pose.position.y,  waypoints[0][0], waypoints[0][1], waypoints[1][0], waypoints[1][1], waypoints[2][0], waypoints[2][1]);
    //    ROS_INFO("D* computeVelocityCommands: computeVelocityCommand");
    control_system_->computeVelocityCommand(waypoints, pose.pose, v_forward, v_angular);
    
    cmd_vel.linear.x = v_forward;
    cmd_vel.angular.z = v_angular;

    ROS_INFO("D* Velocity commands computed: %f %f", v_forward, v_angular);
    return true;
}

bool DStarPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan){
  ROS_INFO("D* setPlan");
    Vector2f prev_wp(plan[0].pose.position.x, plan[0].pose.position.y);
    const float dist = 1;
    
    global_waypoints_.push_back(prev_wp); 
    
    for(unsigned i = 1; i < plan.size(); i++){
        float dx = prev_wp[0] - plan[i].pose.position.x;
        float dy = prev_wp[1] - plan[i].pose.position.y;
    
        if(sqrtf((dx*dx) + (dy*dy)) > dist){
            global_waypoints_.push_back(Vector2f(plan[i].pose.position.x, plan[i].pose.position.y));
            prev_wp[0] = plan[i].pose.position.x;
            prev_wp[1] = plan[i].pose.position.y;
        }
    }

    //make sure start and goal match
    global_waypoints_[global_waypoints_.size()-1] = Vector2f(plan[plan.size()-1].pose.position.x, plan[plan.size()-1].pose.position.y);
    
    //testing. Remove following lines when you actually want to run it for real
    global_waypoints_.clear();
    global_waypoints_.push_back(Vector2f(plan[0].pose.position.x, plan[0].pose.position.y));
    global_waypoints_.push_back(Vector2f(plan[plan.size()-1].pose.position.x, plan[plan.size()-1].pose.position.y));
    
    ROS_INFO("D* Test LP Start point: %f %f", global_waypoints_[0][0], global_waypoints_[0][1]);
    ROS_INFO("D* Test LP Goal point: %f %f", global_waypoints_[1][0], global_waypoints_[1][1]);
    
    
    ROS_INFO("D* num waypoints %lu", global_waypoints_.size());
    return true;
}

bool DStarPlanner::isGoalReached(){
    if(global_waypoints_.empty()){
      return false;
    }
    
    Vector2f goal = global_waypoints_[global_waypoints_.size()-1];
    Vector2f current_pose = getCurrentPose();
    
    float dx = goal[0] - current_pose[0];
    float dy = goal[1] - current_pose[1];
    return sqrtf(dx*dx + dy*dy) < goal_tol_;
}

int DStarPlanner::isStateValid(float x, float y){
  //ROS_INFO("D* is state valid");
  StateData *temp = readStateMap(x, y);
  return temp->occupancy < occupancy_threshold_;
}


//TODO: process sampled point cloud
//1. temp_cloud = sample_cloud + local_terrain_cloud_
//2. Run region growing segmentation on temp_cloud
//3. local_terrain_cloud_ = temp_ground_cloud
//4. temp_obstacle_cloud gets used to update the occupancy grid with nearest neighbors approach
//This all needs to happen faster than the lidar can publish.
//Could become problematic if point clouds get massive

//Change of plans. I'm just going to add the pre-segmented point cloud directly into the costmap.
//THis is going to be the fastest way of doing things.
//This callback is going to have to be subscribed to /rtabmap/cloud_ground or whatever its called

//Change of plans yet again.


void DStarPlanner::updateEdgeCostsCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    ROS_INFO("D* update edge costs callback");
    ros::WallTime start_time = ros::WallTime::now();
    ros::WallDuration exe_time;

    if(!state_map_){
      ROS_INFO("D* Occ grid not yet initialized. Can't update edges in grid.");
      return;
    }
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *sample_cloud);
    
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.setInputCloud(sample_cloud);
    sor.filter(temp_cloud);
    *sample_cloud = temp_cloud;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::PointCloud<pcl::PointXYZ> sample_map_frame;
    pcl_ros::transformPointCloud("map", *sample_cloud, sample_map_frame, tf_listener_);
    
    *sample_cloud = sample_map_frame + *local_terrain_cloud_;
    
    segmentPointCloud(sample_cloud, obstacle_cloudPtr, ground_cloudPtr, 1);

    exe_time = ros::WallTime::now() - start_time;
    ROS_INFO("D* updateEdge: Segmenting point cloud %u %u", exe_time.sec, exe_time.nsec);
    
    
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "map";
    
    //pcl::toROSMsg(*sample_cloud, cloud_msg);
    //cloud_pub1_.publish(cloud_msg);
    
    //pcl::toROSMsg(*ground_cloudPtr, cloud_msg);
    //cloud_pub2_.publish(cloud_msg);
    
    pcl::toROSMsg(*obstacle_cloudPtr, cloud_msg);
    cloud_pub3_.publish(cloud_msg);
        
    if(obstacle_cloudPtr->points.empty()){
      return;
    }
    
    for(unsigned i = 0; i < obstacle_cloudPtr->points.size(); i++){
        obstacle_cloudPtr->points[i].z = 0;
    }
    
    //one alternative would be to cluster obstacle point cloud and then compute radius and then do sparse update of point cloud.

    /*
    pcl::search::KdTree<pcl::PointXYZ>::Ptr obs_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    obs_tree->setInputCloud(obstacle_cloudPtr);
    obs_tree->setSortedResults(true);
    
    //num nearest neighbors
    std::vector<int> pointIdxKNNSearch(max_neighbors);
    std::vector<float> pointKNNSquaredDistance(max_neighbors);
    
    pcl::PointXYZ searchPoint;
    */

    private_nh_->getParam("/LocalMap/occupancy_threshold", occupancy_threshold_);
    
    unsigned max_neighbors = 16;
    float temp_occ;
    int sum;
    int num_neighbors;
    unsigned offset;
    
    StateData temp_state_data;
    std::vector<StateData> update_nodes;

    exe_time = ros::WallTime::now() - start_time;
    ROS_INFO("D* updateEdge: Doing nearest neighbor grid update %u %u", exe_time.sec, exe_time.nsec);
    
    ROS_INFO("D* Done intitializing pcl, updating Edges in occupancy grid");

    
    unsigned *point_density_grid = new unsigned[width_*height_];
    for(unsigned y = 0; y < height_; y++){
      offset = y*width_;
      for(unsigned x = 0; x < width_; x++){
        point_density_grid[offset+x] = 0;
      }
    }
    
    for(unsigned i = 0; i < obstacle_cloudPtr->points.size(); i++){
      int x = floorf((obstacle_cloudPtr->points[i].x - x_offset_) / map_res_);
      int y = floorf((obstacle_cloudPtr->points[i].y - y_offset_) / map_res_);
      if(x < 0 || x >= width_ || y < 0 || y >= height_){
        continue;
      }
      point_density_grid[(y*width_)+x]++;
    }

    float *blur_density_grid = new float[width_*height_];
    const int kernel_size = 5;
    const float sigma_sq = 12;
    for(int i = 0; i < height_; i++){
      for(int j = 0; j < width_; j++){
        
        float dist_sq;
        float weight;
        float total_weight = 0;
        float occ_sum = 0;
        
        for(int k = std::max((int)0,(int)(i-kernel_size)); k <= std::min(int(height_-1),int(i+kernel_size)); k++){
            for(int m = std::max((int)0,int(j-kernel_size)); m <= std::min(int(width_-1),int(j+kernel_size)); m++){
              float dx = m - j;
              float dy = k - i;
                
              dist_sq = (dx*dx) + (dy*dy);
              weight = expf(-1.0*dist_sq/sigma_sq);
              occ_sum += weight*point_density_grid[(k*width_) + m];
              total_weight += weight;
              //occ_sum = std::max(occ_sum, (float)point_density_grid[(k*width_) + m]);
            }
        }
        blur_density_grid[(i*width_) + j] = occ_sum / total_weight;
      }
    }

    
    for(unsigned y = 0; y < height_; y++){
      offset = y*width_;
      for(unsigned x = 0; x < width_; x++){
        temp_occ = blur_density_grid[offset+x] / max_neighbors;
        //if(fabs(temp_occ - state_map_[offset+x].occupancy) > .001){
        if(temp_occ > occupancy_threshold_){
          temp_state_data.x = x;
          temp_state_data.y = y;
          temp_state_data.occupancy = temp_occ;
          update_nodes.push_back(temp_state_data);
          drawObstacle(&state_map_[offset+x], 0);
        }
      }
    }
    
    
    
    /*
    for(unsigned y = 0; y < height_; y++){
        offset = y*width_;
        for(unsigned x = 0; x < width_; x++){
            sum = 0;
            
            searchPoint.x = (x*map_res_) + x_offset_;
            searchPoint.y = (y*map_res_) + y_offset_;
            searchPoint.z = 0;
            
            num_neighbors = obs_tree->nearestKSearch(searchPoint, max_neighbors, pointIdxKNNSearch, pointKNNSquaredDistance);
            for(unsigned i = 0; i < num_neighbors; i++){
              if(sqrtf(pointKNNSquaredDistance[i]) < (map_res_)){
                    sum++;
                }
            }
            
            temp_occ = sum / (float)max_neighbors;
            if(fabs(temp_occ - state_map_[offset+x].occupancy) > .01){
              temp_state_data.x = x;
              temp_state_data.y = y;
              temp_state_data.occupancy = temp_occ;             //if(sum > occupancy_threshold_){
              update_nodes.push_back(temp_state_data);
            }
            if(state_map_[offset+x].occupancy >= occupancy_threshold_){
              drawObstacle(&state_map_[offset+x], 0);
            }
            else{
              drawObstacle(&state_map_[offset+x], 1);
            }
        }
    }
    */
    ROS_INFO("D* updateEdge: Done with detecting new states to update. Entering critical section now");
    
    
    //CRITICAL SECTION
    {
        std::lock_guard<std::mutex> lock(update_mu_);
        for(unsigned i = 0; i < update_nodes.size(); i++){
          update_nodes_.push_back(update_nodes[i]);
        }
    }
    
    delete[] blur_density_grid;
    delete[] point_density_grid;
    
    exe_time = ros::WallTime::now() - start_time;
    ROS_INFO("D* updateEdge: exiting critical section %u %u", exe_time.sec, exe_time.nsec);
}


/*
void DStarPlanner::updateEdgeCostsCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    ROS_INFO("D* get global cloud callback");
    ros::WallTime start_time = ros::WallTime::now();
        
    pcl::PointCloud<pcl::PointXYZ> full_cloud;
    pcl::fromROSMsg(*msg, full_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPtr = full_cloud;
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

    segmentPointCloud(cloudPtr, global_obstacle_cloud_, ground_cloudPtr);

    for(unsigned i = 0; i < global_obstacle_cloud_->points.size(); i++){
        global_obstacle_cloud_->points[i].z = 0;
    }
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr obs_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    obs_tree->setInputCloud(global_obstacle_cloud_);
    obs_tree->setSortedResults(true);
    
    unsigned max_neighbors = 16; //num nearest neighbors
    std::vector<int> pointIdxKNNSearch(max_neighbors);
    std::vector<float> pointKNNSquaredDistance(max_neighbors);
    
    pcl::PointXYZ searchPoint;
    float temp_occ;
    int sum;
    int num_neighbors;
    unsigned offset;
    
    StateData temp_state_data;
    std::vector<StateData> update_nodes;
    
    ROS_INFO("D* Done intitializing pcl, updating Edges in occupancy grid");
    if(!state_map_){
      ROS_INFO("D* Occ grid not yet initialized. Can't update edges in grid.");
      return;
    }
    for(unsigned y = 0; y < height_; y++){
        offset = y*width_;
        for(unsigned x = 0; x < width_; x++){
            sum = 0;
            
            searchPoint.x = (x*map_res_) + x_offset_;
            searchPoint.y = (y*map_res_) + y_offset_;
            searchPoint.z = 0;
            
            num_neighbors = obs_tree->nearestKSearch(searchPoint, max_neighbors, pointIdxKNNSearch, pointKNNSquaredDistance);
            for(unsigned i = 0; i < num_neighbors; i++){
                if(sqrtf(pointKNNSquaredDistance[i]) < map_res_){
                    sum++;
                }
            }
            
            temp_occ = sum / (float)max_neighbors;
            if(temp_occ != state_map_[offset+x].occupancy){
              temp_state_data.x = x;
              temp_state_data.y = y;
              temp_state_data.occupancy = temp_occ;             //if(sum > occupancy_threshold_){
              update_nodes.push_back(temp_state_data);
            }
        }
    }
    
    
    
    //CRITICAL SECTION
    {
        std::lock_guard<std::mutex> lock(update_mu_);
        for(unsigned i = 0; i < update_nodes.size(); i++){
          update_nodes_.push_back(update_nodes[i]);
        }
    }
    
    ros::WallDuration exe_time = ros::WallTime::now() - start_time;
    ROS_INFO("D* updateEdge: exiting critical section %u %u", exe_time.sec, exe_time.nsec);
}
*/

//Done: process aggregate point cloud
//This function has one job. Segment the full aggregated point cloud into ground and terrain
//Having these will help segment the for the local grid.
//Computes the global_terrain_cloud_ and global_obstacle_cloud_
void DStarPlanner::getGlobalCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    ROS_INFO("D* get global cloud callback");
    pcl::PointCloud<pcl::PointXYZ> full_cloud;
    pcl::fromROSMsg(*msg, full_cloud);
    
    ROS_INFO("D* global cloud frame %s", msg->header.frame_id.c_str());
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPtr = full_cloud;
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

    segmentPointCloud(cloudPtr, global_obstacle_cloud_, ground_cloudPtr, 0);
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr mls_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

    float radius;
    private_nh_->getParam("/TerrainMap/filter_radius", radius);
        
    mls.setInputCloud(ground_cloudPtr);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(mls_tree);
    mls.setSearchRadius(radius);
    mls.setComputeNormals(false);
    
    //ROS_INFO("D* about to smooth the point cloud");
    mls.process(*global_terrain_cloud_);
    ROS_INFO("D* point cloud smoothed");

    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    
    sor.setInputCloud(global_terrain_cloud_);
    sor.filter(temp_cloud);
    *global_terrain_cloud_ = temp_cloud;
    
    has_init_map_ = 1;
}

void DStarPlanner::segmentPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloudPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloudPtr, int local){
    float normal_radius;
    float smoothness_threshold;
    float curvature_threshold;
    int num_neighbors;
    
    private_nh_->getParam("/TerrainMap/normal_radius", normal_radius);
    private_nh_->getParam("/TerrainMap/num_neighbors", num_neighbors);

    if(local){
      private_nh_->getParam("/LocalMap/curvature_threshold", curvature_threshold);
      private_nh_->getParam("/LocalMap/smoothness_threshold", smoothness_threshold);
    }
    else{
      private_nh_->getParam("/TerrainMap/curvature_threshold", curvature_threshold);
      private_nh_->getParam("/TerrainMap/smoothness_threshold", smoothness_threshold);
    }
    
    //ROS_INFO("D* Starting normal estimation");
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloudPtr);
    normal_estimator.setRadiusSearch (normal_radius);
    normal_estimator.compute (*normals);

    //ROS_INFO("D* Done estimating normals, onto region growing");
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (num_neighbors);
    reg.setInputCloud (cloudPtr);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (smoothness_threshold);
    reg.setCurvatureThreshold (curvature_threshold);
    
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    ROS_INFO("D* Num clusters %lu", clusters.size());
    
    
    unsigned biggest_cluster = 0;
    unsigned most_points = clusters[0].indices.size();
    unsigned temp_num_points;
    for(unsigned i = 1; i < clusters.size(); i++){
        temp_num_points = clusters[i].indices.size();
        if(temp_num_points > most_points){
            most_points = temp_num_points;
            biggest_cluster = i;
        }
    }
    
    ROS_INFO("D* Biggest cluster is size %lu", clusters[biggest_cluster].indices.size());
    
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices());
    *ground_indices = clusters[biggest_cluster];
    pcl::ExtractIndices<pcl::PointXYZ> extract_ground;
    extract_ground.setInputCloud(cloudPtr);
    extract_ground.setIndices(ground_indices);
    extract_ground.setNegative(false);
    extract_ground.filter(*ground_cloudPtr);
    extract_ground.setNegative(true);
    extract_ground.filter(*obstacle_cloudPtr);
    
    //*cloudPtr = *ground_cloudPtr;
}

//May need to check how getRobotPose works. It could block while it waits to hear from odom.
//Which would really slow things down.
Vector2f DStarPlanner::getCurrentPose(){
    Vector2f current_pose;
    geometry_msgs::PoseStamped pose;
    costmap_ros_->getRobotPose(pose);
    current_pose[0] = pose.pose.position.x;
    current_pose[1] = pose.pose.position.y;
    return current_pose;
}

//Done: init the occupancy grid.
//1. Use a window filter to set the local terrain and obstacle point clouds
//2. Build an initial occupancy grid the same way you did for the global grid.
void DStarPlanner::initOccupancyGrid(Vector2f start, Vector2f goal){
    ROS_INFO("D* init occupancy grid");
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> out_cloud;
    pcl::PassThrough<pcl::PointXYZ> pass;
    
    pass.setInputCloud(global_terrain_cloud_);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_offset_,x_offset_ + x_range_);
    pass.filter(out_cloud);
    *in_cloud = out_cloud;

    pass.setInputCloud(in_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_offset_,y_offset_ + y_range_);
    pass.filter(out_cloud);
    *local_terrain_cloud_ = out_cloud;
    
    //ROS_INFO("D* global_obstacle_cloud_ %lu: x_offset_ %f x_range_ %f y_offset_ %f y_range_ %f", global_obstacle_cloud_->points.size(), x_offset_, x_range_, y_offset_, y_range_);
    
    pass.setInputCloud(global_obstacle_cloud_);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_offset_,x_offset_ + x_range_);
    pass.filter(out_cloud);
    *in_cloud = out_cloud;
    
    pass.setInputCloud(in_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_offset_,y_offset_ + y_range_);
    pass.filter(out_cloud);
    *local_obstacle_cloud_ = out_cloud;

    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr obs_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    obs_tree->setInputCloud(local_obstacle_cloud_);
    obs_tree->setSortedResults(true);

    unsigned max_neighbors = 16; //num nearest neighbors
    std::vector<int> pointIdxKNNSearch(max_neighbors);
    std::vector<float> pointKNNSquaredDistance(max_neighbors);
    
    pcl::PointXYZ searchPoint;
    int sum;
    int num_neighbors;
    unsigned offset;

    ROS_INFO("D* init Occupancy Grid threshold %f", occupancy_threshold_);
    
    //ROS_INFO("D* Initializing occupancy grid");
    for(unsigned y = 0; y < height_; y++){
        offset = y*width_;
        for(unsigned x = 0; x < width_; x++){
            sum = 0;
            
            searchPoint.x = (x*map_res_) + x_offset_;
            searchPoint.y = (y*map_res_) + y_offset_;
            searchPoint.z = 0;
            
            num_neighbors = obs_tree->nearestKSearch(searchPoint, max_neighbors, pointIdxKNNSearch, pointKNNSquaredDistance);
            for(unsigned i = 0; i < num_neighbors; i++){
              if(sqrtf(pointKNNSquaredDistance[i]) < (map_res_)){
                    sum++;
                }
            }
            state_map_[offset+x].occupancy = sum/(float)max_neighbors; //(float)std::min(sum, 4);

        }

    }
    
    ROS_INFO("D* Done initializing occupancy grid");
}


int DStarPlanner::initPlanner(Vector2f start, Vector2f goal){
    ROS_INFO("D* init planner");
    open_list_.clear();

    init_curr_wp_ = 1; //this is required to set curr_wp_ to origin for the control system when the robot is at the very start of the path plan
    
    //Following code sets up occupancy grid properties
    //Occupancy grid width width and height are twice the distance from start to goal.
    //x and y_offset are used to make sure start and goal are centered in the graph.
    //x and y_offset are the   offset of the occ_grid
    x_range_ = std::max(2*fabs(start[0] - goal[0]), 10.0f);
    x_offset_ = std::min(start[0], goal[0]) - (x_range_*.25);
    
    y_range_ = std::max(2*fabs(start[1] - goal[1]), 10.0f);
    y_offset_ = std::min(start[1], goal[1]) - (y_range_*.25);
    
    width_ = ceilf(x_range_ / map_res_);
    height_ = ceilf(y_range_ / map_res_);
    
    
    
    if(state_map_){
        delete[] state_map_;
    }
    state_map_ = new StateData[width_*height_];
    
    unsigned offset;
    for(unsigned i = 0; i < height_; i++){
        offset = width_*i;
        for(unsigned j = 0; j < width_; j++){
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
    initOccupancyGrid(start, goal);
    
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
    private_nh_->getParam("/move_base/DStarPlanner/goal_tol", goal_tol_);
    private_nh_->getParam("/LocalMap/occupancy_threshold", occupancy_threshold_);
    //private_nh_->getParam("/move_base/local_costmap/pcl_topic", pcl_topic);

    //SECTION is going to be for building the global map.
    global_terrain_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    global_obstacle_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    local_terrain_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    local_obstacle_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);    
    
    has_init_map_ = 0;
    ros::Subscriber init_sub = private_nh_->subscribe<sensor_msgs::PointCloud2>("/rtabmap/cloud_map", 100, &DStarPlanner::getGlobalCloudCallback, this);
    do{
      //ros::spinOnce();
        loop_rate.sleep();
    } while(!has_init_map_);
    init_sub.shutdown();
    //END global map section
    ROS_INFO("D* Got global cloud");
    
    
    planner_failed_ = 0;
    //Start asynchronous callback thread. Will listen for new lidar point clouds and it will
    //provide updated edge_costs for D* as the backpointers are followed
    //ros::CallbackQueue pcl_obs_queue_;
    //ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::String>("/rtabmap/cloud_obstacles", 1000, DStarPlanner::updateEdgeCostsCallback, std::make_shared<auvsl::DStarPlanner>(this), &pcl_obs_queue_);
    
    pcl_sub_ = private_nh_->subscribe<sensor_msgs::PointCloud2>("/mid/points", 1, &DStarPlanner::updateEdgeCostsCallback, this);
    cloud_pub1_ = private_nh_->advertise<sensor_msgs::PointCloud2>("local_cloud", 1000);
    cloud_pub2_ = private_nh_->advertise<sensor_msgs::PointCloud2>("ground_cloud", 1000);
    cloud_pub3_ = private_nh_->advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 1000);
    initialized_ = 1;
    //ROS_INFO("init over");
    
    ROS_INFO("D* Entering DStar Main Thread");
    Vector2f curr_pose;
    Vector2f X_pos;
    StateData* X_state;
    StateData* temp_start;
    StateData* temp_goal;
    
    std::vector<StateData*> actual_path;
    
    ROS_INFO("D* Waiting for global plan");
    while(global_waypoints_.empty()){
      loop_rate.sleep();
    }
    ROS_INFO("D* Got global plan");
    
    
    ROS_INFO("D* num gobal waypoitns %lu", global_waypoints_.size());
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
    
    pcl_sub_.shutdown();
    
    return;
}

int DStarPlanner::stepPlanner(StateData*& robot_state, Vector2f &X_robot){
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
        unsigned m_idx = (my*width_) + mx;
        
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
    static Vector2f curr_wp;
    
    ROS_INFO("D* follow back pointer");
    unsigned x_actual; //map physical location to local map index
    unsigned y_actual;
    Vector2f current_pose = getCurrentPose();
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
          Vector2f temp_vec = getRealPosition(temp_state->x, temp_state->y);
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


//Physically follow backpointer
/*
void DStarPlanner::followBackpointer(StateData*& robot_state){
    ROS_INFO("D* follow back pointer");
    unsigned x_actual; //map physical location to local map index
    unsigned y_actual;
    
    unsigned x_desired = robot_state->b_ptr->x;
    unsigned y_desired = robot_state->b_ptr->y;
    
    ROS_INFO("D* followBackPointer: setting waypoints for control system");
    //Critical Section. Sets local waypoints for control system to use as lookahead.
    //Need to guard local_waypoints which is being read in the computeVelocityCommands function/move_base thread.
    {
        std::lock_guard<std::mutex> lock(wp_mu_);
        StateData* temp_state = robot_state;
        
        local_waypoints_.clear();
        for(unsigned i = 0; i < lookahead_len_; i++){
          temp_state = temp_state->b_ptr; //start with first b_ptr so ignore current pos because we are already there
          if(temp_state == NULL){
            break;
          }
          Vector2f temp_vec = getRealPosition(temp_state->x, temp_state->y);
          local_waypoints_.push_back(temp_vec);
        }
    }
    ROS_INFO("D* followBackPointer: Num Waypoints %lu", local_waypoints_.size());
    ROS_INFO("D* followBackPointer: actually following back pointer and also listening for edgeCost updates");
    float dx;
    float dy;
    ros::Rate loop_rate(10); //make sure to call faster than lidar publishes
    do{
        Vector2f current_pose = getCurrentPose();
        getMapIdx(current_pose, x_actual, y_actual);
        
        Vector2f goal_pose = getRealPosition(x_desired, y_desired);
        
        ROS_INFO("D* current_pose %0.3f %0.3f   goal pose %0.3f %0.3f   %u %u   %u %u", current_pose[0], current_pose[1], goal_pose[0], goal_pose[1], x_actual, y_actual, x_desired, y_desired);
        drawRobotPos(x_actual, y_actual);

        dx = goal_pose[0] - current_pose[0];
        dy = goal_pose[1] - current_pose[1];

        //ros::spinOnce();
        loop_rate.sleep();
    } while(!(x_actual == x_desired && y_actual == y_desired) && !(sqrtf((dx*dx)+(dy*dy)) < .2));
    ROS_INFO("D* followBackPointer: Back Pointer = Followed");
    robot_state = robot_state->b_ptr;
}
*/

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
  
  
  
  Vector2f pos;
  for(int i = 0; i < 8; i++){
    unsigned nx = X->x + dxdy[i][0]; //It won't go negative, but it will overflow and then be outside the range of the grid
    unsigned ny = X->y + dxdy[i][1];
    //Make sure state is within the grid. Or else a segfault will occur
    if((nx >= 0) && (nx < width_) && (ny >= 0) && (ny < height_)){
      pos = getRealPosition(nx, ny);
      //ROS_INFO("D* idx %u %u   pos %f %f", nx, ny, pos[0], pos[1]);
      neighbors.push_back(&state_map_[(ny*width_)+nx]);
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

float DStarPlanner::getPathCost(Vector2f X, Vector2f G){
  //ROS_INFO("D* getPathCost");
  StateData* state = readStateMap(X[0], X[1]);
  return state->curr_cost;
}

float DStarPlanner::getMinPathCost(Vector2f X, Vector2f G){
  //ROS_INFO("D* getMinPathCost");
  StateData* state = readStateMap(X[0], X[1]);
  return state->min_cost;
}

void DStarPlanner::getMapIdx(Vector2f X, unsigned &x, unsigned &y){
    float x_scale = width_ / x_range_;
    float y_scale = height_ / y_range_;
    
    int x_int = floorf((X[0] - x_offset_)*x_scale);
    int y_int = floorf((X[1] - y_offset_)*y_scale);
    
    x = std::min(std::max(x_int, int(0)), int(width_));
    y = std::min(std::max(y_int, int(0)), int(height_));
    
    //ROS_INFO("D* getMapIdx    %f  %f      %d  %d   %u %u",   X[0], X[1],   x_int, y_int,  x, y);
}

StateData* DStarPlanner::readStateMap(float rx, float ry){
    unsigned x;
    unsigned y;
    
    getMapIdx(Vector2f(rx, ry), x, y);
    return &(state_map_[(y*width_)+x]);
}

Vector2f DStarPlanner::getRealPosition(unsigned x, unsigned y){
    float x_scale = x_range_ / width_;
    float y_scale = y_range_ / height_;
    Vector2f X;
    
    
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
    rect.id = (state->y*width_) + state->x;
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
    
    Vector2f goal = getRealPosition(state->x, state->y);
    
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
  Vector2f goal = getRealPosition(state->x, state->y);
  
  visualization_msgs::Marker rect;
  rect.header.frame_id = "map";
  rect.header.stamp = ros::Time::now();
  rect.ns = "dstar_obs";
  rect.action = visualization_msgs::Marker::ADD;
  rect.pose.orientation.w = 1.0;
  rect.pose.orientation.x = 0.0;
  rect.pose.orientation.y = 0.0;
  rect.pose.orientation.z = 0.0;
  rect.id = (state->y*width_) + state->x;
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
  
  Vector2f start = getRealPosition(state->x, state->y);
  Vector2f end = getRealPosition(state->b_ptr->x, state->b_ptr->y);

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
  point_list.id = (state->y*width_) + state->x;
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
  Vector2f goal = getRealPosition(x, y);
  
  visualization_msgs::Marker rect;
  rect.header.frame_id = "map";
  rect.header.stamp = ros::Time::now();
  rect.ns = "dstar_robot_pos";
  rect.action = visualization_msgs::Marker::ADD;
  rect.pose.orientation.w = 1.0;
  rect.pose.orientation.x = 0.0;
  rect.pose.orientation.y = 0.0;
  rect.pose.orientation.z = 0.0;
  rect.id = (y*width_) + x;
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
  
  Vector2f goal = getRealPosition(state->x, state->y);
  
  visualization_msgs::Marker rect;
  rect.header.frame_id = "map";
  rect.header.stamp = ros::Time::now();
  rect.ns = "dstar_goal";
  rect.action = visualization_msgs::Marker::ADD;
  rect.pose.orientation.w = 1.0;
  rect.pose.orientation.x = 0.0;
  rect.pose.orientation.y = 0.0;
  rect.pose.orientation.z = 0.0;
  rect.id = (state->y*width_) + state->x;
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
    Vector2f vec;
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


