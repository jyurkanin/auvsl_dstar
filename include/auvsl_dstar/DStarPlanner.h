#include "TerrainMap.h"
#include "ControlSystem.h"

#include <Eigen/Dense>

#include <vector>
#include <mutex>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <boost/thread/thread.hpp>
#include <rosgraph_msgs/Clock.h>

/*
 * Implementation for this algorithm is from
 * https://www.ri.cmu.edu/pub_files/pub3/stentz_anthony__tony__1994_2/stentz_anthony__tony__1994_2.pdf
 * And also partially from
 * https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf
 * starting at slide 31. I'm using this resource instead of the original Stenz paper
 * because this slide show gives much more detailed pseudo code and is much easier to
 * understand.
 */

using namespace Eigen;


namespace auvsl{
    
enum TAG {NEW, CLOSED, OPEN};
enum STATE_TYPE {RAISE, LOWER, NORMAL};


#define EPSILON 1e-5

//This is not efficient
struct StateData{
    float min_cost;
    float curr_cost;
    struct StateData *b_ptr;
    unsigned x;
    unsigned y;
    TAG tag;
    float occupancy;
};

typedef struct StateData StateData;

class DStarPlanner : public nav_core::BaseLocalPlanner{
public:
    DStarPlanner();
    ~DStarPlanner();

    //base_local_planner virtual function overrides
    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;
    void initialize(std::string name) override;
    bool isGoalReached() override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

    //ROS related extra functions
    void followBackpointer(StateData*& robot_state);
    
    //D* related Functions
    int initPlanner(Eigen::Vector2f start, Eigen::Vector2f goal);
    void runPlanner();
    int stepPlanner(StateData*& robot_state, Eigen::Vector2f &robot_pos);
    int replan(StateData* robot_state);

    void updateEdgeCostsCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void getGlobalCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void initOccupancyGrid(Eigen::Vector2f start, Eigen::Vector2f goal);
  
    float getEdgeCost(StateData* X, StateData* Y);    //c(X)
    float getPathCost(Eigen::Vector2f X, Eigen::Vector2f G);    //h(X)
    float getMinPathCost(Eigen::Vector2f X, Eigen::Vector2f G); //Min Path Cost since state was added to open list. This is the "key function"
    void getNeighbors(std::vector<StateData*> &neighbors, StateData* X, int replan);
    void insertState(StateData* X, float path_cost);
    void deleteState(StateData *state);
    float processState(int replan);
    
    
    //Drawing related functions
    void initWindow();
    void drawStateType(StateData *state, STATE_TYPE s_type);
    void drawStateTag(StateData *state);
    void drawStateBPtr(StateData *state);
    void drawPath(StateData *start);
    void drawGoal(StateData *state);
    void drawFinishedGraph(StateData *state, std::vector<StateData*> &actual_path);
    void drawObstacle(StateData *state, int clear);
    void drawRobotPos(StateData* state);
    void drawRobotPos(unsigned x, unsigned y);
    
    //Helper functions
    void getMapIdx(Eigen::Vector2f X, unsigned &x, unsigned &y);
    StateData* readStateMap(float rx, float ry); //will perform the necessary quantization to go from floating state to grid index
    Eigen::Vector2f getRealPosition(unsigned x, unsigned y);
    Eigen::Vector2f getCurrentPose();
    int getROSPose(geometry_msgs::PoseStamped &pose);
  
    //Costmap related functions
    int isStateValid(float x, float y);
    
    
private:
    float x_range_;
    float x_offset_;

    float y_range_;
    float y_offset_;

    float occupancy_threshold_;
    float map_res_;
    
    unsigned width_;
    unsigned height_;
  
    StateData *state_map_; //states are 8 connected
    //SimpleTerrainMap *terrain_map_;
    std::vector<StateData*> open_list_; //This is going to be sorted by key function.

    //waypoints generated from global planner. Should further spaced than local waypoints
    unsigned curr_waypoint_;
    std::vector<Eigen::Vector2f> global_waypoints_;
    
    //These represent the higher resolution immediate next waypoints to reach.
    std::mutex wp_mu_;
    unsigned lookahead_len_;
    std::vector<Eigen::Vector2f> local_waypoints_;
    
    std::mutex update_mu_;
    std::vector<StateData> update_nodes_;
        
    std::ofstream log_file;
    ControlSystem *control_system_;
    
    volatile int has_pose_;
    float goal_tol_;

    volatile int has_init_map_;
    int initialized_;
     
    int init_curr_wp_;
    
    int planner_failed_;
    
    boost::thread *planner_thread_;
    ros::NodeHandle *private_nh_;
    ros::Subscriber pcl_sub_;
    ros::Publisher dstar_visual_pub_;
};


}
