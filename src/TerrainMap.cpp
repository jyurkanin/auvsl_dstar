#include "TerrainMap.h"

#include <math.h>
#include <algorithm>
#include <stdlib.h>

#include <ros/ros.h>
#include <ompl/util/RandomNumbers.h>



//simple helper function.
int isPosInBox(float x, float y, Rectangle *rect){
  return (x > rect->x) && (x < (rect->x + rect->width)) &&
         (y > rect->y) && (y < (rect->y + rect->height));
}





SimpleTerrainMap::SimpleTerrainMap(){
    Ymin = -100;
    Ymax = 100;
    Xmin = -100;
    Xmax = 100;
}

SimpleTerrainMap::~SimpleTerrainMap(){
  for(unsigned i = 0; i < obstacles.size(); i++){
    delete obstacles[i];
  }
  for(unsigned i = 0; i < unknown_obstacles.size(); i++){
    delete unknown_obstacles[i];
  }

  unknown_obstacles.clear();
  obstacles.clear();
}


BekkerData soil_table[5] = {
    {29.76,      2083,   .8,      0, 22.5*M_PI/180.0, "Medium Soil"},
    {102,        5301,   .8,     .2, 31.1*M_PI/180.0, "Sand"},
    {30.08,      499.7,  .7,      0, 14.0*M_PI/180.0, "Clay"},
    {33.0,       2100.0, .71,     0, .37, "Rantoul"},
    {29.761774,  2000.0, .435841, 0, .37, "Dataset"}
};

BekkerData lookup_soil_table(int index){
    return soil_table[index];
}


BekkerData SimpleTerrainMap::getSoilDataAt(float x, float y) const{
  return lookup_soil_table(4);
  //return test_bekker_data_;
}


float SimpleTerrainMap::getAltitude(float x, float y, float z_guess) const{
  //return .2*x;//x;//.1*sinf(x*.2);//
  //return fmax(0.0f,sinf(x*.5));//(4 / (2*sqrtf((x*x) + (y*y))+1));
  //return fmax(0, x-2);//sinf(x*.5);
  //return (1e-3f*rand()/RAND_MAX);
  return 0;
}

void SimpleTerrainMap::generateObstacles(){
  //Just going to try a couple seed until I get a good obstacle field.
  //Not going to error check, i.e. if the starting point is inside an
  //obstacle or whatever. Ill just check and try a different seed.

  ompl::RNG rng;
  const int max_obstacles = 8;

  for(int i = 0; i < max_obstacles; i++){
    Rectangle *rect = new Rectangle();

    rect->width = rng.uniformReal(5, 10);
    rect->height = rng.uniformReal(40, 80);

    rect->x = -80 + (160*i/(max_obstacles-1)); //rng.uniformReal(-100, 100);
    rect->y = rng.uniformReal(-50, 50) - rect->height/2;


    obstacles.push_back(rect);
  }

}


void SimpleTerrainMap::generateUnknownObstacles(){
  ompl::RNG rng;
  const int max_obstacles = 1000;

  for(int i = 0; i < max_obstacles; i++){
    Rectangle *rect = new Rectangle();

    if(rng.uniformBool()){
      rect->width = rng.uniformReal(1, 4);
      rect->height = 1;
    }
    else{
      rect->width = 1;
      rect->height = rng.uniformReal(1, 4);
    }


    rect->x = rng.uniformReal(-80, 80) - rect->width/2;
    rect->y = rng.uniformReal(-80, 80) - rect->height/2;


    unknown_obstacles.push_back(rect);
  }

}


void SimpleTerrainMap::detectAllObstacles(){
  for(unsigned i = 0; i < unknown_obstacles.size(); i++){
    obstacles.push_back(unknown_obstacles[i]);
  }
  unknown_obstacles.clear();
}


//"Detect" unknown obstacles that are within a certain distance.
//arguments are the vehicles position.
int SimpleTerrainMap::detectObstacles(float x, float y){
  //https://stackoverflow.com/questions/5254838/calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
  int got_new = 0;
  
  const float SENSOR_RANGE = 2; //Robot will detect any obstacles within 2 meters
  for(unsigned i = 0; i < unknown_obstacles.size(); i++){
    float rect_min_x = unknown_obstacles[i]->x;
    float rect_max_x = unknown_obstacles[i]->x + unknown_obstacles[i]->width;

    float rect_min_y = unknown_obstacles[i]->y;
    float rect_max_y = unknown_obstacles[i]->y + unknown_obstacles[i]->height;
    
    float dx = std::max(std::max(rect_min_x - x, x - rect_max_x), 0.0f);
    float dy = std::max(std::max(rect_min_y - y, y - rect_max_y), 0.0f);

    //check if obstacle is in range of sensors.
    if((dx*dx + dy*dy) < (SENSOR_RANGE*SENSOR_RANGE)){
      obstacles.push_back(unknown_obstacles[i]);
      unknown_obstacles.erase(unknown_obstacles.begin() + i);
      got_new = 1;
    }
  }

  return got_new;
}

std::vector<Rectangle*> SimpleTerrainMap::getObstacles() const{
    return obstacles;
}

int SimpleTerrainMap::isRealStateValid(float x, float y){
  for(unsigned i = 0; i < unknown_obstacles.size(); i++){
    if(isPosInBox(x, y, unknown_obstacles[i])){
      return 0;
    }
  }

  return isStateValid(x,y);
}

//Is state valid based on known information. Does not include unknown obstacles.
//Only checks if state is valid based on map information.
//This function does not validate actual vehicle state. Just x y position.
int SimpleTerrainMap::isStateValid(float x, float y) const{
  for(unsigned i = 0; i < obstacles.size(); i++){
    if(isPosInBox(x, y, obstacles[i])){
      //ROS_INFO("INVALID STATE: OBSTACLE %f %f", x, y);
      return 0;
    }
  }

  /*
  for(unsigned i = 0; i < unknown_obstacles.size(); i++){
    if(isPosInBox(x, y, unknown_obstacles[i])){
      //ROS_INFO("INVALID STATE: OBSTACLE %f %f", x, y);
      return 0;
    }
  }
  */

  if((x > Xmin) && (x < Xmax) &&
     (y > Ymin) && (y < Ymax)){
    return 1;
  }
  else{
    //ROS_INFO("INVALID STATE: OFF MAP %f %f", x, y);
    return 0;
  }
}


void SimpleTerrainMap::getBounds(float &max_x, float &min_x, float &max_y, float &min_y) const{
  max_x = Xmax; //(cols_*map_res_) + x_origin_;
  min_x = Xmin;

  max_y = Ymax; //(rows_*map_res_) + y_origin_;
  min_y = Ymin;
}
