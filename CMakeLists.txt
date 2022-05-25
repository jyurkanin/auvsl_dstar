cmake_minimum_required(VERSION 3.0.2)
project(auvsl_dstar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  auvsl_control
)


## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES auvsl_planner
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime auvsl_control
#  DEPENDS rbdl ompl octomap  #One day I may have to figure this out.
)



#set(CMAKE_CXX_FLAGS "-Wl,--no-undefined")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${catkin_INCLUDE_DIRS}
)



include_directories(include/auvsl_dstar)

#find_package(Eigen REQUIRED)
include_directories(/usr/include/eigen3)


add_executable(auvsl_dstar_node
  src/auvsl_dstar_node.cpp
  src/ControlSystem.cpp
  src/DStarPlanner.cpp
  src/TerrainMap.cpp
  )


add_dependencies(auvsl_dstar_node auvsl_control_generate_messages_cpp)

target_link_libraries(auvsl_dstar_node ${pluginlib_LIBRARIES})
target_link_libraries(auvsl_dstar_node ${catkin_LIBRARIES})
set_target_properties(auvsl_dstar_node PROPERTIES COMPILE_FLAGS "-O3 -g")
