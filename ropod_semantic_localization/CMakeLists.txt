cmake_minimum_required(VERSION 2.8.3)
project(ropod_semantic_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  ropod_ros_msgs
  roscpp
)

catkin_package(CATKIN_DEPENDS
  roscpp
  ropod_ros_msgs
  actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ropod_semantic_localization src/localization.cpp src/querying.cpp src/visualization.cpp)
add_dependencies(ropod_semantic_localization ropod_ros_msgs_gencpp)
target_link_libraries(ropod_semantic_localization ${LIBS} ${catkin_LIBRARIES})



