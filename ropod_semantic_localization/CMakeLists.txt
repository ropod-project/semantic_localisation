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
  tf
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  Localization.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ropod_semantic_localization
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(semantic_planner src/planner.cpp)
add_dependencies(semantic_planner ropod_ros_msgs_gencpp)
target_link_libraries(semantic_planner ${LIBS} ${catkin_LIBRARIES})

add_executable(ropod_semantic_localization src/localization.cpp src/querying.cpp src/visualization.cpp src/localization_code.cpp)
add_dependencies(ropod_semantic_localization ropod_ros_msgs_gencpp)
target_link_libraries(ropod_semantic_localization ${LIBS} ${catkin_LIBRARIES})

add_executable(tf_broadcaster_localization src/tf_broadcaster_localization.cpp)
add_dependencies(tf_broadcaster_localization ropod_ros_msgs_gencpp)
target_link_libraries(tf_broadcaster_localization ${LIBS} ${catkin_LIBRARIES})
