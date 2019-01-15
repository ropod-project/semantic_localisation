cmake_minimum_required(VERSION 2.8.3)
project(aruco_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	rospy
	message_generation
	std_msgs
	geometry_msgs
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Aruco.msg
#   Message2.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aruco_detection
  CATKIN_DEPENDS  message_runtime
#  DEPENDS system_lib
)

#
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

