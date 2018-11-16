cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
project(pillar_detector)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  tf
)

## Generate actions in the 'action' folder
add_action_files(
	DIRECTORY action
  FILES
  PillarDetector.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pillar_detector
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## add_executable(detector src/detector.cpp src/circleransac.cpp)
## target_link_libraries(detector ${catkin_LIBRARIES})

add_executable( detector_server src/pillar_detector_server.cpp src/pillar_detector_code.cpp)
target_link_libraries(detector_server ${catkin_LIBRARIES})
add_dependencies(
  detector_server	
  ${pillar_detector_EXPORTED_TARGETS}
)


