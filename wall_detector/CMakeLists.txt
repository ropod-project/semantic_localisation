cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
project(wall_detector)

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
  WallDetector.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wall_detector
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

add_executable( wall_detector_server src/wall_detector_server.cpp src/wall_detector_code.cpp)
target_link_libraries(wall_detector_server ${catkin_LIBRARIES})
add_dependencies(
  wall_detector_server	
  ${wall_detector_EXPORTED_TARGETS}
)

add_executable( wall_detector_test src/wall_detector_test.cpp)
target_link_libraries(wall_detector_test ${catkin_LIBRARIES})
add_dependencies(
  wall_detector_test	
  ${wall_detector_EXPORTED_TARGETS}
)