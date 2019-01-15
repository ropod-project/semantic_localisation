// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// SEMANTIC LOCALIZATION
#include <wall_detector/WallDetectorAction.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "wall_detector_test");
  ros::NodeHandle node;
  wall_detector::WallDetectorGoal wd_goal;
  wall_detector::WallDetectorResult wd_result;
  geometry_msgs::PoseStamped dummy;
  actionlib::SimpleActionClient<wall_detector::WallDetectorAction> wd("/wall_detector", true);
  //ros::Duration(10).sleep();
  ros::Rate r(2);
  while( ros::ok )
  {
    wd.sendGoal(wd_goal);
    r.sleep();
  }
  ros::spin();
  return 0;
}