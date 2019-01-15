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
#include <pillar_detector/PillarDetectorAction.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "pillar_detector_test");
  ros::NodeHandle node;
  pillar_detector::PillarDetectorGoal pd_goal;
  pillar_detector::PillarDetectorResult pd_result;
  geometry_msgs::PoseStamped pillar_pose;
  actionlib::SimpleActionClient<pillar_detector::PillarDetectorAction> pd("/pillar_detector", true);
  //ros::Duration(10).sleep();
  ros::Rate r(.5);
  while( ros::ok )
  {
    pd.waitForServer();
    pd_goal.diameter = 0.65;
    pd.sendGoal(pd_goal);
    r.sleep();
  }
  ros::spin();
  return 0;
}