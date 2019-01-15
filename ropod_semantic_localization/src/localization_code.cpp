#include <ropod_semantic_localization/localization.h>

void Localization::LocalPoseTracking( localarea current_area, localarea next_area, localmotion lm )
{
  ROS_INFO("Moving from area %i to area %i", current_area.id, next_area.id);
  if( lm.motion_type == "wall-follow")
  {
    ROS_INFO("Using wall-following");
    ROS_INFO("Following wall-side with id %i", lm.wall.id);
  }
  if( lm.motion_type == "junction")
  {
    ROS_INFO("Crossing junction");
    ROS_INFO("Crossing junction with id %i", lm.j.id);
  }
  if( lm.motion_type == "overtake")
  {
    ROS_INFO("Overtaking");
    ROS_INFO("Node with id %i", lm.feature.id);
  }
  if( lm.motion_type == "way-point")
  {
    ROS_INFO("Driving to waypoint");
    ROS_INFO("With id %i ", next_area.topology.id); 
  }
  if( lm.motion_type == "enter-door")
  {
    ROS_INFO("Entering door");
    ROS_INFO("Entering doors not implemented yet");
  }
}

void Localization::DetectTransition( localarea la )
{
 
}

void Localization::UpdateOdom2MapCorrection()
{
  odom2map_correction.header.frame_id ="map";
  odom2map_correction.child_frame_id = odomtopic;
  odom2map_correction.header.stamp = ros::Time::now();
  odom2map_correction.pose.pose.position.x = 0.0;
  odom2map_correction.pose.pose.position.y = 0.0;
  odom2map_correction.pose.pose.orientation.x = 0.0;
  odom2map_correction.pose.pose.orientation.y = 0.0;
  odom2map_correction.pose.pose.orientation.z = 0.0;
  odom2map_correction.pose.pose.orientation.w = 1.0;  
}

void Localization::test()
{ 
  tf::StampedTransform transform;
  pillar_detector::PillarDetectorGoal pd_goal;
  wall_detector::WallDetectorGoal wd_goal;
  ros::Rate r(10);
  while(ros::ok)
  {
    ros::spinOnce();  
    // Create Goal ID
    try
    {
      listener.lookupTransform("map", baselinktf, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what() );
      ros::Duration(0.2).sleep();
      continue;
    }
    ROS_INFO("Robot x in world frame : %f", transform.getOrigin().x());
    ROS_INFO("Robot y in world frame : %f", transform.getOrigin().y());
    pd_goal.diameter = 0.65;
    SendGoalPD(pd_goal);
    SendGoalWD( wd_goal);
    r.sleep();
  }
}

void Localization::InitializeOdomBuffer()
{
  odom_sub = nh_.subscribe<nav_msgs::Odometry>(odomtopic, 1, &Localization::BufferOdomData, this);
}

void Localization::InitializeMarkerBuffer()
{
  marker_sub = nh_.subscribe<aruco_detection::Aruco>("/ArUco_Marker", 1, &Localization::BufferMarkers, this);
}

void Localization::BufferOdomData(const nav_msgs::Odometry::ConstPtr& odom)
{
  if( odom_buffer_.empty())
  {
    odom_buffer_.push( odom );
  }
  else
  {
    while( !odom_buffer_.empty() )
    {
      odom_buffer_.pop();
    }
    odom_buffer_.push( odom );
  }
}

void Localization::BufferMarkers(const aruco_detection::Aruco::ConstPtr& markers)
{
  if( marker_buffer_.empty())
  {
    marker_buffer_.push( markers );
  }
  else
  {
    while( !marker_buffer_.empty() )
    {
      marker_buffer_.pop();
    }
    marker_buffer_.push( markers );
  }
}