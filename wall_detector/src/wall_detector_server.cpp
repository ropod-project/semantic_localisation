#include "/home/martin/catkin_ws_test/src/semantic_localisation/wall_detector/include/wall_detector/wall_detector.h"
//#include <wall_detector/wall_detector.h>

WallDetector::WallDetector() : nh_("~"),wall_detector_server(nh_,"/wall_detector",
  boost::bind(&WallDetector::wall_detector_server_execute, this, _1),false)
{ 
  wall_detector_server.start();
}

WallDetector::~WallDetector()
{
}

void WallDetector::InitializeBuffer(std::string robot)
{
  std::string topicname ="/";
  topicname.append(robot);
  topicname.append("/laser/scan");
  scan_sub = nh_.subscribe<sensor_msgs::LaserScan>(topicname, 1, &WallDetector::displayLaserdata, this);
}

void WallDetector::wall_detector_server_execute(const wall_detector::WallDetectorGoalConstPtr &goal)
{ 
  //ROS_INFO("WallDetector: Server is ready!");
  if( scan_buffer_.empty() )
  {
    ROS_INFO("WallDetector: nothing buffered yet");
  }
  else
  {
    scan_result = scan_buffer_.front();
    Detect( goal );
  }
  result.x1.clear();
  result.y1.clear();
  result.x2.clear();
  result.y2.clear();
  result.inlierpercentage.clear();
  int i = 0;
  if( !walls.empty())
  {
    for( auto it_wall = walls.begin(); it_wall < walls.end(); it_wall++)
    {
      result.x1.push_back( walls[i].c1.x );
      result.y1.push_back( walls[i].c1.y );
      result.x2.push_back( walls[i].c2.x );
      result.y2.push_back( walls[i].c2.y );
      result.inlierpercentage.push_back( walls[i].inlierpercentage );
      i++;
    }
  }
  wall_detector_server.setSucceeded(result);
}

void WallDetector::displayLaserdata(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  if( scan_buffer_.empty() )
  {
    scan_buffer_.push(scan);
  }
  else
  {
    while( !scan_buffer_.empty() )
    {
      scan_buffer_.pop();
    }
    scan_buffer_.push(scan);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wall_detector");
  ros::NodeHandle node;
  std::string robot;
  node.getParam("wall_detector/robot", robot);
  WallDetector detector;
  detector.InitializeBuffer( robot );
  ros::spin();
  return 0;
}

