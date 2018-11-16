#include "/home/martin/catkin_ws_test/src/pillar_detector/include/pillar_detector/pillar_detector.h"
//#include <pillar_detector/pillar_detector.h>

PillarDetector::PillarDetector() : nh_("~"),pillar_detector_server(nh_,"/pillar_detector",
  boost::bind(&PillarDetector::pillar_detector_server_execute, this, _1),false)
{ 
  pillar_detector_server.start();
}

PillarDetector::~PillarDetector()
{
}

void PillarDetector::InitializeBuffer(std::string robot)
{
  std::string topicname ="/";
  topicname.append(robot);
  topicname.append("/laser/scan");
  scan_sub = nh_.subscribe<sensor_msgs::LaserScan>(topicname, 1, &PillarDetector::displayLaserdata, this);
}

void PillarDetector::pillar_detector_server_execute(const pillar_detector::PillarDetectorGoalConstPtr &goal)
{ 
  std::tuple<point,bool> test;
  point pc;
  result.test = 5;
  scan_result = scan_buffer_.front();
  if( scan_result!=scan_result)
  {
    ROS_INFO("no laserscan available yet");
  }
  else
  {
    Detect( goal );
    if( goal->detect == "target" && !goal->pose.empty())
    {
      ROS_INFO("Detecting target pillar(s)");
    }
    else if( goal->detect == "all" )
    {
      ROS_INFO("Detecting all pillars");
    }
    else
    {
      ROS_INFO("No detection target(s) set, default = detect all");
    }
  }
  
  
   if( true)
    {
      pillar_detector_server.setSucceeded(result);
    }
    else
    {
      pillar_detector_server.setAborted(result);
    } 
}

void PillarDetector::displayLaserdata(const sensor_msgs::LaserScan::ConstPtr& scan)
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
  ros::init(argc, argv, "pillar_detector");
  ros::NodeHandle node;
  std::string robot;
  node.getParam("pillar_detector/robot", robot);
  PillarDetector detector;
  detector.InitializeBuffer( robot );
  ros::spin();
  return 0;
}

