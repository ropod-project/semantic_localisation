#include "/home/martin/catkin_ws_test/src/semantic_localisation/pillar_detector/include/pillar_detector/pillar_detector.h"
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
  ROS_INFO("PillarDetector: initializing laser buffer");
  std::string topicname ="/";
  topicname.append(robot);
  topicname.append("/laser/scan");
  scan_sub = nh_.subscribe<sensor_msgs::LaserScan>(topicname, 1, &PillarDetector::displayLaserdata, this);
}

void PillarDetector::pillar_detector_server_execute(const pillar_detector::PillarDetectorGoalConstPtr &goal)
{ 
  //ROS_INFO("PillarDetector: Server is Ready!");
  if( scan_buffer_.empty() )
  {
    ROS_INFO("PillarDetector: nothing buffered yet");
  }
  else
  {
    scan_result = scan_buffer_.front();
    Detect( goal ); 
  }
  result.x.clear();
  result.y.clear();
  result.inlierpercentage.clear();
  int i = 0;
  if( !pillars.empty())
  {
    for( auto it_pillar = pillars.begin(); it_pillar < pillars.end(); it_pillar++)
    {
      result.x.push_back( std::get<0>(pillars[i]).x );
      result.y.push_back( std::get<0>(pillars[i]).y );
      result.inlierpercentage.push_back( std::get<1>(pillars[i]) );
      i++;
    }
  }
  pillar_detector_server.setSucceeded(result);
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

