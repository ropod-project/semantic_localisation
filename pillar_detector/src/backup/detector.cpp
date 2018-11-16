#include <pillar_detector/pillar_detector.h>

float angle_min;
float angle_max;
float angle_increment;
//float time_increment;
float scan_time;
float range_min;
float range_max;
int n_beams;
float pillar_radius = 0.325;


void displayLaserdata(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  std::vector<float> ranges;
  angle_min 		= scan->angle_min;
  angle_max 		= scan->angle_max;
  angle_increment 	= scan->angle_increment;
  range_min		= scan->range_min;
  range_max		= scan->range_max;
  n_beams 		= scan->ranges.size();

  // unused variables
  {
  //	std::vector<float> intensities;
  //	angle_increment = scan->angle_increment;
  //	time_increment	= scan->time_increment;
  //	scan_time	= scan->scan_time;
  }

  for( int i = 0; i< n_beams; i++)
  {
    ranges.push_back(scan->ranges[i]);
    //	intensities.push_back(scan->intensities[i]);
  }
  ROS_INFO("Performing Circle RANSAC");
  CircleRansac( ranges, angle_min, angle_max, range_min, range_max, n_beams, pillar_radius);
    ros::spinOnce();
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pillar_detector");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/ropod/laser/scan", 1, displayLaserdata);

  ros::spin();
  return 0; 
}


