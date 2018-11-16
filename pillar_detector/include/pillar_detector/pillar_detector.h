#ifndef PILLAR_DETECTOR_H
#define PILLAR_DETECTOR_H

// 
#include <pillar_detector/PillarDetectorAction.h>


// C++
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <cstdlib>
#include <math.h>
#include <iostream>
#include <sstream>
#include <queue>
#include <time.h>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include <ros/callback_queue.h>

const float  PI_F=3.14159265358979f;
const double PI  =3.141592653589793238463;

struct point{
  float x;
  float y;
};

struct polpoint{
  float r;
  float a;
}; 

class PillarDetector
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<pillar_detector::PillarDetectorAction> pillar_detector_server;
    void pillar_detector_server_execute( const pillar_detector::PillarDetectorGoalConstPtr &goal);
    void displayLaserdata(const sensor_msgs::LaserScan::ConstPtr& scan);
    pillar_detector::PillarDetectorResult result;
    ros::Subscriber scan_sub;
    std::queue<sensor_msgs::LaserScan::ConstPtr> scan_buffer_;

public:
    sensor_msgs::LaserScan::ConstPtr scan_result;
    PillarDetector();
    virtual ~PillarDetector();
    void Detect( const pillar_detector::PillarDetectorGoalConstPtr goal );
    void InitializeBuffer( std::string robot);
    std::tuple< float, float > CalcRange( geometry_msgs::PoseWithCovariance initial_guess );
    std::tuple<point,bool> FindPillarCandidate(int index_min, int index_max, float diameter, float diameter_error, int max_iter);
    polpoint scan_2_robotpol( int i);
    int Nearest_Beam_Index( float angle);
    int CalcSearchRange(polpoint p, float diameter);
};
    //tf::Quaternion q(x,y,z,w); 
    //double roll,pitch,yaw;
    //tf::Matrix3x3 m(q);
    //m.getRPY(roll, pitch,yaw);


#endif /* PILLAR_DETECTOR_H */