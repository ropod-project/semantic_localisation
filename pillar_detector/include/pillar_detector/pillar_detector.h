#ifndef PILLAR_DETECTOR_H
#define PILLAR_DETECTOR_H

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
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <ros/callback_queue.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pillar_detector/PillarDetectorAction.h>

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
    pillar_detector::PillarDetectorFeedback feedback;
    ros::Subscriber scan_sub;
    ros::Publisher vis_pub = nh_.advertise<visualization_msgs::MarkerArray>( "detected_pillars", 0 );
    std::queue<sensor_msgs::LaserScan::ConstPtr> scan_buffer_;

public:
    sensor_msgs::LaserScan::ConstPtr scan_result;
    PillarDetector();
    virtual ~PillarDetector();
    void Detect( const pillar_detector::PillarDetectorGoalConstPtr goal );
    void InitializeBuffer( std::string robot);
    void VisualizeDetections();
    std::tuple< float, float > CalcRange( geometry_msgs::PoseStamped initial_guess );
    std::tuple<point,bool> FindPillarCandidate(int index_min, int index_max, float diameter, float diameter_error, int max_iter);
    point robotpol_2_robotcart( polpoint p);
    polpoint scan_2_robotpol( int i);
    polpoint robotcart_2_robotpol( point p);
    point FindCircleCenter( point p1, point p2, point p3, float diameter);
    int Nearest_Beam_Index( float angle);
    int CalcSearchRange(polpoint p, float diameter);
    float Calc_Distance( point a, point b);
    bool CheckInlier( point circle_center, point sample , float diameter, float diameter_error);   
    bool InRange(float r, float min, float max);
    std::vector< std::tuple <point, float> > pillars;
};


#endif /* PILLAR_DETECTOR_H */