#ifndef WALL_DETECTOR_H
#define WALL_DETECTOR_H

// 
#include <wall_detector/WallDetectorAction.h>

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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

struct wall{
  std::vector<int> indices;
  int c1_index;
  int c2_index;
  point c1;
  point c2;
  int 	inliers;
  float inlierpercentage;
};

class WallDetector
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<wall_detector::WallDetectorAction> wall_detector_server;
    void wall_detector_server_execute( const wall_detector::WallDetectorGoalConstPtr &goal);
    void displayLaserdata(const sensor_msgs::LaserScan::ConstPtr& scan);
    wall_detector::WallDetectorResult result;
    ros::Subscriber scan_sub;
    ros::Publisher vis_pub = nh_.advertise<visualization_msgs::MarkerArray>( "detected_walls", 0 );
    std::queue<sensor_msgs::LaserScan::ConstPtr> scan_buffer_;

public:
    sensor_msgs::LaserScan::ConstPtr scan_result;
    WallDetector();
    virtual ~WallDetector();
    void Detect( const wall_detector::WallDetectorGoalConstPtr goal );
    void InitializeBuffer( std::string robot);
    void VisualizeDetections();
    point robotpol_2_robotcart( polpoint p);
    polpoint scan_2_robotpol( int i);
    polpoint robotcart_2_robotpol( point p);
    std::tuple<point, point, bool> FindLineCandidate(int index_min, int index_max, int max_iter);
    float Calc_Distance( point a, point b); 
    float Distance2Line( point ps, point p1, point p2);
    int Nearest_Beam_Index( float angle);
    bool InRange(float r, float min, float max); 
    bool CheckInlier( point s, point p1, point p2, float threshold);
    bool CheckDuplicate( point p1, point p2, int p1_index, int p2_index);
    std::tuple<bool,bool> CheckSegmentation( point ps, point p1, point p2, float threshold);
    std::tuple<float , float> FitLine( std::vector<int> indices);
    void FindInliers( point p1, point p2, float threshold, int index_min, int index_max, bool refine);
    std::vector<wall> walls;
};


#endif /* WALL_DETECTOR_H */