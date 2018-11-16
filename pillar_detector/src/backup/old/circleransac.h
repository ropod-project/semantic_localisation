#include "ros/ros.h"
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include <vector>
#include <tuple>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>

#ifndef CIRCLERANSAC_H_INCLUDED
#define CIRCLERANSAC_H_INCLUDED
  void CircleRansac( std::vector<float> ranges, float angle_min, float angle_max, float range_min, 
		     float range_max, int n_beams, float radius = 0.5,float r_threshold = 1e-3, int ransac_iterations =1e3);

  
struct point{
  float x;
  float y;
};

struct polpoint{
  float r;
  float a;
};  
  
  
#endif
  