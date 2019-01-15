#include "/home/martin/catkin_ws_test/src/semantic_localisation/pillar_detector/include/pillar_detector/pillar_detector.h"
//#include <pillar_detector/pillar_detector.h>
 
point PillarDetector::robotpol_2_robotcart( polpoint p)
{
  point result;
  result.x = p.r *cos(p.a);
  result.y = p.r *sin(p.a);
  return result; 
}

polpoint PillarDetector::robotcart_2_robotpol( point p)
{
  polpoint result;
  result.r = sqrt( pow(p.x,2) + pow(p.y ,2) );
  result.a = atan2(p.y,p.x); 
  return result;
}

polpoint  PillarDetector::scan_2_robotpol( int i)
{
  polpoint result;
  result.r = scan_result->ranges[i];
  result.a = scan_result->angle_min + (i*scan_result->angle_increment);
  return result;
}

float PillarDetector::Calc_Distance( point a, point b)
{
  float result;
  result = hypot( (b.x - a.x) , (b.y - a.y) );
  return result;
}

point PillarDetector::FindCircleCenter( point p1, point p2, point p3, float diameter)
{
  point circle_center;
  float p2p_distance;
  float f1;
  float f2; 
  if( p2.y > p1.y)
  {
    f1=-1;
  }
  else if( p1.y> p2.y )
  {
    f1=1;
  }
  else
  {
    
  }
  p2p_distance = Calc_Distance(p1,p2);
  if( pow( (diameter/2) ,2) - pow( (p2p_distance/2), 2) < 0 )
  {
    f2 = 0;
  }
  else
  {
    f2 =  sqrt( pow( (diameter/2) ,2) - pow( (p2p_distance/2), 2) );
  }
  circle_center.x = p3.x + f1*(  f2 * ( (p1.y - p2.y) / p2p_distance ) ); 
  circle_center.y = p3.y + f1*(  f2 * ( (p2.x - p1.x) / p2p_distance ) );
  return circle_center;
}


bool PillarDetector::CheckInlier( point circle_center, point sample , float diameter, float diameter_error)
{
  float d;
  d = Calc_Distance( circle_center, sample );
  if( sqrt( pow( ( d- (diameter/2) ) , 2 ) ) < diameter_error)
  {
    return true;
  }
  else
  {
    return false; 
  }
}

int PillarDetector::Nearest_Beam_Index( float angle)
{
  int result;
  result = round((angle-scan_result->angle_min)/scan_result->angle_increment);
  return result;
}

bool PillarDetector::InRange(float r, float min, float max)
{
  if( (min < r) && (r < max))
  {
    return true;
  }
  else
  {
    return false;
  }
}

int PillarDetector::CalcSearchRange(polpoint p, float radius)
{
  int result;
  if( p.r < radius )  // prevent asin( |arg| > 1) = Nan
  {
    p.r = radius;
  }
  result = round( asin( radius/ p.r )/scan_result->angle_increment );
  return result;
}

std::tuple< float, float > PillarDetector::CalcRange( geometry_msgs::PoseStamped initial_guess )
{
  std::tuple< float, float> range;
  range = std::make_tuple(0.0, 0.0);
  return range;
}

std::tuple<point,bool> PillarDetector::FindPillarCandidate(int index_min, int index_max, float diameter, float diameter_error, int max_iter)
{
  struct timeval t1;
  gettimeofday(&t1, NULL);
  srand( t1.tv_usec * t1.tv_sec );
  std::tuple<point,bool> result;
  polpoint pp1;
  polpoint pp2;
  point p1;
  point p2;
  point p3;
  point pc;
  int p1_index;
  int p2_index;
  int min_index;
  int max_index;
  int p2_search_range;
  int search_range_error = 3;
  float scanner_range_min = scan_result->range_min;
  float scanner_range_max = scan_result->range_max;
  for(int i = 0; i< max_iter; i++)
  {
   p1_index = index_min + rand()%(index_max - index_min);
   if( InRange( scan_result->ranges[p1_index], scanner_range_min, scanner_range_max) ) // check if first point is in scanner range, otherwise resample
   {
     pp1 = scan_2_robotpol( p1_index );
     p1 = robotpol_2_robotcart( pp1 );
     p2_search_range = CalcSearchRange( pp1, diameter) + search_range_error;
     for(int i = 0; i<max_iter; i++)
     {
      min_index = p1_index - p2_search_range;
      max_index = p1_index + p2_search_range;
      if(min_index < 0){min_index = 0;}
      if(max_index > scan_result->ranges.size() ){max_index = scan_result->ranges.size();}
      p2_index = min_index + rand()%(max_index - min_index);
      pp2 = scan_2_robotpol( p2_index );
      p2 = robotpol_2_robotcart( pp2 );
      if( InRange( scan_result->ranges[p2_index], scanner_range_min, scanner_range_max) && (p2_index != p1_index) && ( Calc_Distance(p1,p2) < (diameter + diameter_error)) )
	// check if 2nd point is in scanner range, not the same as point 1, and not more than pillardiameter + error in diameter away
      {
	p3.x = (p1.x + p2.x)/2; 
	p3.y = (p1.y + p2.y)/2; // create a third fictional point between p1 and p2
	pc = FindCircleCenter( p1, p2, p3, diameter+diameter_error); // calculate the circle center based on 3 points + diameter (select outward facing one)
	result = std::make_tuple( pc, true);
	return result;
      }
     }
   }
  }
  result = std::make_tuple( pc, false) ;
  return result;
}

void PillarDetector::VisualizeDetections()
{
  visualization_msgs::MarkerArray markerarray;
  point pillar_point;
  int i = 17000;
  markerarray.markers.clear();
  int k = 0;
  for( auto it_pillars = pillars.begin(); it_pillars != pillars.end(); it_pillars++)
  {
    i++;
    pillar_point = std::get<0>(pillars[k]);
    visualization_msgs::Marker marker;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0; 
    marker.pose.position.x = pillar_point.x;
    marker.pose.position.y = pillar_point.y;
    marker.pose.position.z = 1;
    marker.pose.orientation.w = 1.0;
    marker.id = i;
    marker.header.frame_id = "ropod/laser/scan";
    marker.header.stamp = ros::Time(0);
    marker.ns = "my_namespace";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.66;  // Hardcoded the diameter, not really a way to change this yet
    marker.scale.y = 0.66;
    marker.scale.z = 2;
    markerarray.markers.push_back(marker);
    k++;
  }
  vis_pub.publish(markerarray);
}


void PillarDetector::Detect( const pillar_detector::PillarDetectorGoalConstPtr goal ) 
{ 
  pillars.clear();
  std::tuple<point, float> pillar;
  std::tuple< point, bool> fpc;
  point pc; 
  polpoint ppc;
  polpoint pps;
  point    ps;
  bool duplicate;
  bool print = false;
  bool visualize = true;
  int min_index;
  int max_index;
  int max_iter;
  int pc_index;
  int inlier;
  int inlier_min = 12; 
  int expected_inlier_amount;
  float inlier_percentage_min = 0.75;
  int buffer_range = 3;
  float diameter = goal->diameter;
  float diameter_error = 0.005;
  float inlier_percentage;
  int edge_index_low;
  int edge_index_high;
  
  int index_min = Nearest_Beam_Index(scan_result->angle_min);
  int index_max = Nearest_Beam_Index(scan_result->angle_max);
  max_iter = round( (index_max - index_min)/2 );
  for(int i = 0; i< max_iter; i++ )
  {
    feedback.iter = i;
    pillar_detector_server.publishFeedback(feedback);
    int inlier_search_range;
    fpc = FindPillarCandidate( index_min, index_max, diameter, diameter_error, max_iter ); 
    if( std::get<1>(fpc))
    {
      pc = std::get<0>(fpc);
      ppc = robotcart_2_robotpol(pc);
      pc_index = Nearest_Beam_Index(ppc.a);
      inlier_search_range = CalcSearchRange( ppc, (diameter/2) ) + buffer_range;
      min_index = pc_index - inlier_search_range;
      max_index = pc_index + inlier_search_range;
      if( min_index < 0 ){ min_index =0;}
      if( max_index > (scan_result->ranges.size())){ max_index = (scan_result->ranges.size());}
      inlier =0;
      for( int j = min_index; j<max_index; j++)
      {
	pps = scan_2_robotpol( j );
	ps = robotpol_2_robotcart(pps);
	if( CheckInlier( pc, ps, diameter, diameter_error) )
	{
	  inlier++;
	}
      }
      expected_inlier_amount = 2*CalcSearchRange( ppc , (diameter/2) );
      if( (inlier > inlier_min) &&  ( (float)inlier >  (inlier_percentage_min * (float)expected_inlier_amount) ) && expected_inlier_amount > inlier_min ) 
      {
	inlier_percentage = (float)inlier / (float)expected_inlier_amount;
	if( inlier_percentage > 1){ inlier_percentage = 1;}
	pillar = std::make_tuple( pc , inlier_percentage);
	if( !pillars.empty())
	{
	  duplicate = false;
	  int k = 0;
	  for( auto it_pillar = pillars.begin(); it_pillar < pillars.end(); it_pillar++)
	  {
	    if( Calc_Distance( std::get<0>( pillars[k] ) , pc) < diameter )
	      {
		duplicate = true;
		if( std::get<1>( pillars[k] ) < inlier_percentage)
		{
		  pillars.erase( pillars.begin() + k);
		  pillars.push_back( pillar );
		}
	      }
	    k++;
	  }
	  if( !duplicate)
	  {
	    pillars.push_back( pillar );
	  }
	}
	else
	{
	  pillars.push_back( pillar );
	}
      }
    }
  }
  if( visualize )
  {
    VisualizeDetections();
  }
  if( print)
  {
    int l=0;
    for( auto it_pillar = pillars.begin(); it_pillar < pillars.end(); it_pillar++)
    {
      point pc = std::get<0>(pillars[l]);
      float in = std::get<1>(pillars[l]);
      ROS_INFO("Pillar %i found at:", l);
      ROS_INFO("pc.x : %f pc.y: %f", pc.x, pc.y);
      ROS_INFO("inlierpercentage: %f", in);
      l++;
    }
  }
}
