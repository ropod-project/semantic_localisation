#include "/home/martin/catkin_ws_test/src/pillar_detector/include/pillar_detector/pillar_detector.h"
//#include <pillar_detector/pillar_detector.h>
 
point robotpol_2_robotcart( polpoint p)
{
  point result;
  result.x = p.r *cos(p.a);
  result.y = p.r *sin(p.a);
  return result; 
}

polpoint robotcart_2_robotpol( point p)
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

float Calc_Distance( point a, point b)
{
  float result;
  result = hypot( (b.x - a.x) , (b.y - a.y) );
  return result;
}

point FindCircleCenter( point p1, point p2, point p3, float diameter)
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


bool CheckInlier( point circle_center, point sample , float diameter, float diameter_error)
{
  float d;
  d = Calc_Distance( circle_center, sample );
  if( sqrt( pow( (d- (diameter/2) ) , 2 ) ) < diameter_error)
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

bool InRange(float r, float min, float max)
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
  

int PillarDetector::CalcSearchRange(polpoint p, float diameter)
{
  int result;
  result = round( atan2( diameter, p.r )/ scan_result->angle_increment);
  return result;
}

std::tuple< float, float > PillarDetector::CalcRange( geometry_msgs::PoseWithCovariance initial_guess )
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

void PillarDetector::Detect( const pillar_detector::PillarDetectorGoalConstPtr goal )
{ 
  std::tuple< point, bool> fpc;
  point pc; 
  polpoint ppc;
  polpoint pps;
  point    ps;
  int min_index;
  int max_index;
  int pc_index;
  int inlier;
  int max_iter = 1000; // minimum N maximum N^3 searches, change later to different iterators for each step
  float diameter = goal->diameter[0];
  float diameter_error = 0.01;
  if(false)
  {
    //angle_min = function;
    //angle_max = function;
  }
  for(int i = 0; i< max_iter; i++ )
  {
    int index_min = Nearest_Beam_Index(scan_result->angle_min);
    int index_max = Nearest_Beam_Index(scan_result->angle_max);
    int inlier_search_range;
    fpc = FindPillarCandidate( index_min, index_max, diameter, diameter_error, max_iter ); 
    if( std::get<1>(fpc))
    {
      pc = std::get<0>(fpc);
      ppc = robotcart_2_robotpol(pc);
      pc_index = Nearest_Beam_Index(ppc.a);
      inlier_search_range = CalcSearchRange( ppc, (diameter/2) ) +3;
      min_index = pc_index - inlier_search_range;
      max_index = pc_index + inlier_search_range;
      if( min_index < 0 ){ min_index =0;}
      if( max_index > scan_result->ranges.size()){ max_index = scan_result->ranges.size();}
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
      if( (inlier > 20) &&  (inlier > 0.8*CalcSearchRange( ppc, (diameter) )) )
      {
	ROS_INFO("inlier search range %i", inlier_search_range);
	ROS_INFO("pc_index %i", pc_index);
	ROS_INFO("pc.x: %f pc.y: %f", pc.x, pc.y);
      }
    }
  }
}
