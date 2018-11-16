#include <pillar_detector/circleransac.h>
using namespace std;

point pol_2_cart( polpoint p)
{
  point result;
  result.x = p.r *cos(p.a);
  result.y = p.r *sin(p.a);
  return result; 
}

polpoint cart_2_pol( point p)
{
  polpoint result;
  result.r = sqrt( pow( p.x , 2 ) + pow( p.y , 2) );
  result.a = atan2(p.y,p.x); 
  return result;
}

int Expected_Amount_of_Inliers( float distance, float radius, int n_beams, float angle_increment)
{
  int result = static_cast<int>( 2*atan2(radius,distance) / angle_increment );
  return result;
}

int Nearest_Beam_Index( float angle, float angle_min, float angle_increment )
{
  int result;
  result = static_cast<int>((angle - angle_min)/ angle_increment);
  return result;
}

bool Check_In_Range( vector<float> ranges , int n_beams , float min, float max)
{
  int in_range = 0;
  for(int i =0; i < n_beams ; i++)
  {
    if( ranges[i] > min && ranges[i] < max )
    {
      in_range++;
    }
  }
  if( in_range >= 2)
  {
    return true;
  }
  else
  {
    return false;
  }
}

float Calc_Distance( point a, point b)
{
  float result;
  result = hypot( (b.x - a.x) , (b.y - a.y) );
  return result;
}

tuple<point, bool> FindCircle( vector<float> ranges, float angle_min, float angle_max, float range_min, float range_max, int n_beams, float radius, float r_threshold, int iterations)
{
  // Initialize variables
  polpoint      first_point_pol;
  polpoint      second_point_pol;
  point 	first_point;
  point 	second_point;
  point 	third_point;
  point 	circle_center;
  int 		min_point_index_distance = 1;
  float 	p2p_distance 	= 4*(radius+r_threshold);
  float 	angle_increment = ((angle_max - angle_min)/ n_beams);
  float 	f;
  int 		i 		= 0;
  int 		j		= 0;
  int 		index 		= rand() % n_beams;
  bool 		success		= true;
  
  // Select 1st point randomly from all beams, check if  range_min <  range < range_max
  while( (ranges[index] < range_min || ranges[index] > range_max )&& j < iterations )
  {
    index= rand() % n_beams;
  }
  // Remember index
  int first_point_index = index;

  // Select 2nd point randomly from all remaining beams (index2 != index1) check if 2nd point can be on 
  // the same circle as 1st point (distance < 2*R circle) 
  
  while( p2p_distance > 2*(radius+r_threshold) && i < iterations && j < iterations)
  {
    index= rand() % n_beams;
    i++;
    if(ranges[index] > range_min && ranges[index] < range_max && abs(index - first_point_index)>= min_point_index_distance )
    {
      first_point_pol.a = angle_min + first_point_index * angle_increment;
      first_point_pol.r = ranges[ first_point_index ];
      
      first_point = pol_2_cart(first_point_pol);
      
      second_point_pol.a = angle_min + index * angle_increment;
      second_point_pol.r = ranges[ index ];
      
      second_point = pol_2_cart( second_point_pol );

      p2p_distance = Calc_Distance( first_point , second_point );
    }
  }
  if( i>= iterations || j>= iterations)
  {
    success = false; 
  }
  
  // Create a third point in the middle of point 1 and point 2
  third_point.x = (first_point.x + second_point.x) /2;
  third_point.y = (first_point.y + second_point.y) /2;
  
  // Find the circle center that is pointing away from the robot 
  // Use the line perpendicular to 1st point -> 2nd point that goes through 3rd point.
  
  if( second_point.y  > first_point.y)
  {
    f = -1;
  }
  else if(first_point.y > second_point.y)
  {
    f = 1;
  }
  else
  {
    success = false;
  }
  circle_center.x = third_point.x + f*(  sqrt( pow(radius+r_threshold,2) - pow( (p2p_distance/2), 2) ) * ( (first_point.y - second_point.y) / p2p_distance ) ); 
  circle_center.y = third_point.y + f*(  sqrt( pow(radius+r_threshold,2) - pow( (p2p_distance/2), 2) ) * ( (second_point.x - first_point.x) / p2p_distance ) );
  tuple<point, bool> result( circle_center, success);
  return result;
}

bool CheckInlier( point r, point s , float radius, float r_threshold)
{
  float d;
  d = hypot( (s.x - r.x), (s.y - r.y));
  if(   sqrt( pow( d - radius, 2 ) ) < 2*r_threshold )
  {
    return true;
  }
  else
  {
    return false; 
  }
}

void CircleRansac( vector<float> ranges, float angle_min, float angle_max, float range_min, float range_max, int n_beams, float radius, float r_threshold, int ransac_iterations)
{

  bool in_range = Check_In_Range(ranges, n_beams, range_min, range_max);
  int find_circle_iterations = 100;
  int i_ransac = 0;
  int i_circle_center;
  int i_min;
  int i_max;
  float 	angle_increment = ((angle_max - angle_min)/ n_beams);
  float 	d_2_circle_center;
  float 	exp_amount; 
  float 	i_threshold = 2.0;
  vector<point> pillars;
  vector<point> possible_pillars;
  point 	randomcircle;
  tuple<point, bool> create_circle;
  polpoint 	samplepol;
  polpoint 	circlepol;
  point 	sample;
  
  while(i_ransac < ransac_iterations && in_range)
  {
    vector<float> mc_x;
    vector<float> mc_y;
    create_circle = FindCircle(ranges, angle_min, angle_max, range_min, range_max, n_beams, radius, r_threshold, find_circle_iterations);
    if( get<1>(create_circle))
    {
      int inlier = 0;  
      mc_x.clear();
      mc_y.clear();
      randomcircle = get<0>(create_circle);
      circlepol = cart_2_pol ( randomcircle );
      d_2_circle_center = circlepol.r; 
      i_circle_center = Nearest_Beam_Index( circlepol.a , angle_min, angle_increment);
      exp_amount= Expected_Amount_of_Inliers(d_2_circle_center , radius+r_threshold, n_beams, angle_increment);
      
      
      if( (i_circle_center - ( (exp_amount+2) /2)) >= 0)
      {
	i_min = i_circle_center - ( (exp_amount+2) /2);
      }
      else
      {
	i_min = 0;
      }
      if( (i_circle_center + ( (exp_amount+2) /2 ) ) <= n_beams)
      {
	i_max = i_circle_center + ( (exp_amount+2) /2);
      }
      else
      {
	i_max = n_beams;
      }   
      
      for( int j = i_circle_center - ( (exp_amount+2) /2); j< i_circle_center + ( (exp_amount+2) /2); j++)
      {
	if( j > 0 && j < n_beams )
	{
	  samplepol.r = ranges[j];
	  samplepol.a = angle_min + j * angle_increment;
	  sample = pol_2_cart( samplepol );
	  if (CheckInlier( randomcircle , sample , radius, r_threshold) )
	  {
	    mc_x.push_back(sample.x);
	    mc_y.push_back(sample.y);
	    inlier++;
	  }
	}
      }
      
      if( inlier >= exp_amount - i_threshold )
      {

	if( pillars.empty() )
	{
	  pillars.push_back( randomcircle );
	}
	else
	{
	  bool duplicate_detected = false; 
	  for( int k=0; k < pillars.size(); k++)
	  {
	    if( Calc_Distance( pillars[k] , randomcircle ) < radius+r_threshold )
	    {
	      duplicate_detected = true;
	    }
	  }
	  if( !duplicate_detected )
	  {
	    pillars.push_back( randomcircle );
	  }
	}
      }
      else
      {
	// obstructed or simply not a pillar cluster
	// check diff( expected inliers vs inliers ) 
	// check which beams are blocked and at what range
	
      }
    }   
    i_ransac++;
  } 
  for( int l=0 ; l < pillars.size() ; l++)
  {
    ROS_INFO_STREAM( l+1 ) ; 
    ROS_INFO_STREAM( pillars[l].x );
    ROS_INFO_STREAM( pillars[l].y );
  }
}