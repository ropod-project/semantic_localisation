#include "/home/martin/catkin_ws_test/src/semantic_localisation/wall_detector/include/wall_detector/wall_detector.h"

std::tuple<float , float> WallDetector::FitLine( std::vector<int> indices)
{
  std::tuple<float, float> result;
  int index;
  point sample;
  float xsum=0,x2sum=0,ysum=0,xysum=0,a ,b, n;
  n = indices.size();
  for( int i =0; i<indices.size(); i++)
  {
    index = indices[i];
    sample = robotpol_2_robotcart( scan_2_robotpol( index ) );
    xsum = xsum + sample.x;
    ysum = ysum + sample.y;
    x2sum = x2sum+pow(sample.x,2);
    xysum = xysum + (sample.x * sample.y);
  }
  a=(n*xysum-xsum*ysum)/(n*x2sum-xsum*xsum);		//calculate slope
  b=(x2sum*ysum-xsum*xysum)/(x2sum*n-xsum*xsum);        //calculate intercept
  result = std::make_tuple(a,b);
  return result;
}

point WallDetector::robotpol_2_robotcart( polpoint p)
{
  point result;
  result.x = p.r *cos(p.a);
  result.y = p.r *sin(p.a);
  return result; 
}

polpoint WallDetector::robotcart_2_robotpol( point p)
{
  polpoint result;
  result.r = sqrt( pow(p.x,2) + pow(p.y ,2) );
  result.a = atan2(p.y,p.x); 
  return result;
}

polpoint  WallDetector::scan_2_robotpol( int i)
{
  polpoint result;
  result.r = scan_result->ranges[i];
  result.a = scan_result->angle_min + (i*scan_result->angle_increment);
  return result;
}

float WallDetector::Calc_Distance( point a, point b)
{
  float result;
  result = hypot( (b.x - a.x) , (b.y - a.y) );
  return result;
}

int WallDetector::Nearest_Beam_Index( float angle)
{
  int result;
  result = round((angle-scan_result->angle_min)/scan_result->angle_increment);
  return result;
}

bool WallDetector::InRange(float r, float min, float max)
{
  if( r > min && r < max)
  {
    return true;
  }
  else
  {
    return false;
  }
}

float WallDetector::Distance2Line( point ps, point p1, point p2)
{
  float result;
  float x0,x1,x2;
  float y0,y1,y2;
  float numerator;
  float denominator;
  x0 = ps.x;
  y0 = ps.y;
  x1 = p1.x;
  y1 = p1.y;
  x2 = p2.x;
  y2 = p2.y;
  numerator = (y2-y1)*x0 - (x2-x1)*y0 +x2*y1 -y2*x1;
  if( numerator < 0 )
  {
    numerator = -1*numerator;
  }
  denominator = Calc_Distance( p1,p2);
  result = numerator/denominator;
  return result;
}

bool WallDetector::CheckInlier( point ps, point p1, point p2, float threshold)
{
  float distance;
  distance = Distance2Line(ps,p1,p2);
  if( distance > threshold)
  {
    return false;
  }
  else
  {
    return true; 
  }
}

bool WallDetector::CheckDuplicate( point p1, point p2, int p1_index, int p2_index)
{
  float duplicate_threshold = 0.30;
  float angle_threshold = 2*PI_F/180;
  float distance_threshold = 0.1;
  point origin;
  float sd;
  float wd;
  float sx1, sx2, sy1, sy2;
  float wx1, wx2, wy1, wy2;
  float sdx, sdy, st;
  float wdx, wdy, wt;
  origin.x = 0.0;
  origin.y = 0.0;
  sx1 = p1.x;
  sx2 = p2.x;
  sd = Distance2Line( origin, p1, p2 );
  sdx = sx1 - sx2;
  sdy = sy1 - sy2;
  st = 0.5*PI_F - atan2(sdx,sdy);
  
  if( walls.empty() )
  {
    return false;
  }
  for( auto it_walls = walls.begin(); it_walls<walls.end(); it_walls++)
  {
    wx1 = it_walls->c1.x;
    wx2 = it_walls->c2.x;
    wd = Distance2Line( origin , it_walls->c1 , it_walls->c2 );
    wdx = wx1 - wx2;
    wdy = wy1 - wy2;
    wt  = 0.5*PI_F - atan2( wdx,wdy);    
    //if( sqrt( pow( (wt-st), 2 ) < angle_threshold && sqrt( pow( (wd-sd),2) )) < distance_threshold )
    if( CheckInlier( p1, it_walls->c1, it_walls->c2 , duplicate_threshold) && CheckInlier( p2,it_walls->c1, it_walls->c2 , duplicate_threshold))
    {
      if( (p1_index < it_walls->c1_index && p2_index < it_walls->c1_index) ||
	  (p1_index > it_walls->c2_index && p2_index > it_walls->c2_index) )
      {
      }
      else
      {
	//ROS_INFO(" angle: %f range: %f, wall angle: %f wall range: %f", st, sd, wt, wd);
	return true;
      }
    }
  }
  return false;
}

std::tuple< bool, bool> WallDetector::CheckSegmentation( point ps, point p1, point p2, float threshold)
{
  std::tuple< bool, bool> result;
  float x1,x2,x3,x4;
  float y1,y2,y3,y4;
  float angle_diff;
  float range_diff;
  float angle_threshold = (2*PI_F)/360;
  point intersect;
  polpoint intersect_pol;
  polpoint ps_pol;
  float denominator;
  x1 = p1.x;
  y1 = p1.y;
  x2 = p2.x;
  y2 = p2.y;
  x3 = 0.0;
  y3 = 0.0;
  x4 = ps.x;
  y4 = ps.y;
  denominator = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
  result = std::make_tuple( false, false);
  if( denominator == 0 )
  {
    return result;
  }
  intersect.x = ( (x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4) ) / 
		( denominator );
  intersect.y = ( (x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4) ) /
		( denominator ); 
  intersect_pol = robotcart_2_robotpol( intersect );
  ps_pol	= robotcart_2_robotpol( ps );
  if( intersect_pol.a > ps_pol.a)
  {
    angle_diff = intersect_pol.a - ps_pol.a;
  }
  else
  {
    angle_diff = ps_pol.a - intersect_pol.a;
  }
  if( angle_diff > angle_threshold )
  {
    return result;
  }
  if( ps_pol.r > intersect_pol.r) // cornerpoint
  {
    if( Distance2Line( ps , p1, p2) > threshold)
    {
      result = std::make_tuple( true, true);
      return result;
    }
  }
  else // obstruction
  {
    if( Distance2Line( ps , p1, p2) > threshold)
    {
      result = std::make_tuple( true, false);
      return result;
    }
  }
  return result;
}

std::tuple<point, point, bool> WallDetector::FindLineCandidate(int index_min, int index_max, int max_iter)
{
  int p1_index;
  int p2_index;
  point lcp1;
  point lcp2;
  std::tuple<point, point, bool> result;
  float scanner_range_min = scan_result->range_min;
  float scanner_range_max = scan_result->range_max;
  struct timeval t1;
  gettimeofday(&t1, NULL);
  srand( t1.tv_usec * t1.tv_sec );
  for( int i = 0; i<max_iter ; i++ )
  {
    p1_index = index_min + rand()%(index_max - index_min);
    if( InRange( scan_result->ranges[p1_index], scanner_range_min, scanner_range_max) )
    {
      for(int j=0; j<max_iter; j++)
      {
	p2_index = index_min + rand()%(index_max - index_min);
	if( InRange( scan_result->ranges[p2_index], scanner_range_min, scanner_range_max) && p1_index!=p2_index )
	{
	  lcp1 = robotpol_2_robotcart( scan_2_robotpol( p1_index ) );
	  lcp2 = robotpol_2_robotcart( scan_2_robotpol( p2_index ) );
	  if( !CheckDuplicate( lcp1, lcp2, p1_index, p2_index ) )
	  {
	    result = std::make_tuple( lcp1,  lcp2, true);
	    return result;
	  }
	}
      }
    }
  }
  result = std::make_tuple( lcp1,  lcp2, false);
  return result;
}
  
void WallDetector::FindInliers( point p1, point p2, float threshold, int index_min, int index_max, bool refine)
{
  point sample;
  int inlier;
  int outlier;
  int c1_index;
  int c2_index;
  wall wall;
  float min_wall_segment_length = 0.5;
  float inlier_percentage_min = 0.75;
  int inlier_min = 12;
  std::vector<int> inlier_indices;
  point corner1;
  point corner2;
  outlier = 0;
  inlier = 0;
  inlier_indices.clear();
  c1_index = scan_result->ranges.size();
  c2_index = 0;
  for(int j = index_min ; j<index_max; j++)
  {
    sample = robotpol_2_robotcart( scan_2_robotpol( j ) );
    if( InRange(scan_result->ranges[j], scan_result->range_min, scan_result->range_max) && CheckInlier( sample, p1 , p2 , threshold) )
    {
      outlier =0; // reset outlier counter
      if( j < c1_index )
      {
	c1_index = j;
      }
      if( j > c2_index)
      {
	c2_index = j;
      }
      inlier_indices.push_back(j);
      inlier++;
    }
    else
    {
      if( inlier >= 1) // if first inlier was found, but now we have an outlier
      {
	outlier++;
	if( outlier > 10) // if 10 outliers in a row this wall ends here
	{
	  corner1 = robotpol_2_robotcart( scan_2_robotpol( c1_index ) ); //calculate cornerpoints
	  corner2 = robotpol_2_robotcart( scan_2_robotpol( c2_index ) );
	  if( inlier > inlier_min && float(inlier) >  inlier_percentage_min *float((c2_index - c1_index)+1) && Calc_Distance( corner1, corner2) > min_wall_segment_length )
	  {
	    wall.indices = inlier_indices;
	    wall.c1_index = c1_index;
	    wall.c2_index = c2_index;
	    wall.c1 = corner1;
	    wall.c2 = corner2;
	    wall.inliers = inlier;
	    wall.inlierpercentage = ((float)inlier/ (float)((c2_index - c1_index)+1));
	    walls.push_back(wall);
	    c1_index = scan_result->ranges.size();
	    c2_index = 0;
	    inlier = 0;
	    inlier_indices.clear();
	    outlier = 0;
	  }
	}
      }
    }
  }
}   

void WallDetector::VisualizeDetections()
{
  visualization_msgs::MarkerArray markerarray;
  geometry_msgs::Point start_point;
  geometry_msgs::Point end_point;
  int i = 18000;
  markerarray.markers.clear();
  for( auto it_walls = walls.begin(); it_walls != walls.end(); it_walls++)
  {
    i++;
    visualization_msgs::Marker marker;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0; 
    marker.id = i;
    marker.header.frame_id = "ropod/laser/scan";
    marker.header.stamp = ros::Time(0);
    marker.ns = "my_namespace";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    start_point.x = it_walls->c1.x;
    start_point.y = it_walls->c1.y;
    start_point.z = 0.0;
    end_point.x = it_walls->c2.x;
    end_point.y = it_walls->c2.y;
    end_point.z = 0.0;
    marker.points.push_back(start_point);
    marker.points.push_back(end_point);
    markerarray.markers.push_back(marker);
  }
  vis_pub.publish(markerarray);
}

  
void WallDetector::Detect( const wall_detector::WallDetectorGoalConstPtr goal ) 
{ 
  std::tuple<point, point, bool> flc;
  bool print = false;
  bool visualize = true;
  int index_min;
  int index_max;
  int max_iter;
  point corner1;
  point corner2;
  point sample;
  index_min = 0;
  index_max = scan_result->ranges.size();
  max_iter = scan_result->ranges.size()/2;
  float threshold = 0.04;
  walls.clear();
  for( int i = 0; i< max_iter; i++)
  {
    flc = FindLineCandidate( index_min, index_max, max_iter );
    if( std::get<2>(flc) )
    {
      FindInliers( std::get<0>(flc), std::get<1>(flc), threshold, index_min, index_max, true);
    }
  }
  if( visualize )
  {
    VisualizeDetections();
  }
  if( print)
  {
    if( !walls.empty() )
    //if(false)
    {
      int i=0;
      for( auto it_walls = walls.begin(); it_walls < walls.end(); it_walls++)
      {
	i++;
	ROS_INFO("Wall c1.x: %f   c1.y: %f ", it_walls->c1.x, it_walls->c1.y);
	ROS_INFO("Wall c2.x: %f   c2.y: %f ", it_walls->c2.x, it_walls->c2.y);
	ROS_INFO("Wall length: %f", Calc_Distance( it_walls->c1, it_walls->c2));
	ROS_INFO("Inliers %i ", it_walls->inliers);
	ROS_INFO("Inlierpercentage %f ", it_walls->inlierpercentage);
      }
      ROS_INFO("amount of walls: %i", i);
    }
  }
}



