#include <ropod_semantic_localization/localization.h>

void Localization::VisualizeFeatures( std::vector< ropod_ros_msgs::OSMNode > nodes )
{
  visualization_msgs::MarkerArray markerarray;
  for( auto it_nodes = nodes.begin(); it_nodes != nodes.end(); it_nodes++)
  {
    visualization_msgs::Marker marker;
    marker.id = it_nodes->id;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = .2;
    marker.scale.y = .2;
    marker.scale.z = .2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.pose.position.z = 1;
    for( auto it_tags = it_nodes->tags.begin(); it_tags !=it_nodes->tags.end(); it_tags++)
    {
      if( it_tags->key =="indoor" && it_tags->value == "pillar" )
      {
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
      }
      if( it_tags->key =="indoor" && it_tags->value == "sign" )
      {
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
      }
      if( it_tags->key =="diameter" )
      {
	marker.scale.x = strtof(it_tags->value.c_str(),NULL);
	marker.scale.y = strtof(it_tags->value.c_str(),NULL);
	marker.scale.z = 2;
	marker.pose.position.z = 1;
      }
      if( it_tags->key =="mount_height")
      {
	char* pEnd;
	marker.pose.position.z = strtof(it_tags->value.c_str(),NULL);
      }
    }
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = it_nodes->x;
    marker.pose.position.y = it_nodes->y;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;

    markerarray.markers.push_back(marker);
  }
  vis_pub.publish(markerarray);
}

void Localization::VisualizeSides( std::vector< int > corner_ids, std::vector<ropod_ros_msgs::OSMRelation> side_relations )
{
  int id = 30000;
  int j = 0;
  std::vector< ropod_ros_msgs::OSMNode > nodes;
  ropod_ros_msgs::OSMQueryGoal goal;
  geometry_msgs::Point start_point;
  geometry_msgs::Point end_point;
  visualization_msgs::MarkerArray markerarray;
    
  goal.ids= corner_ids;
  nodes = Query_Nodes( goal );
  
  for( int i=0 , e=corner_ids.size(); i<e ; i+=2)
  {
    visualization_msgs::Marker marker;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0; 
    for( auto it_tags = side_relations[j].tags.begin() ; it_tags != side_relations[j].tags.end(); it_tags++)
    {
      if( it_tags->key =="colour")
      {
	if( it_tags->value =="yellow")
	{
	  marker.color.b = 0.0;
	}
	else if( it_tags->value =="blue")
	{
	  marker.color.r = 0.0;
	  marker.color.g = 0.0;
	  marker.color.b = 1.0;	  
	}
	else if( it_tags->value =="green")
	{
	  marker.color.r = 0.0;
	  marker.color.b = 0.0; 
	}
	else if( it_tags->value =="red")
	{
	  marker.color.g = 0.0;
	  marker.color.b = 0.0; 
	}
	else
	{
	  marker.color.a = 1.0;
	  marker.color.r = 0.4;
	  marker.color.g = 0.2;
	  marker.color.b = 0.0; 	  
	}
      }
      else if( it_tags->key =="material") // material is key of Wall not of wall-side so this doesn't work yet
      {
	if( it_tags->value =="glass")
	{
	  marker.color.a = 1.0;
	  marker.color.r = 0.2;
	  marker.color.g = 0.5;
	  marker.color.b = 1.0;
	}
      }
    }
    j++;
    id++;
    for( auto it_nodes = nodes.begin(); it_nodes != nodes.end(); it_nodes++)
    {
      if( it_nodes->id == corner_ids[i] )
      {
	start_point.x = it_nodes->x;
	start_point.y = it_nodes->y;
      }
      else if (it_nodes->id == corner_ids[i+1])
      {
	end_point.x = it_nodes->x;
	end_point.y = it_nodes->y;
      }
    }    
    marker.id = id;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    start_point.z = 0.0;
    end_point.z = 0.0;
    marker.points.push_back(start_point);
    marker.points.push_back(end_point);
    markerarray.markers.push_back(marker);
  }
  vis_pub.publish(markerarray);
}
