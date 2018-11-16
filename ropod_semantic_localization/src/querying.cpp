#include <ropod_semantic_localization/localization.h>


void Localization::OSMQueryResultCb(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::OSMQueryResultConstPtr& result)
{
  osm_query_result = *result;
}

void Localization::PillarDetectorResultCb(const actionlib::SimpleClientGoalState& state,const pillar_detector::PillarDetectorResultConstPtr& result)
{
  pd_result = *result;
}


bool Localization::SendGoalOSM(ropod_ros_msgs::OSMQueryGoal goal)
{
  ac.waitForServer(); //will wait for infinite time
  ac.sendGoal(goal,boost::bind(&Localization::OSMQueryResultCb, this, _1, _2));
  bool finished_before_timeout = ac.waitForResult(ros::Duration(5.0));  
   if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
	return true;
      }
      else
      {
	return false;
      }
    }
}

bool Localization::SendGoalPD(pillar_detector::PillarDetectorGoal goal)
{
  pd.waitForServer(); //will wait for infinite time
  pd.sendGoal(goal,boost::bind(&Localization::PillarDetectorResultCb, this, _1, _2));
  bool finished_before_timeout = pd.waitForResult(ros::Duration(5.0));  
   if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = pd.getState();
      if (pd.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
	return true;
      }
      else
      {
	return false;
      }
    }
}

pillar_detector::PillarDetectorGoal Localization::CreatePDGoal( std::vector<ropod_ros_msgs::OSMNode> feature_nodes, nav_msgs::Odometry::ConstPtr& odom )
{
  std::vector<ropod_ros_msgs::OSMNode> pillars;
  pillar_detector::PillarDetectorGoal pd_goal;
  geometry_msgs::PoseWithCovariance pillar_pose;
  float diameter = 0.5;
  int i =0;
  pd_goal.detect = "target";
  for(auto it_nodes = feature_nodes.begin(); it_nodes != feature_nodes.end(); it_nodes++)
  {
    i++;
    for( auto it_tags = it_nodes->tags.begin(); it_tags !=it_nodes->tags.end(); it_tags++)
    {
      if(it_tags->key =="indoor" && it_tags->value == "pillar" )
      {
	pillars.push_back( feature_nodes[i]);
      }
    }
  }
  for(auto it_pillars = pillars.begin(); it_pillars != pillars.end(); it_pillars++)
  {   
    // CHANGE TO POSE + COVARIANE in laser frame using tf 
    pillar_pose.pose.position.x  = it_pillars->x;
    pillar_pose.pose.position.y  = it_pillars->y;
    pd_goal.pose.push_back( pillar_pose );
    for( auto it_tags = it_pillars->tags.begin(); it_tags !=it_pillars->tags.end(); it_tags++)
    {
      if(it_tags->key =="diameter")
      {      
	diameter = strtof(it_tags->value.c_str(),NULL);
      }
    }
    pd_goal.diameter.push_back(diameter);
  }
  return pd_goal;
}

//Recursively searches all relations and subrelations for relations with role d
std::vector<int> Localization::RecursiveSearch(ropod_ros_msgs::OSMQueryGoal x, hierarchy d)
{
  x.data_type="relation";
  x.query_type="info"; 
  if( SendGoalOSM( x ) )
  {    
    std::string search_role;
    std::vector<int> new_goal_ids;
    std::vector<int> ids;
    std::vector<int> sub_ids;
    int depth;
    switch(d)
    {
      case level:	search_role="level", 	depth=0; 	break;
      case room:	search_role="room" , 	depth=1; 	break;
      case corridor:	search_role="corridor", depth=1; 	break;
      case side:	search_role="side", 	depth=2; 	break;
      case feature:	search_role="feature", 	depth=3; 	break;
      default:		ROS_INFO(" Not a searchable role");	break;
    }
    for (auto it_relation = osm_query_result.relations.begin(); it_relation != osm_query_result.relations.end(); it_relation++)
    {
      for(auto it_members = it_relation->members.begin(); it_members !=it_relation->members.end(); it_members++)
      {
	if(it_members->role == search_role)
	{
	  ids.push_back(it_members->id);
	}
	else if(  (it_members->role == "level" 		&& depth > 0)||
		  (it_members->role == "room"  		&& depth > 1)||
		  (it_members->role == "corridor"	&& depth > 1)||
		  (it_members->role == "side"		&& depth > 2))
	{
	  new_goal_ids.push_back( it_members->id);
	}
	else
	{
	}
      }
    }
    if( !new_goal_ids.empty() )
    {
      ropod_ros_msgs::OSMQueryGoal goal;
      goal.ids = new_goal_ids;
      sub_ids = RecursiveSearch( goal, d);
    }
    ids.insert( ids.end(), sub_ids.begin(), sub_ids.end() );
    return ids;
  }
}

std::vector< ropod_ros_msgs::OSMNode > Localization::Query_Nodes(ropod_ros_msgs::OSMQueryGoal x)
{
  std::vector< ropod_ros_msgs::OSMNode > result;
  x.data_type="node";
  x.query_type="info"; 
  if(SendGoalOSM(x))
  {
    result = osm_query_result.nodes;
  }
  return result;
}

std::vector< ropod_ros_msgs::OSMRelation > Localization::Query_Sides(ropod_ros_msgs::OSMQueryGoal x)
{
  std::vector< ropod_ros_msgs::OSMRelation > result;
  x.data_type="relation";
  x.query_type="info"; 
  if(SendGoalOSM(x))
  {
    result = osm_query_result.relations;
  }
  return result;
}

std::vector< int >Localization::Query_Corners(ropod_ros_msgs::OSMQueryGoal x )
{
  ropod_ros_msgs::OSMQueryGoal goal;
  std::vector< int > result;
  int corner_check;
  x.data_type="relation";
  x.query_type="info"; 
  if( SendGoalOSM( x ) )
  {
    for (auto it_relation = osm_query_result.relations.begin(); it_relation != osm_query_result.relations.end(); it_relation++)
    {
      corner_check = 0;
      for(auto it_members = it_relation->members.begin(); it_members !=it_relation->members.end(); it_members++)
      {
	if(it_members->role == "corner")
	{
	  corner_check++;
	  result.push_back( it_members->id);
	}
      }
      if( corner_check != 2 )
      {
	ROS_INFO("Side does not have 2 corners, error in JOSM Model, Returning Empty result");
	result.clear();
	return result;
      }     
    }
  }
  return result;
}