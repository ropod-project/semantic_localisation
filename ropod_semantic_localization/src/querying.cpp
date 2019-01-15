#include <ropod_semantic_localization/localization.h>


void Localization::OSMQueryResultCb(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::OSMQueryResultConstPtr& result)
{
  osm_query_result = *result;
}

void Localization::PillarDetectorResultCb(const actionlib::SimpleClientGoalState& state,const pillar_detector::PillarDetectorResultConstPtr& result)
{
  pd_result = *result;
}

void Localization::WallDetectorResultCb(const actionlib::SimpleClientGoalState& state,const wall_detector::WallDetectorResultConstPtr& result)
{
  wd_result = *result;
}

ropod_ros_msgs::OSMNode Localization::SelectNode( int id )
{
  ropod_ros_msgs::OSMNode dummy;
  int i = 0;
  for( auto it_node = feature_nodes.begin(); it_node != feature_nodes.end(); it_node++)
  {
    if( it_node->id == id)
    {
      return feature_nodes[i];
    }
    i++;
  }
  ROS_INFO("A Node was requested that doesn't exist or hasn't been queried yet");
  return dummy;  
}

junc Localization::SelectJunction( int id )
{
  junc dummy;
  int i = 0;
  for( auto it_junc = junctions.begin(); it_junc != junctions.end(); it_junc++)
  {
    if( it_junc->id == id)
    {
      return junctions[i];
    }
    i++;
  }
  ROS_INFO("A Junction was requested that doesn't exist or hasn't been queried yet");
  return dummy;
}

localarea Localization::SelectLocalArea( int id )
{
  localarea dummy;
  int i = 0;
  for( auto it_la = localareas.begin(); it_la != localareas.end(); it_la++)
  {
    if( it_la->id == id)
    {
      return localareas[i];
    }
    i++;
  }
  ROS_INFO("A Local area was requested that doesn't exist or hasn't been queried yet");
  return dummy;
}

localmotion Localization::SelectLocalMotion( int id )
{
  localmotion dummy;
  int i = 0;
  for( auto it_lm = localmotions.begin(); it_lm != localmotions.end(); it_lm++)
  {
    if( it_lm->id == id)
    {
      return localmotions[i];
    }
    i++;
  }
  ROS_INFO("A Local motion was requested that doesn't exist or hasn't been queried yet");
  return dummy;
}

void Localization::QueryLocalMotion( std::vector<int> ids)
{
  localmotion lm;
  localmotions.clear();
  ropod_ros_msgs::OSMQueryGoal x;
  x.ids = ids;
  Query_Relations(x);
  for( auto it_relation = osm_query_result.relations.begin(); it_relation != osm_query_result.relations.end(); it_relation++)
  {
    lm.corners.clear();
    lm.id = it_relation->id;
    for( auto it_tag = it_relation->tags.begin(); it_tag != it_relation->tags.end(); it_tag++)
    {
      if( it_tag->key == "motion")
      {
	std::string value(it_tag->value);
	lm.motion_type= value;
      }
    }
    for( auto it_member = it_relation->members.begin(); it_member != it_relation->members.end(); it_member++)
    {
      if( it_member->role == "motion-feature")
      {
	if( it_member->type == "node")
	{  
	  if( lm.motion_type == "way-point")
	  {
	  }
	  else
	  {
	    for( auto it_node = feature_nodes.begin(); it_node != feature_nodes.end(); it_node++)
	    {
	      int i =0;
	      if( it_member->id == it_node->id)
	      {
		lm.feature = feature_nodes[i];
		break;
	      }
	      i++;
	    }
	  }
	}
	if( it_member->type =="relation")
	{
	  if(lm.motion_type == "junction")
	  {
	    int i =0;
	    for(auto it_junction = junctions.begin(); it_junction != junctions.end(); it_junction++)
	    {
	      if( it_member->id == it_junction->id)
	      {
		lm.j = junctions[i];
		break;
	      }
	      i++;
	    }
	  }
	  else
	  {
	    int i = 0;
	    for( auto it_relation = side_relations.begin(); it_relation != side_relations.end(); it_relation++)
	    {
	      if( it_member->id == it_relation->id)
	      {
		lm.wall = side_relations[i];
		lm.corners.push_back( corner_nodes[2*i]);
		lm.corners.push_back( corner_nodes[(2*i)+1]);
	      }
	      i++;
	    }
	  }
	}
      }
    }
    localmotions.push_back(lm);
  }
}

void Localization::QueryLocalAreas( std::vector<int> ids )
{
  localareas.clear();
  int corner1_id;
  int corner2_id;
  std::vector<ropod_ros_msgs::OSMNode> topology_nodes;
  std::vector<int> topology_ids;
  localarea la;
  ropod_ros_msgs::OSMQueryGoal x;
  x.data_type="relation";
  x.query_type="info";
  x.ids = ids;
  if( SendGoalOSM( x) )
  {
    for( auto it_relation = osm_query_result.relations.begin(); it_relation != osm_query_result.relations.end(); it_relation++)
    {
      la.corners.clear();
      la.feature_nodes.clear();
      la.sides.clear();
      la.id = it_relation->id;
      for ( auto it_member = it_relation->members.begin(); it_member != it_relation->members.end(); it_member++)
      {
	if( it_member->role == "topology" )
	{
	  topology_ids.push_back(it_member->id);
	}
	else if ( it_member->role =="local-feature" )
	{
	  if( it_member->type == "node")
	  {
	   la.feature_nodes.push_back( SelectNode( it_member->id ) );
	  }
	  if( it_member->type == "relation" )
	  {
	    int i = 0;
	    for( auto it_junction = junctions.begin(); it_junction != junctions.end(); it_junction++)
	    {
	      if( it_member->id == it_junction->id)
	      {
		la.sides.insert(la.sides.end() , junctions[i].sides.begin(), junctions[i].sides.end());
		la.corners.insert(la.corners.end() , junctions[i].corners.begin(), junctions[i].corners.end());
		la.feature_nodes.insert(la.feature_nodes.end() , junctions[i].feature_nodes.begin(), junctions[i].feature_nodes.end());
		break;
	      }
	      i++;
	    }
	    i = 0;
	    for( auto it_room = rooms.begin(); it_room != rooms.end(); it_room++)
	    {
	      if( it_member->id == it_room->id)
	      {
		la.sides.insert(la.sides.end() , rooms[i].sides.begin(), rooms[i].sides.end());
		la.corners.insert(la.corners.end() , rooms[i].corners.begin(), rooms[i].corners.end());
		la.feature_nodes.insert(la.feature_nodes.end() , rooms[i].feature_nodes.begin(), rooms[i].feature_nodes.end());
		break;
	      }
	      i++;
	    }
	    i=0;
	    for( auto it_side = side_relations.begin(); it_side != side_relations.end(); it_side++)
	    {
	      if( it_member->id == it_side->id)
	      {
		la.sides.push_back( side_relations[i] ); 
		la.corners.push_back( corner_nodes[ (2*i) ] );
		la.corners.push_back( corner_nodes[ (2*i) ] );
		break;
	      }
	      i++;
	    }
	  }
	}
      }
      localareas.push_back(la);
    }
  }
  // only works if all local areas have a topology node
  x.ids = topology_ids;
  topology_nodes = Query_Nodes(x);
  int i = 0;
  for( auto it_la = localareas.begin(); it_la != localareas.end(); it_la++)
  {
    int j = 0;
    for( auto it_top = topology_nodes.begin(); it_top != topology_nodes.end(); it_top++)
    {
      if( topology_ids[i] == it_top->id )
      {
	localareas[i].topology = topology_nodes[j];
	break;
      }
      j++;
    }
    i++;
  }
}

void Localization::QueryAll( int id )
{
  ropod_ros_msgs::OSMQueryGoal goal;
  std::vector<int> goal_ids;
  std::vector<int> junction_ids;
  std::vector<int> feature_ids;
  std::vector<int> side_ids;
  std::vector<int> room_ids;
  goal_ids.push_back( id ); 
  goal.ids 	= goal_ids;  
    
  // Search all IDs that denote a feature, side or junction
  feature_ids = RecursiveSearch(goal,feature);
  side_ids = RecursiveSearch(goal,side);
  junction_ids = RecursiveSearch(goal, junction); 
  room_ids = RecursiveSearch(goal, room);
  
  // Store all Nodes that are features 
  goal.ids	= feature_ids;
  feature_nodes = Query_Nodes( goal );
    
  // Find all Corner IDs and side Relations
  goal.ids = side_ids;
  corner_ids = Query_Corners( goal ); 
  side_relations = Query_Relations( goal );  
  goal.ids = corner_ids;
  corner_nodes = Query_Nodes( goal );
  Fix_Corner_Nodes();
  goal.ids = junction_ids;
  junctions = Query_Junctions( goal );
  goal.ids = room_ids;
  rooms = Query_Rooms( goal);
}

void Localization::Fix_Corner_Nodes()
{
  std::vector<ropod_ros_msgs::OSMNode> new_corner_nodes;
  for( int i=0 ; i<corner_ids.size() ; i++)
  {
    int j=0;
    for( auto it_node = corner_nodes.begin() ; it_node != corner_nodes.end(); it_node++)
    {
      if( corner_ids[i] == it_node->id)
      {
	new_corner_nodes.push_back(corner_nodes[j]);
	break;
      }
      j++;
    }
  }
  corner_nodes = new_corner_nodes;
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

bool Localization::SendGoalWD(wall_detector::WallDetectorGoal goal)
{
  wd.waitForServer(); //will wait for infinite time
  wd.sendGoal(goal,boost::bind(&Localization::WallDetectorResultCb, this, _1, _2));
  bool finished_before_timeout = wd.waitForResult(ros::Duration(5.0));  
   if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = wd.getState();
      if (wd.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
	return true;
      }
      else
      {
	return false;
      }
    }
}

pillar_detector::PillarDetectorGoal Localization::CreatePDGoal( std::vector<ropod_ros_msgs::OSMNode> feature_nodes)
{
  tf::StampedTransform transform;
  tf::Quaternion q;
  std::vector<ropod_ros_msgs::OSMNode> pillars;
  pillar_detector::PillarDetectorGoal pd_goal;
  geometry_msgs::PoseStamped pillar_pose;
  float diameter = 0.5;
  float distance;
  float angle;
  float x;
  float y;
  int i =0;
  //pd_goal.detect = "target";
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
    try
    {
      //listener.lookupTransform("map", baselinktopic, ros::Time(0), transform);
      listener.lookupTransform("map", laserscantf, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what() );
      ros::Duration(0.2).sleep();
      continue;
    } 
    q = transform.getRotation();
    double roll,pitch,yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch,yaw);
    x = it_pillars->x - transform.getOrigin().x();
    y = it_pillars->y - transform.getOrigin().y();  
    distance = hypot( x , y);
    angle = atan2(y,x) - yaw;
    pillar_pose.pose.position.x    = distance * cos(angle);
    pillar_pose.pose.position.y    = distance * sin(angle);
    //pd_goal.pose.push_back( pillar_pose );
    for( auto it_tags = it_pillars->tags.begin(); it_tags !=it_pillars->tags.end(); it_tags++)
    {
      if(it_tags->key =="diameter")
      {      
	diameter = strtof(it_tags->value.c_str(),NULL);
      }
    }
    pd_goal.diameter = diameter;
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
      case junction:	search_role="junction", depth=2;	break;
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
		  (it_members->role == "junction"	&& depth > 2)||
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

std::vector< ropod_ros_msgs::OSMRelation > Localization::Query_Relations(ropod_ros_msgs::OSMQueryGoal x)
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
  int i = 0;
  int j =0;
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
	  j++;
	}
      }
      if( corner_check != 2 )
      {
	ROS_INFO("Side does not have 2 corners, error in JOSM Model, Returning Empty result");
	result.clear();
	return result;
      }
      i++;
    }
  }
  return result;
}

std::vector< junc > Localization::Query_Junctions(ropod_ros_msgs::OSMQueryGoal x)
{ 
  std::vector<junc> result;
  junc junct;
  std::vector< ropod_ros_msgs::OSMRelation> junction_relations;
  junction_relations = Query_Relations( x );
  for( auto it_junction = junction_relations.begin(); it_junction != junction_relations.end(); it_junction++)
  {
    junct.id = it_junction->id;
    for( auto it_member = it_junction->members.begin(); it_member != it_junction->members.end(); it_member++)
    {
      if( it_member-> type == "node" )
      {
	int i = 0;
	for( auto it_node = feature_nodes.begin(); it_node != feature_nodes.end(); it_node++)
	{
	  if( it_member->id == it_node->id )
	  {
	    junct.feature_nodes.push_back( feature_nodes[i] );
	    break;
	  }
	  i++;
	}
      }
      else if( it_member-> type == "relation" )
      {
	int j = 0;
	for( auto it_side = side_relations.begin(); it_side != side_relations.end(); it_side++)
	{
	  if( it_member->id == it_side->id )
	  {
	    junct.sides.push_back( side_relations[j] );
	    junct.corners.push_back( corner_nodes[ (2*j) ] );
	    junct.corners.push_back( corner_nodes[ (2*j) +1] );
	    break;
	  }
	  j++;
	}
      }
    } 
    result.push_back(junct);
  }
  return result;
}

std::vector< room_struct > Localization::Query_Rooms(ropod_ros_msgs::OSMQueryGoal x) // same function as query junctions TODO: inheritance
{ 
  std::vector<room_struct> result;
  room_struct temp_room;
  std::vector< ropod_ros_msgs::OSMRelation> room_relations;
  room_relations = Query_Relations( x );
  for( auto it_room = room_relations.begin(); it_room != room_relations.end(); it_room++)
  {
    temp_room.id = it_room->id;
    for( auto it_member = it_room->members.begin(); it_member != it_room->members.end(); it_member++)
    {
      if( it_member-> type == "node" )
      {
	int i = 0;
	for( auto it_node = feature_nodes.begin(); it_node != feature_nodes.end(); it_node++)
	{
	  if( it_member->id == it_node->id )
	  {
	    temp_room.feature_nodes.push_back( feature_nodes[i] );
	    break;
	  }
	  i++;
	}
      }
      else if( it_member-> type == "relation" )
      {
	int j = 0;
	for( auto it_side = side_relations.begin(); it_side != side_relations.end(); it_side++)
	{
	  if( it_member->id == it_side->id )
	  {
	    temp_room.sides.push_back( side_relations[j] );
	    temp_room.corners.push_back( corner_nodes[ (2*j) ] );
	    temp_room.corners.push_back( corner_nodes[ (2*j) +1] );
	    break;
	  }
	  j++;
	}
      }
    } 
    result.push_back(temp_room);
  }
  return result;
}
