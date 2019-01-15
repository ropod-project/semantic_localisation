#include <ropod_semantic_localization/localization.h>

Localization::Localization( std::string robot) : nh_("~"),ac("/osm_query", true), osm_query_result(), 
							  pd("/pillar_detector", true), pd_result(), 
							  wd("/wall_detector",true), wd_result(),
							  localization_server(nh_,"/localization_server",
  boost::bind(&Localization::Localization_server_execute, this, _1),false)
{ 
  odomtopic.append("/");
  odomtopic.append( robot );
  odomtopic.append("/odom");
  baselinktf.append("/");
  baselinktf.append( robot );
  baselinktf.append("/base_link");
  laserscantf.append("/");
  laserscantf.append( robot );
  laserscantf.append("/laser/scan");
  InitializeOdomBuffer();
  InitializeMarkerBuffer();
  ROS_INFO("Waiting for OSM Query server");
  ac.waitForServer(); // TODO this will wait for infinite time
  ROS_INFO("Waiting for Pillardetector");
  pd.waitForServer(); // script won't work without these servers starting
  ROS_INFO("Waiting for Walldetector");
  wd.waitForServer(); // maybe add timer + error message idem in querying script
  ROS_INFO("Starting Localization server");
  localization_server.start();
}

Localization::~Localization()
{

}

void Localization::Localization_server_execute(const ropod_semantic_localization::LocalizationGoalConstPtr &goal)
{
  int plan_step = 0;
  int current_area_id;
  int next_area_id;
  int current_motion_id;
  bool transition = false;
  bool aborted = false;
  localarea current_area;
  localarea next_area;
  localmotion current_motion;
  pillar_detector::PillarDetectorGoal pd_goal;
  wall_detector::WallDetectorGoal wd_goal;
  pd_goal.diameter = 0.65; // TODO: find diameters from map 
  result.result = 1;

  QueryAll( building_id ); 			// Big Query 
  QueryLocalAreas( goal->localization_ids ); 	// uses result of big query to fill local areas
  QueryLocalMotion( goal->motion_ids); 		// uses result of big query to fill local motion
  current_area_id = goal->localization_ids[ plan_step ];
  next_area_id = goal->localization_ids[ plan_step+1 ];
  current_motion_id = goal->motion_ids[ plan_step ];
  ros::Rate r(10);
  while(ros::ok)
  {
    VisualizeFeatures(feature_nodes);
    VisualizeSides(side_relations, corner_nodes);
    VisualizeMarkerDetections();
    SendGoalPD( pd_goal );
    SendGoalWD( wd_goal );
    current_area = SelectLocalArea( current_area_id );
    //VisualizeFeatures( next_area.feature_nodes );
    //VisualizeSides( next_area.sides, next_area.corners);
    next_area = SelectLocalArea( next_area_id );
    current_motion = SelectLocalMotion( current_motion_id ); 
    LocalPoseTracking( current_area , next_area, current_motion );
    DetectTransition( next_area );
    if( transition && plan_step <= goal->motion_ids.size() )
    {
      plan_step++;
      current_area_id = goal->localization_ids[ plan_step ];
      next_area_id = goal->localization_ids[ plan_step+1 ];
      current_motion_id = goal->motion_ids[ plan_step ];  
    }
    if( plan_step == goal->motion_ids.size() )
    {
      break;
    }
    if( aborted )
    {
      localization_server.setAborted(result);
    }
    r.sleep();
  }
  ROS_INFO("Succeeded in executing current plan");
  localization_server.setSucceeded(result);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ropod_semantic_localization");
  std::string robot;
  ros::NodeHandle node;
  node.getParam("ropod_semantic_localization/robot", robot);
  Localization localization(robot);
  ROS_INFO("Semantic Localization Ready!");
  ros::spin();
  return 0;
}


