#include <ropod_semantic_localization/localization.h>
Localization::Localization() : nh_("~"),ac("/osm_query", true), osm_query_result(), pd("/pillar_detector", true), pd_result()
{ 
    ac.waitForServer();
    pd.waitForServer();
}

Localization::~Localization()
{

}

void Localization::InitializeOdomBuffer(std::string robot)
{
  std::string topicname= "/";
  topicname.append( robot );
  topicname.append("/odom");
  odom_sub = nh_.subscribe<nav_msgs::Odometry>("/ropod/odom", 1, &Localization::BufferOdomData, this);
}

void Localization::test()
{ 
  ropod_ros_msgs::OSMQueryGoal goal;
  std::vector<int> goal_ids;
  std::vector<int> feature_ids;
  std::vector<ropod_ros_msgs::OSMNode> feature_nodes;
  std::vector<ropod_ros_msgs::OSMRelation> side_relations;
  std::vector<int> corner_ids;
  std::vector<int> side_ids;
  pillar_detector::PillarDetectorGoal pd_goal;
  ros::Rate r(1);
  while(ros::ok)
  {
    ros::spinOnce();
    // Create Goal ID
    goal_ids.push_back(building_id); 
    goal.ids 	= goal_ids;  
    
    // Search all IDs that denote a feature or side
    feature_ids = RecursiveSearch(goal,feature);
    side_ids = RecursiveSearch(goal,side);
    
    // Store all Nodes that are features
    goal.ids	= feature_ids;
    feature_nodes = Query_Nodes( goal );
    
    // Find all Corner IDs and side Relations
    goal.ids = side_ids;
    corner_ids = Query_Corners( goal ); 
    side_relations = Query_Sides( goal );
    
    // Visualize all features
    VisualizeFeatures( feature_nodes );
    // Use side relations to connect corners to visualize walls
    VisualizeSides( corner_ids , side_relations );  
    
    // Check if odometry data is available
    if( odom_buffer_.empty())
    {
      ROS_INFO("no Odometry available yet");
    }
    else
    {
      // Get current Odometry data
      odom_current = odom_buffer_.front();
      tf::Quaternion q(odom_current->pose.pose.orientation.x , odom_current->pose.pose.orientation.y,odom_current->pose.pose.orientation.z,odom_current->pose.pose.orientation.w); 
      double roll,pitch,yaw;
      tf::Matrix3x3 m(q);
      m.getRPY(roll, pitch,yaw);
      // Create a pillar-detector goal
      pd_goal = CreatePDGoal( feature_nodes , odom_current); // checks which features are pillars, determines relative position + covariance
      SendGoalPD(pd_goal);
    }
    ROS_INFO("Finished querying");   
    r.sleep();
  }
}

void Localization::BufferOdomData(const nav_msgs::Odometry::ConstPtr& odom)
{
  ROS_INFO("updating odom");
  if( odom_buffer_.empty())
  {
    odom_buffer_.push( odom );
  }
  else
  {
    while( !odom_buffer_.empty() )
    {
      odom_buffer_.pop();
    }
    odom_buffer_.push( odom );
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ropod_semantic_localization");
  std::string robot;
  ros::NodeHandle node;
  node.getParam("ropod_semantic_localization/robot", robot);
  Localization localization;
  localization.InitializeOdomBuffer( robot );
  ROS_INFO("Semantic Localization Ready!");
  localization.test();
  ros::spin();
  return 0;
}


