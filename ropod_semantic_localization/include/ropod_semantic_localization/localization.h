#ifndef LOCALIZATION_H
#define LOCALIZATION_H

// C++
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <queue>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ropod_semantic_localization/LocalizationAction.h>

// SEMANTIC LOCALIZATION
#include <pillar_detector/PillarDetectorAction.h>
#include <wall_detector/WallDetectorAction.h>
#include <aruco_detection/Aruco.h>

// ROPOD
#include <ropod_ros_msgs/OSMNode.h>
#include <ropod_ros_msgs/OSMWay.h>
#include <ropod_ros_msgs/OSMRelation.h>
#include <ropod_ros_msgs/OSMTag.h>
#include <ropod_ros_msgs/OSMMember.h>
#include <ropod_ros_msgs/Waypoint.h>
#include <ropod_ros_msgs/Position.h>
#include <ropod_ros_msgs/Shape.h>

#include <ropod_ros_msgs/OSMQueryAction.h> 
#include <ropod_ros_msgs/GetWayptPositionAction.h> 
#include <ropod_ros_msgs/GetWayptShapeAction.h> 

struct junc{
  int id;
  std::vector<ropod_ros_msgs::OSMNode> feature_nodes;
  std::vector<ropod_ros_msgs::OSMRelation> sides;
  std::vector<ropod_ros_msgs::OSMNode> corners;    
};

struct room_struct : junc {};

struct localarea{
  int id;
  ropod_ros_msgs::OSMNode topology;
  std::vector<ropod_ros_msgs::OSMNode> feature_nodes;
  std::vector<ropod_ros_msgs::OSMRelation> sides;
  std::vector<ropod_ros_msgs::OSMNode> corners;
};

struct localmotion{
  int id;
  std::string motion_type;
  ropod_ros_msgs::OSMRelation wall;
  std::vector<ropod_ros_msgs::OSMNode> corners;
  ropod_ros_msgs::OSMNode feature;
  junc j;
};

class Localization
{
private:
    ros::NodeHandle nh_;   
    // SUBSCRIBERS
    ros::Subscriber odom_sub;
    ros::Subscriber marker_sub;
    // PUBLISHERS
    //ros::Publisher localization_pub;
    ros::Publisher vis_pub = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 1, true );
    ros::Publisher det_pub = nh_.advertise<visualization_msgs::MarkerArray>( "detected_markers", 1, true );
    
    void UpdateOdom2MapCorrection();
    // LISTENERS
    tf::TransformListener listener;
    // SERVERS
    actionlib::SimpleActionServer<ropod_semantic_localization::LocalizationAction> localization_server;
    // CLIENTS
    actionlib::SimpleActionClient<ropod_ros_msgs::OSMQueryAction> ac;
    actionlib::SimpleActionClient<pillar_detector::PillarDetectorAction> pd;
    actionlib::SimpleActionClient<wall_detector::WallDetectorAction> wd;
    // CALLBACKS   
    void OSMQueryResultCb(const actionlib::SimpleClientGoalState& state,const ropod_ros_msgs::OSMQueryResultConstPtr& result);
    void PillarDetectorResultCb(const actionlib::SimpleClientGoalState& state,const pillar_detector::PillarDetectorResultConstPtr& result);
    void WallDetectorResultCb(const actionlib::SimpleClientGoalState& state,const wall_detector::WallDetectorResultConstPtr& result);
    // EXECUTES
    void Localization_server_execute(const ropod_semantic_localization::LocalizationGoalConstPtr &goal);
    // BUFFERS, CURRENT & RESULTS
    // functions
    void BufferOdomData( const nav_msgs::Odometry::ConstPtr& odom ); 
    void BufferMarkers(const aruco_detection::Aruco::ConstPtr& markers); 
    // variables
    std::queue<nav_msgs::Odometry::ConstPtr> odom_buffer_;
    std::queue<aruco_detection::Aruco::ConstPtr> marker_buffer_;
    nav_msgs::Odometry::ConstPtr odom_current;
    aruco_detection::Aruco::ConstPtr markers_current;
    ropod_ros_msgs::OSMQueryResult osm_query_result;
    pillar_detector::PillarDetectorResult pd_result;
    wall_detector::WallDetectorResult wd_result;
    ropod_semantic_localization::LocalizationResult result;
    // PRIVATE 
    enum hierarchy{level, room, corridor, junction, side, feature};
    int building_id = 363;
    int level_id    = 362;
    int corridor_id = 336;
    int room_id     = 306;
    
public: 
    // CONSTRUCTOR & DESTRUCTOR
    Localization(std::string robot);
    virtual ~Localization();
    // INITIALIZING
    void InitializeOdomBuffer();
    void InitializeMarkerBuffer();
    //VISUALIZATION
    void VisualizeMarkerDetections();
    void VisualizeFeatures( std::vector<ropod_ros_msgs::OSMNode> features );
    void VisualizeSides( std::vector<ropod_ros_msgs::OSMRelation> sides, std::vector<ropod_ros_msgs::OSMNode> corners );  
    //SEND GOALS
    bool SendGoalOSM(ropod_ros_msgs::OSMQueryGoal goal);
    bool SendGoalPD(pillar_detector::PillarDetectorGoal goal);
    bool SendGoalWD(wall_detector::WallDetectorGoal goal);
    // QUERYING
    void QueryAll( int id );
    std::vector<int> RecursiveSearch(ropod_ros_msgs::OSMQueryGoal x, hierarchy d);
    void QueryLocalAreas( std::vector<int> ids );
    void QueryLocalMotion( std::vector<int> ids );
    std::vector<ropod_ros_msgs::OSMNode> Query_Nodes(ropod_ros_msgs::OSMQueryGoal x);
    std::vector<ropod_ros_msgs::OSMRelation> Query_Relations(ropod_ros_msgs::OSMQueryGoal x);
    std::vector< int > Query_Corners(ropod_ros_msgs::OSMQueryGoal x); 
    std::vector< junc > Query_Junctions(ropod_ros_msgs::OSMQueryGoal x); 
    std::vector< room_struct > Query_Rooms(ropod_ros_msgs::OSMQueryGoal x);
    localarea SelectLocalArea( int id );
    localmotion SelectLocalMotion( int id );
    ropod_ros_msgs::OSMNode SelectNode( int id );
    junc SelectJunction( int id );
    // PUBLIC VARIABLES
    std::vector<localarea> localareas;
    std::vector<localmotion> localmotions;
    nav_msgs::Odometry odom2map_correction;
    std::vector<ropod_ros_msgs::OSMNode> feature_nodes;
    std::vector<ropod_ros_msgs::OSMNode> corner_nodes;
    std::vector<ropod_ros_msgs::OSMRelation> side_relations;
    std::vector<junc> junctions;
    std::vector<room_struct> rooms;
    std::vector< int > corner_ids;
    std::string odomtopic ="";
    std::string baselinktf ="";
    std::string laserscantf ="";
    // FUNCTIONS
    void Fix_Corner_Nodes();
    void LocalPoseTracking( localarea current_area, localarea next_area, localmotion lm );
    void DetectTransition( localarea next_area );
    // OLD STUFF
    pillar_detector::PillarDetectorGoal CreatePDGoal( std::vector<ropod_ros_msgs::OSMNode> feature_nodes);
    void test();
};
    
#endif /* LOCALIZATION_H */