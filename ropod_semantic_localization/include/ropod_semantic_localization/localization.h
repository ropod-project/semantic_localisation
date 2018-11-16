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

// SEMANTIC LOCALIZATION
#include <pillar_detector/PillarDetectorAction.h>

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


class Localization
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub;
    actionlib::SimpleActionClient<ropod_ros_msgs::OSMQueryAction> ac;
    actionlib::SimpleActionClient<pillar_detector::PillarDetectorAction> pd;
    ropod_ros_msgs::OSMQueryResult osm_query_result;
    pillar_detector::PillarDetectorResult pd_result;
    void OSMQueryResultCb(const actionlib::SimpleClientGoalState& state,const ropod_ros_msgs::OSMQueryResultConstPtr& result);
    void PillarDetectorResultCb(const actionlib::SimpleClientGoalState& state,const pillar_detector::PillarDetectorResultConstPtr& result);
    void BufferOdomData( const nav_msgs::Odometry::ConstPtr& odom );
    
    enum hierarchy{level, room, corridor, side, feature};
    ros::Publisher vis_pub = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
    int building_id = 363;
    int level_id    = 362;
    int corridor_id = 336;
    int room_id     = 306;
    std::queue<nav_msgs::Odometry::ConstPtr> odom_buffer_;
    nav_msgs::Odometry::ConstPtr odom_current;
    
public: 
    Localization();
    virtual ~Localization();
    void test();
    void InitializeOdomBuffer( std::string robot );

    void VisualizeFeatures( std::vector<ropod_ros_msgs::OSMNode> feature_nodes );
    void VisualizeSides( std::vector< int > corner_ids, std::vector<ropod_ros_msgs::OSMRelation> side_relations);

    pillar_detector::PillarDetectorGoal CreatePDGoal( std::vector<ropod_ros_msgs::OSMNode> feature_nodes, nav_msgs::Odometry::ConstPtr& odom_current);
    bool SendGoalOSM(ropod_ros_msgs::OSMQueryGoal goal);
    bool SendGoalPD(pillar_detector::PillarDetectorGoal goal);
    std::vector<int> RecursiveSearch(ropod_ros_msgs::OSMQueryGoal x, hierarchy d);
    std::vector<ropod_ros_msgs::OSMNode> Query_Nodes(ropod_ros_msgs::OSMQueryGoal x);
    std::vector<ropod_ros_msgs::OSMRelation> Query_Sides(ropod_ros_msgs::OSMQueryGoal x);
    std::vector< int > Query_Corners(ropod_ros_msgs::OSMQueryGoal x);  
};
    
#endif /* LOCALIZATION_H */