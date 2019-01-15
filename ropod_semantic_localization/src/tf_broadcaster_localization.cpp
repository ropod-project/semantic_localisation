#include <ropod_semantic_localization/localization.h>
#include <tf/transform_broadcaster.h>

nav_msgs::Odometry odom2map_correction_msg;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{	
  ROS_INFO("Updating odom2map correction");
  odom2map_correction_msg.pose.pose.position.x = msg->pose.pose.position.x;
  odom2map_correction_msg.pose.pose.position.y = msg->pose.pose.position.y;
  odom2map_correction_msg.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  odom2map_correction_msg.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  odom2map_correction_msg.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  odom2map_correction_msg.pose.pose.orientation.w = msg->pose.pose.orientation.w;    
  odom2map_correction_msg.header.frame_id = msg->header.frame_id;
  odom2map_correction_msg.child_frame_id = msg->child_frame_id;
  odom2map_correction_msg.header.stamp = msg->header.stamp;  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "localization_tf_publisher");
  std::string robot;
  std::string odomtopic;
  ros::NodeHandle n;
  n.getParam("ropod_semantic_localization/robot", robot);
  odomtopic.append("/");
  odomtopic.append( robot );
  odomtopic.append("/odom");
  tf::TransformBroadcaster broadcaster;
  tf::Transform odom2map_correction;
  
  odom2map_correction_msg.pose.pose.position.x = 0.0;
  odom2map_correction_msg.pose.pose.position.y = 0.0;
  odom2map_correction_msg.pose.pose.orientation.x = 0.0;
  odom2map_correction_msg.pose.pose.orientation.y = 0.0;
  odom2map_correction_msg.pose.pose.orientation.z = 0.0;
  odom2map_correction_msg.pose.pose.orientation.w = 1.0;   
  ros::Rate r(30);

  ros::Subscriber sub_localization = n.subscribe<nav_msgs::Odometry>("localization_msg", 1, poseCallback);
  while(n.ok()){ 
  
    odom2map_correction.setOrigin( tf::Vector3(odom2map_correction_msg.pose.pose.position.x, 
					       odom2map_correction_msg.pose.pose.position.y,
					       0.0) );
    odom2map_correction.setRotation( tf::Quaternion(odom2map_correction_msg.pose.pose.orientation.x, 
						    odom2map_correction_msg.pose.pose.orientation.y, 
						    odom2map_correction_msg.pose.pose.orientation.z, 
						    odom2map_correction_msg.pose.pose.orientation.w) );  

    broadcaster.sendTransform(
      tf::StampedTransform( odom2map_correction, ros::Time::now(),"map", odomtopic ));      
     
    ros::spinOnce();           
    r.sleep();
  }
}