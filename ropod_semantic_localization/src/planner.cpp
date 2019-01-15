#include <ropod_semantic_localization/localization.h>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "semantic_planner");
  string robot;
  string line;
  ros::NodeHandle node;
  actionlib::SimpleActionClient<ropod_semantic_localization::LocalizationAction> planner("/localization_server",true);
  ropod_semantic_localization::LocalizationGoal plan;
  node.getParam("ropod_semantic_localization/robot", robot);
  ROS_INFO("Semantic Planner Ready!");
  ifstream myfile ("/home/martin/catkin_ws_test/src/semantic_localisation/ropod_semantic_localization/src/localization_plan2.txt");
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      plan.localization_ids.push_back( stoi(line) );
    }
    myfile.close();
  }
  ifstream myfile2 ("/home/martin/catkin_ws_test/src/semantic_localisation/ropod_semantic_localization/src/motion_plan2.txt");
  if (myfile2.is_open())
  {
    while ( getline (myfile2,line) )
    {
      plan.motion_ids.push_back( stoi(line) );
    }
    myfile2.close();
  }
  ROS_INFO("Sending plan");
  planner.waitForServer();
  planner.sendGoal(plan);
  bool finished_before_timeout = planner.waitForResult(ros::Duration(5.0));  
   if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = planner.getState();
      if (planner.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
	ROS_INFO("Plan finished");
      }
    }  
  
  
  
  ros::spin();
  return 0;
}
