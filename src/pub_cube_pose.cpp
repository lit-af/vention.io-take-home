/*********************************************************************
// Vention take home assignment: Publish the cube position and 'up' vector
// Author: Alexandre Francoeur
// Adapted from moveit_tutorials/pick_place_tutorial
*********************************************************************/

// ROS
#include <cmath>
#include <ros/init.h>
#include <ros/rate.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// TF2
#include <string>
#include <vector>

void pubCubePose(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){  
  // Todo: replace hardcoded string with ros::param()
  std::vector<std::string> object_id = {"wood_cube"};
  
  // Retreiving the scene objects poses
  auto poses = planning_scene_interface.getObjectPoses(object_id);
  // Logging the cube pose and 'up' vector
  for (const auto& obj  : poses) {
    ROS_INFO("%s origin: [x: %s, y: %s, z: %s]", obj.first.c_str(),
                                                 std::to_string(poses[obj.first].position.x).c_str(), 
                                                 std::to_string(poses[obj.first].position.y).c_str(),
                                                 std::to_string(poses[obj.first].position.z).c_str());
    ROS_INFO("%s 'up' vector: [%s]", obj.first.c_str(), std::to_string(obj.second.orientation.z).c_str());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_cube_pose");
  ros::NodeHandle nh;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::Rate r(2); // 500ms == 2 Hz
  while (ros::ok())
  {
    // Far from perfect, since once the object is attached to the robot,
    // its coordinates aren't updated in the planning scene.
    pubCubePose(planning_scene_interface);
    r.sleep();
  }
  ros::waitForShutdown();
  
  return 0;
}