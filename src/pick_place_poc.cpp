/*********************************************************************
// Vention take home assignment: Pick and place a cube
// Author: Alexandre Francoeur
// Adapted from moveit_tutorials/pick_place_tutorial
*********************************************************************/

// ROS
#include <cmath>
#include <ros/init.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <string>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;
const double gripper_offset = 0.095; // Approx. offset between the panda_link8 and the tcp

const double cube_dim = 0.04; // 40 mm in meter
const double cube_origin_x = 0.8;
const double cube_origin_y = -0.3;
const double cube_origin_z = cube_dim/2;

const double dropzone_dim = cube_dim;
const double cube_target_x = -0.3;
const double cube_target_y = 0.6;
const double cube_target_z = cube_dim/2;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = cube_dim;
  posture.points[0].positions[1] = cube_dim;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = cube_dim/2;
  posture.points[0].positions[1] = cube_dim/2;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = cube_origin_x - gripper_offset;
  grasps[0].grasp_pose.pose.position.y = cube_origin_y;
  grasps[0].grasp_pose.pose.position.z = cube_origin_z;

  // Setting pre-grasp approach
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  openGripper(grasps[0].pre_grasp_posture);

  // Setting posture of eef during grasp
  closedGripper(grasps[0].grasp_posture);

  // Call pick to pick up the object using the grasps given
  move_group.pick("wood_cube", grasps);

}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0,tau/2,0);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* For place location, we set the value to the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = cube_target_x; 
  place_location[0].place_pose.pose.position.y = cube_target_y; 
  place_location[0].place_pose.pose.position.z = cube_target_z;

  // Setting pre-place approach
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as drop zone.
  group.setSupportSurfaceName("drop_zone");
  // Call place to place the object using the place locations given.
  group.place("wood_cube", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // Creating Environment
  // Create vector to hold collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects; 
  collision_objects.resize(2);

  // Add the end zone where the cube will be dropped.
  collision_objects[0].id = "drop_zone";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = dropzone_dim;
  collision_objects[0].primitives[0].dimensions[1] = dropzone_dim;
  collision_objects[0].primitives[0].dimensions[2] = 0.0;

  /* Define the pose of the drop zone. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = cube_target_x;
  collision_objects[0].primitive_poses[0].position.y = cube_target_y;
  collision_objects[0].primitive_poses[0].position.z = 0.0;
  collision_objects[0].primitive_poses[0].orientation.w = 0.0;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Define the object that we will be manipulating
  collision_objects[1].header.frame_id = "panda_link0";
  collision_objects[1].id = "wood_cube";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = cube_dim;
  collision_objects[1].primitives[0].dimensions[1] = cube_dim;
  collision_objects[1].primitives[0].dimensions[2] = cube_dim;

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = cube_origin_x;
  collision_objects[1].primitive_poses[0].position.y = cube_origin_y;
  collision_objects[1].primitive_poses[0].position.z = cube_origin_z;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);

  // Retreiving the object id strings
  std::vector<std::string> object_ids;
  object_ids.reserve(collision_objects.size());
  for(const auto& obj : collision_objects){
    object_ids.push_back(obj.id);
  }

  // Retreiving the scene objects poses
  auto poses = planning_scene_interface.getObjectPoses(object_ids);
  // Logging the cube and drop zone positions
  for (const auto& obj  : poses) {
    ROS_INFO("%s origin: [x: %s, y: %s, z: %s]", obj.first.c_str(),
                                                 std::to_string(poses[obj.first].position.x).c_str(), 
                                                 std::to_string(poses[obj.first].position.y).c_str(),
                                                 std::to_string(poses[obj.first].position.z).c_str());
  }
}

void removeCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
  // Get all scene objects
  auto scene_objects = planning_scene_interface.getObjects();
  // Create a vector with the id strings
  std::vector<std::string> objects_id;
  objects_id.reserve(scene_objects.size());
  for(const auto&  obj : scene_objects){
    objects_id.push_back(obj.first);
  }
  // Delete the collision objects from the scene
  planning_scene_interface.removeCollisionObjects(objects_id);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();
  
  pick(group);

  ros::WallDuration(1.0).sleep();

  place(group);

  removeCollisionObjects(planning_scene_interface);

  ros::WallDuration(1.0).sleep();

  ros::shutdown();
  return 0;
}