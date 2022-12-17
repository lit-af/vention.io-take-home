cmake_minimum_required(VERSION 3.1.3)
project(vention_take_home)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning_interface
    tf2_geometry_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    tf2_geometry_msgs
)

###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_executable(pick_place_poc src/pick_place_poc.cpp)
target_link_libraries(pick_place_poc ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pub_cube_pose src/pub_cube_pose.cpp)
target_link_libraries(pub_cube_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})