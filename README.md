# vention-take-home

This script is an adaptation of the [moveit_tutorials/pick_place_tutorial](https://ros-planning.github.io/moveit_tutorials/doc/pick_place/pick_place_tutorial.html)

## Dependencies
- ROS noetic
- [MoveIt](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#install-ros-and-catkin)

## Setup
1. `mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src`
2. `git clone https://github.com/makingRobotsDoStuff/take-home.git`
3. `cd ~/catkin_ws`
4. `rosdep install --from-paths src --ignore-src -r -y`
5. `catkin_make`
6. `source devel/setup.bash`

## Usage
1. In a first terminal, launch the panda arm demo
```bash
roslaunch panda_moveit_config demo.launch
```
2. In a second terminal, launch the cube pose publisher node
```bash
roslaunch vention_take_home pub_cube_pose.launch
```
3. In a third terminal, launch the pick and place node
```bash
roslaunch vention_take_home pick_place_poc.launch
```
4. Enjoy!

## Development Notes
### Todo
- [X] print: Starting position of the cube
- [X] print: the destination of the cube
- [X] Every 500ms: print the cube pose
- [X] Every 500ms: print the cube 'up' vector

### Known bugs
- The cube pose is only updated when it's part of the planning scene. While it's attached to the manipulator, it's coordinates in the planning scene remain the same as the last time it was part of the planning scene.