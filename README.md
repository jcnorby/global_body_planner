# Global Body Planner
ROS package for the high level body planner described in "<a href="http://www.andrew.cmu.edu/user/amj1/papers/IROS2020_Fast_Global_Motion_Planning.pdf">Fast Global Motion Planning for Dynamic Legged Robots</a>"

### About
This ROS package provides dynamic motion primitives to guide a legged robot from a start state to a goal state while navigating a provided height map. The algorithm is intended sit at the top of an autonomous navigation stack, providing valuable information to guide lower-level footstep planners and/or whole-body controllers to good solutions. The primitives are trajectories that satisfy constraints on the kinematic and dynamic abilities of the desired robot, and can include both stance and flight phases.

The implementation of this algorithm is based on RRT-Connect, with definitions of states and actions that align with legged robot dynamics. The height map data can be specified either directly in source or loaded from a CSV, and is transferred to a common [ROS-based height map data structure](https://github.com/ANYbotics/grid_map). The outputs of the planner are ROS messages with the full body trajectory, as well as the discrete states which define the motion primitives used to create that trajectory. Interface classes are provided to visualize this trajectory with RViz.

### Installation
This package assumes you have [ROS Melodic](http://wiki.ros.org/melodic/Installation) installed and that `~/catkin_ws/src` is a valid path. It is recommended to have at least the base desktop version so you can visualize the planner output with RViz. Remember that the `setup.bash` file must be sourced in any opened terminal.

```
cd ~/catkin_ws/src
git clone https://github.com/jcnorby/global_body_planner.git
./global_body_planner/setup_deps.sh
cd ..
catkin_make
source devel/setup.bash
```

### Usage
A launch file is included to run the planner on an example terrain and visualize the output. This can be called with

```
roslaunch global_body_planner example.launch
```

Several parameters set in `config/params.yaml` can be changed to test different scenarios and planning settings. See the comments in that file for the currently available options.

### Documentation
Documentation for the classes in this package can be created with
```
cd ~/catkin_ws/src/global_body_planner
doxygen Doxyfile
firefox docs/index.html
```
Contact Joe Norby at jnorby@andrew.cmu.edu with questions, or better yet submit Issues/Pull Requests if you find something that can be improved!

