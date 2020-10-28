# Global Body Planner
ROS package for the high level body planner described in "<a href="http://www.andrew.cmu.edu/user/amj1/papers/IROS2020_Fast_Global_Motion_Planning.pdf">Fast Global Motion Planning for Dynamic Legged Robots</a>"

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


