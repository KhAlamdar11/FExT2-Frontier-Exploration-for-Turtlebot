# FExpT2 - Frontier Exploration for Turtlebot2

[![ROS](https://img.shields.io/badge/ROS-Noetic%20or%20later-blue.svg)](http://wiki.ros.org/ROS/Installation)
[![Python](https://img.shields.io/badge/Python-3.7%20or%20later-blue.svg)](https://www.python.org/downloads/)
[![Turtlebot2](https://img.shields.io/badge/Turtlebot2-supported-green.svg)](http://www.turtlebot.com/turtlebot2/)


This package offers a comprehensive frontier exploration (2D) pipeline tailored for Turtlebot2. It includes six distinct candidate point selection methods, four advanced path planners (RRT*, Informed RRT*, BIT*, and FMT*), and two smoothing techniques (B-Spline and Dubins). Designed for flexibility and efficiency, this pipeline facilitates rapid testing and evaluation of various configurations. Furthermore, its adaptable architecture ensures easy translation to other robotic platforms, broadening its applicability beyond Turtlebot2.

Note: For further technical details, please consult the wiki tab of this repository.

## Installation

1. **Dependencies**: Ensure ROS (Robot Operating System) and Python 3 are installed on your system. Additionally, install:

    ```bash
    pip install scikit-image
    sudo apt-get install ros-noetic-actionlib
    ```

2. **Clone the Repository**: Clone this repository into your ROS workspace's `src` directory.
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/KhAlamdar11/FExT2-Frontier-Exploration-for-Turtlebot.git
   ```

3. **Build the Package**: From the root of your ROS workspace, build the package:.
   ```bash
   cd ~/catkin_ws
   catkin build
   ```

4. **Source the Workspace**: Source your ROS workspace to make the package available.
   ```bash
   source devel/setup.bash
   ```


## HOW TO RUN:

To run a simulation, execute the following commands in seperate terminal windows:

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch FExT2 frontier_exploration.launch world_name:=pathway.world
rosrun FExT2 frontier_exploration.py
rosrun FExT2 move_to_pt.py
```

## Parameters:

<span style="color: red;">TODO: Update parameter changing via a single config file</span>

Modify the two attributes in move_to_point.py to set planner configuration (path planner, path smoother and controller). For example:

1. self.planner_config = 'BIT-' (other options: RRTStarOMPL, InRRTStar-, FMT-, BIT-, InRRTStar-Dubins, FMT-Dubins, BIT-Dubins, InRRTStar-BSpline, FMT-BSpline, BIT-BSpline)
2. self.curved_coltroller = False (true if using post processing options ie. dubins or bspline)

Information gain choice can be selected by setting self.criterion variable in frontier_exploration.py. Options available:

0. nearest frontier
1. farthest frontier
2. maximum area
3. maximum information gain (entropy based)
4. biggest cluster
5. dist weighted information gain

General path planner parameters (eg dominion) can be modified from the init of move_to_point. Parameters specific to different path planners need to be modified in utils_lib/path_planners.py file

Update robot width in:

1. frontier_class.py as self.robot_len
2. move_to_point.py as last argument of OnlinePlanner('/projected_map', '/odom', '/cmd_vel', np.array([-15.0, 15.0]), 0.2)

## Authors:

- [Khawaja Alamdar](https://github.com/KhAlamdar11)
- [Nada Abbas](https://github.com/NadaAbbas444)
- [Muhammad Awais](https://github.com/Muhammad0312)

