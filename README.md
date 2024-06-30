# FExpT2 - Frontier Exploration for Turtlebot2

[![Wiki](https://img.shields.io/badge/Wiki-Documentation-blue.svg)](https://github.com/KhAlamdar11/FExT2-Frontier-Exploration-for-Turtlebot/wiki)
[![ROS](https://img.shields.io/badge/ROS-Noetic%20or%20later-blue.svg)](http://wiki.ros.org/ROS/Installation)
[![Python](https://img.shields.io/badge/Python-3.7%20or%20later-blue.svg)](https://www.python.org/downloads/)
[![Turtlebot2](https://img.shields.io/badge/Turtlebot2-supported-green.svg)](http://www.turtlebot.com/turtlebot2/)


This package offers a comprehensive frontier exploration (2D) pipeline tailored for Turtlebot2. It includes six distinct candidate point selection methods, four advanced path planners (RRT*, Informed RRT*, BIT*, and FMT*), and two smoothing techniques (B-Spline and Dubins). Designed for flexibility and efficiency, this pipeline facilitates rapid testing and evaluation of various configurations. Furthermore, its adaptable architecture ensures easy translation to other robotic platforms, broadening its applicability beyond Turtlebot2.

Note: For further theoretical and technical details, please consult the [wiki](https://github.com/KhAlamdar11/FExT2-Frontier-Exploration-for-Turtlebot/wiki) tab of this repository.

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
rosrun FExT2 move_to_pt.py
rosrun FExT2 frontier_exploration.py
```

## Parameters:

Different frontier-related, planner, and controller parameters can be modifed in ```/config/config.yaml``` file. Descriptions are povided in this file while detailed descriptions are available in the Note: For further technical details, please consult the [wiki](https://github.com/KhAlamdar11/FExT2-Frontier-Exploration-for-Turtlebot/wiki) tab of this repository.
 of this repo.

## Authors:

- [Khawaja Alamdar](https://github.com/KhAlamdar11)
- [Nada Abbas](https://github.com/NadaAbbas444)
- [Muhammad Awais](https://github.com/Muhammad0312)

