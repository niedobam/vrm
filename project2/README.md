# Final Project

This is final project for university course VRM - Programming for robots and manipulators, at VUT Brno. It is a simple application of Fanuc CR-7iA robot simulation using [ROS](https://www.ros.org/).

In robot's working envelope at random position is spawned an object (in our case a cube) to which robot will move. Trajectory of robot's movement is visualised.

Program steps:
  1. Robot will move to home position
  2. Cube will be spawned at random position in robot's working envelope
  3. Robot trajectory will be calculated
  4. Planned trajectory will be visualized (in RViz "Marker" visualization has to be added)
  5. Repeat

![Screenshot_20230604_102058](https://github.com/niedobam/vrm/assets/127039716/762a2788-a249-44cc-bc1c-31a376fdc0e0)

## Requirements
Project was made using [ROS Melodic Morenia](http://wiki.ros.org/melodic#Installation) on [Ubuntu 18.04 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04.6/).

## Installation instructions
* Create project directory
  ```bash
  mkdir -p <directory_name>/src
  ```
* Go into project directory
  ```bash
  cd <directory_name>/src
  ```
* Download Fanuc ROS1 drivers
  ```bash
  git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/fanuc.git
  ```
* Download project source files
  ```bash
  git clone LINK
  ```
* Install project with all dependencies
  ```bash
  rosdep update
  rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src
  catkin_make
  ```

## Running instructions - Go into project directory
* In terminal1
  ```bash
  source devel/setup.bash
  roslaunch fanuc_cr7ia_moveit_config demo.launch
  ```
* In terminal2
  ```bash
  source devel/setup.bash
  roslaunch robot_ctrl demo.launch
  ```
* In terminal3
  ```bash
  source devel/setup.bash
  rosrun robot_ctrl test.py
  ```
