# turtlebot3_obstacle_avoidance

## Introduction

This simulation is based on [turtlebot3 website](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).

## Prequisition 
### 1. ROS
1. ros-noetic-dekstop installed.
2. Install some packages required below
```bash
sudo apt-get install ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-move-base-* ros-noetic-actionlib-* ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-dwa-local-planner ros-noetic-slam-karto ros-noetic-map-server ros-noetic-amcl python3-catkin-tools
```

### 2. Clone repositories.
1. Clone repository
```bash
cd ~
mkdir -p noetic_ws/src && cd noetic_ws/src
git clone https://github.com/labiybafakh/turtlebot3_obstacle_avoidance
```
2. Get turtlebot3 packages to run this reposity, but if it has been installed on system, you can skip it.
If vcstool has not been installed, please install it.
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-vcstool
```
```bash
cd turtlebot3_obstacle_avoidance
vcs import . < turtlebot3.repos
```

## Build and Run
### 1. Build
```bash
cd ~/noetic_ws/
catkin build
source devel/setup.bash
```
### 2. Run
#### 1. Creating Map.
to create the map using slam-karto, it can be done by run create_map.launch file.
```bash
export TURTLEBOT3_MODEL=burger
roslaunch obstacle_avoidance creating_map.launch
```
Run turtlebot3_teleop to move the robot by using turtlebot3_teleop_key.launch.
```bash
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
after the whole area has been mapped, teleop can be terminated and save the map by run another launch file.
```bash
source devel/setup.bash
roslaunch obstacle_avoidance map_saver.launch
```
Another shortcut, it also can be also done by changing the path inside the map.yaml file. 
```yaml
image: /home/toys/noetic_ws/src/turtlebot3_obstacle_avoidance/obstacle_avoidance/maps/map.pgm
```

#### 2. Run simulation environment.
Make sure that it can be ran like the video below by using 2D Nav Goal tool in RViz. If it is ok, it is better to re-run the launch file, but it also ok to go to the next step.
```bash
export TURTLEBOT3_MODEL=burger
source devel/setup.bash
roslaunch obstacle_avoidance simulation.launch
```
https://github.com/labiybafakh/turtlebot3_obstacle_avoidance/assets/16336991/f77f1c1a-8961-4169-a508-8bca80552e4a

#### 2. Run the node to publish random position and keep moving.
It should be run in different terminal.
```bash
export TURTLEBOT3_MODEL=burger
source devel/setup.bash
rosrun obstacle_avoidance obastacle_avoidance
```
https://github.com/labiybafakh/turtlebot3_obstacle_avoidance/assets/16336991/936ef258-6614-4399-bc51-da9030d0500e




