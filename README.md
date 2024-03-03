# turtlebot3_obstacle_avoidance

This simulation is based on [turtlebot3 website](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).

## Requirement 
### 1. If gazebo-ros and gazebo-plugins have not been installed, please install it.
```bash
sudo apt-get install ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-move-base-* ros-noetic-actionlib-* ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-dwa-local-planner ros-noetic-slam-karto
```
If vcstool has not been installed, please install it.
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-vcstool
```

### 2. Clone repositories.
1. Clone this repository.
```bash
cd ~
mkdir -p noetic_ws/src && cd noetic_ws/src
git clone https://github.com/labiybafakh/turtlebot3_obstacle_avoidance
```
2. Get another turtlebot3 packages to run this reposity, but if it has been installed on system, you can skip it.
```bash
cd turtlebot3_obstacle_avoidance
vcs import . < turtlebot3.repos
```

## Build and Run
### 1. Build
```bash
cd ~/noetic_ws/
catkin build
```
### 2. Run
#### 1. Run the simulation environment.
```bash
export TURTLEBOT3_MODEL=burger
source devel/setup.bash
roslaunch obstacle_avoidance simulation.launch
```

#### 2. Run the node to publish random position and keep moving.
It should be run in different terminal.
```bash
export TURTLEBOT3_MODEL=burger
source devel/setup.bash
rosrun obstacle_avoidance obastacle_avoidance
```