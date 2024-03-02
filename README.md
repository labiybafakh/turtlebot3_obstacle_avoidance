# turtlebot3_obstacle_avoidance

This simulation is based on [turtlebot3 website](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).

## Requirement 
### 1. If gazebo-ros and gazebo-plugins have not been installed, please install it.
```bash
sudo apt-get install ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-move-base-* ros-noetic-actionlib-* ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-dwa-local-planner
```

### 2. Clone Turtlebot3 Dependencies
#### 1. If vcstool has not been installed, please install it.
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-vcstool
```
#### 2. Clone repositories.
```bash
vcs import . < turtlebot3.repos
```

## 3. Build and Run
### 1. Build
```bash
catkin build
source devel/setup.bash
```
### 2. Run
```bash
export TURTLEBOT3_MODEL=burger
roslaunch obstacle_avoidance obastace_avoidance.launch
```