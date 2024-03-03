#ifndef obstacle_avoidance_hpp
#define obstacle_avoidance_hpp

#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib/client/simple_action_client.h"
#include <tf/transform_broadcaster.h>
#include <iostream>


struct pose_data{
    float x;
    float y;
    float theta;
};

class ObstacleAvoidance{

protected:
    using move_base_client = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
    move_base_client client;
    bool run_flag;
    pose_data robot_pose;

public:
    ObstacleAvoidance(ros::NodeHandle& nh);
    ~ObstacleAvoidance();
    move_base_msgs::MoveBaseGoal goalPosition(pose_data goal_pose);
    bool isPositionReachable(pose_data goal_pose);
    pose_data randomPosition();
    void keepMoving();

};

#endif