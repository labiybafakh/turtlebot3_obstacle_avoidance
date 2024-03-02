#include "obstacle_avoidance.hpp"


ObstacleAvoidance::ObstacleAvoidance(ros::NodeHandle& nh): client("move_base", true){
    ROS_INFO("Waiting..");
    client.waitForServer();
    ROS_INFO("Server OK");
}

ObstacleAvoidance::~ObstacleAvoidance(){
    ros::shutdown();
}

void ObstacleAvoidance::goalPosition(float x, float y, float orientation){
    move_base_msgs::MoveBaseGoal goal_pose;
    tf::Quaternion quat;

    quat.setEuler(orientation, 0, 0);

    goal_pose.target_pose.header.frame_id   = "map";
    goal_pose.target_pose.header.stamp      = ros::Time::now();
    goal_pose.target_pose.pose.position.x   =  x;
    goal_pose.target_pose.pose.position.y   =  y;

    goal_pose.target_pose.pose.orientation.w = quat.getW();
    goal_pose.target_pose.pose.orientation.x = quat.getX();
    goal_pose.target_pose.pose.orientation.y = quat.getY();
    goal_pose.target_pose.pose.orientation.z = quat.getZ();
}