#include "obstacle_avoidance.hpp"
#include <random>
#include <cstdlib>
#include <ctime> 

ObstacleAvoidance::ObstacleAvoidance(ros::NodeHandle& nh): client("move_base", true){
    ROS_INFO("Waiting for server....");
    client.waitForServer();
    ROS_INFO("Server OK!");
}

ObstacleAvoidance::~ObstacleAvoidance(){
    ros::shutdown();
}

move_base_msgs::MoveBaseGoal ObstacleAvoidance::goalPosition(pose_data desire_pose){
    
    move_base_msgs::MoveBaseGoal goal_pose;
    tf::Quaternion quat;

    quat.setEuler(0, 0, desire_pose.theta);

    goal_pose.target_pose.header.frame_id   = "map";
    goal_pose.target_pose.header.stamp      = ros::Time::now();
    goal_pose.target_pose.pose.position.x   =  desire_pose.x;
    goal_pose.target_pose.pose.position.y   =  desire_pose.y;

    goal_pose.target_pose.pose.orientation.w = quat.getW();
    goal_pose.target_pose.pose.orientation.x = quat.getX();
    goal_pose.target_pose.pose.orientation.y = quat.getY();
    goal_pose.target_pose.pose.orientation.z = quat.getZ();

    return goal_pose;
}

void ObstacleAvoidance::keepMoving(){
    pose_data pose = this->randomPosition();

    

    client.sendGoal(this->goalPosition(this->randomPosition()));
    client.waitForResult();
    
    bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("Goal reached: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not complete before the time out.");
    }
}

bool ObstacleAvoidance::isPositionReachable(pose_data goal_pose){

}

pose_data ObstacleAvoidance::randomPosition(){
    pose_data random_pose;
    srand(time(0));

    const float x_min = -2.1;
    const float x_max = 2.1;
    const float y_min = -2.1;
    const float y_max = 2.1;
    const float t_min = -3.12;
    const float t_max = 3.12;

    // std::random_device rd;  // Will be used to obtain a seed for the random number engine
    // std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()

    random_pose.x       = x_min + (static_cast<float>(rand()) / RAND_MAX) * (x_max - (x_min));
    random_pose.y       = y_min + (static_cast<float>(rand()) / RAND_MAX) * (y_max - (y_min));
    random_pose.theta   = t_min + (static_cast<float>(rand()) / RAND_MAX) * (t_max - (t_min));

    ROS_INFO("random -> %.2f %.2f %.2f", random_pose.x, random_pose.y, random_pose.theta);


    return random_pose;


}



