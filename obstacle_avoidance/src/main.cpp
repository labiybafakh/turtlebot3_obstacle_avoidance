#include "obstacle_avoidance.hpp"
#include <memory>

typedef struct{
    float x,y,z;
} Vector3D;

std::vector<Vector3D> way_points = {
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0}
};

using move_base_client = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

move_base_msgs::MoveBaseGoal goalPosition(float x, float y, float orientation){
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

    return goal_pose;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle node_handler;
    move_base_client client("move_base", true);
    ros::Rate rate(100);

    // ObstacleAvoidance nav_robot(node_handler);

    auto nav_robot = std::make_shared<ObstacleAvoidance>(node_handler);

    // ROS_INFO("Waiting for server..");
    // client.waitForServer();
    // ROS_INFO("Server is connected!");


    while(!ros::isShuttingDown()){

        nav_robot->keepMoving();
        // client.sendGoal(goalPosition(0,-0.5,0));
        // client.waitForResult();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}