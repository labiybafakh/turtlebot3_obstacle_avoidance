#include "obstacle_avoidance.hpp"
#include <random>
#include <cstdlib>
#include <ctime> 

ObstacleAvoidance::ObstacleAvoidance(ros::NodeHandle& nh): client("move_base", true){

    map_sub = nh.subscribe("map", 10, &ObstacleAvoidance::mapCallback, this);

    ROS_INFO("Waiting for server....");
    client.waitForServer();
    ROS_INFO("Server OK!");

}

ObstacleAvoidance::~ObstacleAvoidance(){
    ros::shutdown();
}

move_base_msgs::MoveBaseGoal ObstacleAvoidance::goalPosition(pose_data desire_pose){
    /*
        Compile the desire or goal pose into move_base message.
    */
    
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
    /*
        Robot will keep move by given random point for every
        previous goal has been reached.
    */

    pose_data pose = this->randomPosition();
    
    ROS_INFO("Goal Pose: -> x:%f, y:%f, theta:%f", pose.x, pose.y, pose.theta);

    if(!isPositionReachable(pose)){
        ROS_ERROR("Goal is unreachable");
    }
    else{
        ROS_INFO("Goal is reachable");
        client.sendGoal(this->goalPosition(pose));
        
        //Time out of the move_base if the goal pose has not reached in 30s 
        bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = client.getState();
            ROS_INFO("Goal reached: %s", state.toString().c_str());
        } else {
            ROS_WARN("Action did not complete before the time out.");
        }
    }

}

pose_data ObstacleAvoidance::randomPosition(){
    /*
        Generate random position and orientation for robot to make the robot keep moving.
    */
    pose_data random_pose;
    srand(time(0));

    //define boundary of generation random value according to the map
    const float x_min = -2.1;
    const float x_max = 2.1;
    const float y_min = -2.1;
    const float y_max = 2.1;
    const float t_min = -3.12;
    const float t_max = 3.12;

    random_pose.x       = x_min + (static_cast<float>(rand()) / RAND_MAX) * (x_max - (x_min));
    random_pose.y       = y_min + (static_cast<float>(rand()) / RAND_MAX) * (y_max - (y_min));
    random_pose.theta   = t_min + (static_cast<float>(rand()) / RAND_MAX) * (t_max - (t_min));

    return random_pose;
}

bool ObstacleAvoidance::isPositionReachable(pose_data goal_pose){
    /*
        It will check the given position is reachable by checking the data of occupancy grid.
        The reachable area should be a free grid which represent value 0 of its occupancy grid data.
    */

    double resolution = current_map.info.resolution;
    double origin_x = current_map.info.origin.position.x;
    double origin_y = current_map.info.origin.position.y;
    
    int grid_x = (int)((goal_pose.x - origin_x) / resolution); 
    int grid_y = (int)((goal_pose.y - origin_y) / resolution);

    //Check whether that grid is inside the boundary or not 
    if (grid_x >= 0 && grid_x < current_map.info.width && grid_y >= 0 && grid_y < current_map.info.height) {
        int index = (grid_y * current_map.info.width) + grid_x;
        // Check whether that grid is free or not
        if (current_map.data[index] == 0) {
            //target is free
            return true;
        }
        else{
            //target is not free
            return false;
        }
    } else {
        // target is out of bounds
        return false;
    }
}

void ObstacleAvoidance::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid_map) {
    //collect information of the map
    this->current_map = *grid_map;
}
