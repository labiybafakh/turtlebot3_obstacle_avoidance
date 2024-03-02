#ifndef obstacle_avoidance_hpp
#define obstacle_avoidance_hpp

#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib/client/simple_action_client.h"
#include <tf/transform_broadcaster.h>

class ObstacleAvoidance{

protected:
    using move_base_client = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
    move_base_client client;

public:
    ObstacleAvoidance(ros::NodeHandle& nh);
    ~ObstacleAvoidance();
    void goalPosition(float x, float y, float orientation);

};

#endif