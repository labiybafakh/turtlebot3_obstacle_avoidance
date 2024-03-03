#include "obstacle_avoidance.hpp"
#include <memory>


int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle node_handler;
    ros::Rate rate(100);

    auto nav_robot = std::make_shared<ObstacleAvoidance>(node_handler);


    while(ros::ok()){

        nav_robot->keepMoving(); //Run the keep moving function with given random goal until the goal reached.

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}