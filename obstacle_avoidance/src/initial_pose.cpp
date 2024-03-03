#include "ros/ros.h"    
#include "geometry_msgs/PoseWithCovarianceStamped.h"



int main(int argc, char** argv){

    ros::init(argc, argv, "init_pose");

    ros::NodeHandle nh;
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 50);

    ros::Rate rate(20);

    //Check whether the initial_pose has activated or not.
    while(pose_publisher.getNumSubscribers() < 1){
        ROS_INFO("Waiting intialpose subscriber....");
        rate.sleep();
    }

    for(int i=0; i<2; i++){
        /*
            Publish initialpose to adjust the tf of map into the correct position.
        */
        geometry_msgs::PoseWithCovarianceStamped initial_pose;

        initial_pose.header.frame_id        = "map";
        initial_pose.header.stamp           = ros::Time::now();
        initial_pose.pose.pose.position.x   = -2.0;
        initial_pose.pose.pose.position.y   = -0.5;
        initial_pose.pose.pose.position.z   = 0;
        initial_pose.pose.pose.orientation.x= 0;
        initial_pose.pose.pose.orientation.y= 0;
        initial_pose.pose.pose.orientation.z= 0;
        initial_pose.pose.pose.orientation.w= 1;

        pose_publisher.publish(initial_pose);
        ROS_INFO("Map pose has been adjusted.");

        ros::spinOnce();
        rate.sleep();
    }



    return 0;

}