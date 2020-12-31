//
// create_pointcloud_node.cpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Dec 22 2020
//
#include <ros/ros.h>
#include "create_navigation/create_pointcloud.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "create_pointcloud");

    ros::NodeHandle nh;

    CreatePointCloud converter;

    if (converter.initialize(nh))
    {
        ros::spin();
    }
    else
    {
        ROS_ERROR("Error initializing point cloud converter");
        return 1;
    }

    return 0;
}
