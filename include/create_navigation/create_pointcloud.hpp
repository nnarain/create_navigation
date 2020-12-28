//
// create_pointcloud.hpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Dec 22 2020
//
#ifndef CREATE_NAVIGATION_CREATE_POINT_CLOUD_HPP
#define CREATE_NAVIGATION_CREATE_POINT_CLOUD_HPP

#include <ros/ros.h>
#include <create_msgs/Bumper.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <limits>
#include <cmath>

/**
 * Convert sensors on the Create robot in a point cloud usable by nav stack
*/
class CreatePointCloud
{
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

    enum class BumpPosition
    {
        Front, Left, Right, None
    };

    static constexpr auto INVALID = std::numeric_limits<float>::quiet_NaN();

public:
    CreatePointCloud()
        : p_invalid_{INVALID, INVALID, INVALID}
    {
    }

    bool initialize(ros::NodeHandle& nh)
    {
        ros::NodeHandle pnh{"~"};

        std::string frame{"base_link"};
        pnh.getParam("frame", frame);

        float radius = 0;
        if (!pnh.getParam("radius", radius))
        {
            ROS_ERROR("Point cloud radius not defined!");
            return false;
        }

        float height = 0;
        if (!pnh.getParam("height", height))
        {
            ROS_ERROR("Point clod height not defined!");
            return false;
        }

        cloud_.header.frame_id = frame;
        cloud_.is_dense = false;

        // Add the three invalid points
        cloud_.resize(3);
        cloud_[0] = p_invalid_;
        cloud_[1] = p_invalid_;
        cloud_[2] = p_invalid_;

        // Create the three points used when the bumper detects an obstacle
        p_front_ = getPoint(radius, 0, height);
        p_left_ = getPoint(radius, M_PI / 4.0f, height);
        p_right_ = getPoint(radius, -M_PI / 4.0f, height);

        pc_pub_ = nh.advertise<PointCloud>("bumper/pointcloud", 1);
        pub_timer_ = nh.createTimer(ros::Duration{0.1}, &CreatePointCloud::pubTimerCallback, this);

        bumper_sub_ = nh.subscribe("bumper", 1, &CreatePointCloud::bumperCallback, this);

        return true;
    }

private:
    void bumperCallback(const create_msgs::BumperConstPtr& msg)
    {
        const auto bump_state = getBumpState(*msg);

        // Set points in the cloud depending on what the bump position is
        // When there is no contact on one of the sensors a point is added at infinity
        switch(bump_state)
        {
        case BumpPosition::Front:
            cloud_[0] = p_front_;
            cloud_[1] = p_invalid_;
            cloud_[2] = p_invalid_;
            break;
        case BumpPosition::Left:
            cloud_[0] = p_invalid_;
            cloud_[1] = p_left_;
            cloud_[2] = p_invalid_;
            break;
        case BumpPosition::Right:
            cloud_[0] = p_invalid_;
            cloud_[1] = p_invalid_;
            cloud_[2] = p_right_;
            break;
        case BumpPosition::None:
            cloud_[0] = p_invalid_;
            cloud_[1] = p_invalid_;
            cloud_[2] = p_invalid_;
            break;
        }
    }

    BumpPosition getBumpState(const create_msgs::Bumper& msg)
    {
        if (msg.is_left_pressed && msg.is_right_pressed)
        {
            return BumpPosition::Front;
        }
        else if (msg.is_left_pressed)
        {
            return BumpPosition::Left;
        }
        else if (msg.is_right_pressed)
        {
            return BumpPosition::Right;
        }

        return BumpPosition::None;
    }

    pcl::PointXYZ getPoint(float radius, float angle, float height)
    {
        const auto x = radius * cos(angle);
        const auto y = radius * sin(angle);

        return pcl::PointXYZ{x, y, height};
    }

    void pubTimerCallback(const ros::TimerEvent&)
    {
        pcl_conversions::toPCL(ros::Time::now(), cloud_.header.stamp);
        pc_pub_.publish(cloud_);
    }

    ros::Publisher pc_pub_;
    ros::Timer pub_timer_;
    ros::Subscriber bumper_sub_;

    PointCloud cloud_;
    pcl::PointXYZ p_front_;
    pcl::PointXYZ p_left_;
    pcl::PointXYZ p_right_;
    pcl::PointXYZ p_invalid_;
};

#endif // CREATE_NAVIGATION_CREATE_POINT_CLOUD_HPP
