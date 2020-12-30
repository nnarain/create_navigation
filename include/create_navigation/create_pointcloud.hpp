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
 * Converter bumper and light sensors on the robot to point cloud messages.
*/
class CreatePointCloud
{
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

    enum class ContactPosition
    {
        Front, Left, Right, None
    };

    enum class PointIndex
    {
        // Contact sensors
        ContactFront, ContactLeft, ContactRight,
        // Light sensors
        LightLeft, LightLeftFront, LightLeftCenter,
        LightRight, LightRightFront, LightRightCenter,
        // An invalid point
        Invalid,
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

        cloud_.resize(points_.size());

        cloud_.header.frame_id = frame;
        cloud_.is_dense = false;

        // Create the three points used when the bumper detects an obstacle
        cloud_[static_cast<size_t>(PointIndex::ContactFront)] = getPoint(radius, 0, height);
        cloud_[static_cast<size_t>(PointIndex::ContactLeft)] = getPoint(radius, M_PI / 4.0f, height);
        cloud_[static_cast<size_t>(PointIndex::ContactRight)] = getPoint(radius, -M_PI / 4.0f, height);

        cloud_[static_cast<size_t>(PointIndex::Invalid)] = p_invalid_;

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
        case ContactPosition::Front:
            insertPoint(PointIndex::ContactFront);
            insertPoint(PointIndex::ContactLeft, PointIndex::Invalid);
            insertPoint(PointIndex::ContactRight, PointIndex::Invalid);
            break;
        case ContactPosition::Left:
            insertPoint(PointIndex::ContactLeft);
            insertPoint(PointIndex::ContactFront, PointIndex::Invalid);
            insertPoint(PointIndex::ContactRight, PointIndex::Invalid);
            break;
        case ContactPosition::Right:
            insertPoint(PointIndex::ContactRight);
            insertPoint(PointIndex::ContactFront, PointIndex::Invalid);
            insertPoint(PointIndex::ContactLeft, PointIndex::Invalid);
            break;
        case ContactPosition::None:
            insertPoint(PointIndex::ContactFront, PointIndex::Invalid);
            insertPoint(PointIndex::ContactLeft, PointIndex::Invalid);
            insertPoint(PointIndex::ContactRight, PointIndex::Invalid);
            break;
        }
    }

    ContactPosition getBumpState(const create_msgs::Bumper& msg)
    {
        if (msg.is_left_pressed && msg.is_right_pressed)
        {
            return ContactPosition::Front;
        }
        else if (msg.is_left_pressed)
        {
            return ContactPosition::Left;
        }
        else if (msg.is_right_pressed)
        {
            return ContactPosition::Right;
        }

        return ContactPosition::None;
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

    void insertPoint(PointIndex idx)
    {
        insertPoint(idx, idx);
    }

    void insertPoint(PointIndex cloud_idx, PointIndex point_idx)
    {
        cloud_[static_cast<size_t>(cloud_idx)] = points_[static_cast<size_t>(point_idx)];
    }

    ros::Publisher pc_pub_;
    ros::Timer pub_timer_;
    ros::Subscriber bumper_sub_;

    // The cloud to populate with sensor data
    // In base_link frame
    PointCloud cloud_;
    // An array of possible points in the point cloud
    // 3 contact position points + 6 light sensor points + 1 invalid
    std::array<pcl::PointXYZ, 3 + 6 + 1> points_;
    // A point that is invalid
    pcl::PointXYZ p_invalid_;
};

#endif // CREATE_NAVIGATION_CREATE_POINT_CLOUD_HPP
