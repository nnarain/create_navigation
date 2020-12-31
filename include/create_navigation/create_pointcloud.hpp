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
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

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

        int detect = 200;
        pnh.getParam("light_detect", detect);
        light_detection_threshold_ = detect;

        cloud_.header.frame_id = frame;
        cloud_.is_dense = false;

        cloud_.resize(points_.size());

        // Create the three points used when the bumper detects an obstacle
        points_[static_cast<size_t>(PointIndex::ContactFront)] = getPointOnRadius(radius, 0, height);
        points_[static_cast<size_t>(PointIndex::ContactLeft)] = getPointOnRadius(radius, M_PI / 4.0f, height);
        points_[static_cast<size_t>(PointIndex::ContactRight)] = getPointOnRadius(radius, -M_PI / 4.0f, height);

        points_[static_cast<size_t>(PointIndex::Invalid)] = p_invalid_;

        // Insert defaults points
        insertPoint(PointIndex::ContactFront, PointIndex::Invalid);
        insertPoint(PointIndex::ContactLeft, PointIndex::Invalid);
        insertPoint(PointIndex::ContactRight, PointIndex::Invalid);
        insertPoint(PointIndex::LightLeft, PointIndex::Invalid);
        insertPoint(PointIndex::LightLeftFront, PointIndex::Invalid);
        insertPoint(PointIndex::LightLeftCenter, PointIndex::Invalid);
        insertPoint(PointIndex::LightRight, PointIndex::Invalid);
        insertPoint(PointIndex::LightRightFront, PointIndex::Invalid);
        insertPoint(PointIndex::LightRightCenter, PointIndex::Invalid);

        pc_pub_ = nh.advertise<PointCloud>("bumper/pointcloud", 1);
        pub_timer_ = nh.createTimer(ros::Duration{0.1}, &CreatePointCloud::pubTimerCallback, this);

        bumper_sub_ = nh.subscribe("bumper", 1, &CreatePointCloud::bumperCallback, this);

        tf_init_timer_ = nh.createTimer(ros::Duration{0.1}, &CreatePointCloud::tfInitTimerCallback, this, true);

        return true;
    }

private:
    void bumperCallback(const create_msgs::BumperConstPtr& msg)
    {
        processContactSensors(*msg);
        processLightSensors(*msg);
    }

    void processContactSensors(const create_msgs::Bumper& msg)
    {
        const auto bump_state = getBumpState(msg);

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

    void processLightSensors(const create_msgs::Bumper& msg)
    {
        processLightSensor(PointIndex::LightLeft, msg.light_signal_left);
        processLightSensor(PointIndex::LightLeftFront, msg.light_signal_front_left);
        processLightSensor(PointIndex::LightLeftCenter, msg.light_signal_center_left);
        processLightSensor(PointIndex::LightRight, msg.light_signal_right);
        processLightSensor(PointIndex::LightRightFront, msg.light_signal_front_right);
        processLightSensor(PointIndex::LightRightCenter, msg.light_signal_center_right);
    }

    void processLightSensor(PointIndex idx, uint16_t sensor)
    {
        if (sensor >= light_detection_threshold_)
        {
            insertPoint(idx);
        }
        else
        {
            insertPoint(idx, PointIndex::Invalid);
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

    pcl::PointXYZ getPointOnRadius(float radius, float angle, float height)
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

    void tfInitTimerCallback(const ros::TimerEvent&)
    {
        ros::NodeHandle pnh{"~"};

        float light_point_range = 0.075f;
        pnh.getParam("light_range", light_point_range);

        points_[static_cast<size_t>(PointIndex::LightLeft)] = getSensorPoint("left_light_sensor_link", light_point_range);
        points_[static_cast<size_t>(PointIndex::LightLeftFront)] = getSensorPoint("left_front_light_sensor_link", light_point_range);
        points_[static_cast<size_t>(PointIndex::LightLeftCenter)] = getSensorPoint("left_center_light_sensor_link", light_point_range);
        points_[static_cast<size_t>(PointIndex::LightRight)] = getSensorPoint("right_light_sensor_link", light_point_range);
        points_[static_cast<size_t>(PointIndex::LightRightFront)] = getSensorPoint("right_front_light_sensor_link", light_point_range);
        points_[static_cast<size_t>(PointIndex::LightRightCenter)] = getSensorPoint("right_center_light_sensor_link", light_point_range);
    }

    pcl::PointXYZ getSensorPoint(const std::string& frame, float distance_offset)
    {
        geometry_msgs::PointStamped sensor_base;
        sensor_base.header.frame_id = frame;
        sensor_base.header.stamp = ros::Time{};
        sensor_base.point.x = distance_offset;
        sensor_base.point.y = 0;
        sensor_base.point.z = 0;

        geometry_msgs::PointStamped sensor_position_base_link;

        try
        {
            listener_.waitForTransform("base_link", frame, ros::Time{}, ros::Duration(10.0));
            listener_.transformPoint("base_link", sensor_base, sensor_position_base_link);
        }
        catch(const std::runtime_error& e)
        {
            ROS_ERROR_STREAM("Error computing light sensor positions: " << e.what());
        }


        pcl::PointXYZ sensor_point;
        sensor_point.x = sensor_position_base_link.point.x;
        sensor_point.y = sensor_position_base_link.point.y;
        sensor_point.z = sensor_position_base_link.point.z;

        return sensor_point;
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

    ros::Timer tf_init_timer_;

    // The cloud to populate with sensor data
    // In base_link frame
    PointCloud cloud_;
    // An array of possible points in the point cloud
    // 3 contact position points + 6 light sensor points + 1 invalid
    std::array<pcl::PointXYZ, 3 + 6 + 1> points_;
    // A point that is invalid
    pcl::PointXYZ p_invalid_;

    uint16_t light_detection_threshold_;

    tf::TransformListener listener_;
};

#endif // CREATE_NAVIGATION_CREATE_POINT_CLOUD_HPP
