//
// Created by ataparlar on 01.08.2022.
//

#ifndef BUILD_CLOCK_MSG_PUB_H
#define BUILD_CLOCK_MSG_PUB_H

#include <rosbag2_cpp/writer.hpp>

#include <applanix_msgs/msg/navigation_solution_gsof49.hpp>
#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <memory>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"


class ClockMsgPublisher : public rclcpp::Node {
public:
    ClockMsgPublisher();

    // Publishers
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_msg_publisher;

    // Subscribers
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_time_sub_;

    // Messages
    rosgraph_msgs::msg::Clock clock_msg;

    // Callbacks
    void vehicle_time_callback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg);
    void bag_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);


        // ------------------
    std::unique_ptr<rosbag2_cpp::Writer> writer_;

};

#endif //BUILD_CLOCK_MSG_PUB_H
