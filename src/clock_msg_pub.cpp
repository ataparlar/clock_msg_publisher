//
// Created by ataparlar on 01.08.2022.
//

#include "../include/clock_msg_publisher/clock_msg_pub.h"


// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.


ClockMsgPublisher::ClockMsgPublisher()
    : Node("ClockMsgPublisher") {
  std::cout.precision(20);

    clock_msg_publisher = this->create_publisher<rosgraph_msgs::msg::Clock>(
            "/clock",
            100);

    vehicle_time_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", 10,
            std::bind(&ClockMsgPublisher::vehicle_time_callback, this, std::placeholders::_1));

    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    writer_->open("my_bag");

}


void ClockMsgPublisher::vehicle_time_callback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr vehicle_msg){
    clock_msg.clock = vehicle_msg->header.stamp;
}

void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
    writer_->write(msg, "clock", "rosgraph_msgs/Clock", clock_msg);
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockMsgPublisher>());
  rclcpp::shutdown();
  return 0;
}
