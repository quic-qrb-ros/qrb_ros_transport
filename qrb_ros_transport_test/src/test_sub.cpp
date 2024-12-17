// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_transport_image_type/image.hpp"
#include "qrb_ros_transport_imu_type/imu.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb_ros::transport
{

class TestSubComponent : public rclcpp::Node
{
public:
  explicit TestSubComponent(const rclcpp::NodeOptions & options);

private:
  template <typename T>
  void create_test_subscriber();
  void print_performance_stats(const std_msgs::msg::Header & header);

  rclcpp::SubscriptionBase::SharedPtr sub_{ nullptr };
  std::string topic_name_;

  rclcpp::Time last_calculate_time_{ 0, 0, RCL_STEADY_TIME };
  int msg_count_{ 0 };
  uint64_t sum_latency_{ 0 };
};

TestSubComponent::TestSubComponent(const rclcpp::NodeOptions & options)
  : Node("TestSubComponent", options)
{
  auto message_type = this->declare_parameter("test_type", "qrb_ros::transport::type::Image");
  topic_name_ = this->declare_parameter("topic_name", "image");

  RCLCPP_INFO(get_logger(), "=== Subscribe type: %s, topic_name: %s", message_type.c_str(),
      topic_name_.c_str());

  if (message_type == "qrb_ros::transport::type::Image") {
    create_test_subscriber<type::Image>();
  } else if (message_type == "qrb_ros::transport::type::Imu") {
    create_test_subscriber<type::Imu>();
  } else if (message_type == "sensor_msgs::msg::Image") {
    create_test_subscriber<sensor_msgs::msg::Image>();
  } else if (message_type == "sensor_msgs::msg::Imu") {
    create_test_subscriber<sensor_msgs::msg::Imu>();
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "Unknown type: " << message_type);
  }
}

template <typename T>
void TestSubComponent::create_test_subscriber()
{
  sub_ = this->create_subscription<T>(topic_name_, 30,
      [this](const std::shared_ptr<T> msg) { print_performance_stats(msg->header); });
}

void TestSubComponent::print_performance_stats(const std_msgs::msg::Header & header)
{
  rclcpp::Clock steady_clock{ RCL_STEADY_TIME };

  auto now = steady_clock.now();
  auto diff = now - last_calculate_time_;

  // calculate average performance data every 5s
  if (diff.seconds() > 5) {
    RCLCPP_INFO_STREAM(get_logger(), "fps: " << msg_count_ / diff.seconds() << ", latency: "
                                             << sum_latency_ / msg_count_ << " ns");
    last_calculate_time_ = now;
    msg_count_ = 0;
    sum_latency_ = 0;
  } else {
    msg_count_++;
    sum_latency_ += (this->now() - header.stamp).nanoseconds();
  }
}

}  // namespace qrb_ros::transport

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::transport::TestSubComponent)
