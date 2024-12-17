// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <fstream>

#include "qrb_ros_transport_image_type/image.hpp"
#include "qrb_ros_transport_imu_type/imu.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb_ros::transport
{

class TestDumpComponent : public rclcpp::Node
{
public:
  explicit TestDumpComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::SubscriptionBase::SharedPtr sub_{ nullptr };

  template <typename T>
  void create_dump_subscriber();

  void dump_msg(const std::shared_ptr<qrb_ros::transport::type::Image> msg);
  void dump_msg(const std::shared_ptr<sensor_msgs::msg::Image> msg);
  void dump_msg(const std::shared_ptr<qrb_ros::transport::type::Imu> msg);
  void dump_msg(const std::shared_ptr<sensor_msgs::msg::Imu> msg);

  void save_data_to_file(const std::string & path, const char * data, std::size_t size);
  void read_dmabuf_to_file(const std::string & path,
      std::shared_ptr<lib_mem_dmabuf::DmaBuffer> dmabuf);

  std::string dump_path_;
  std::string dump_topic_;
};

TestDumpComponent::TestDumpComponent(const rclcpp::NodeOptions & options)
  : Node("TestDumpComponent", options)
{
  auto message_type = this->declare_parameter("test_type", "qrb_ros::transport::type::Image");
  dump_topic_ = this->declare_parameter("topic_name", "image");
  dump_path_ = this->declare_parameter("dump_file", "/data/dump");

  RCLCPP_INFO(
      get_logger(), "=== Dump type: %s, topic_name: %s", message_type.c_str(), dump_topic_.c_str());

  if (message_type == "qrb_ros::transport::type::Image") {
    create_dump_subscriber<qrb_ros::transport::type::Image>();
  } else if (message_type == "qrb_ros::transport::type::Imu") {
    create_dump_subscriber<qrb_ros::transport::type::Imu>();
  } else if (message_type == "sensor_msgs::msg::Image") {
    create_dump_subscriber<sensor_msgs::msg::Image>();
  } else if (message_type == "sensor_msgs::msg::Imu") {
    create_dump_subscriber<sensor_msgs::msg::Imu>();
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "Unknown type: " << message_type);
  }
}

template <typename T>
void TestDumpComponent::create_dump_subscriber()
{
  sub_ = this->create_subscription<T>(
      dump_topic_, 30, [this](const std::shared_ptr<T> msg) { dump_msg(msg); });
}

void TestDumpComponent::save_data_to_file(const std::string & path,
    const char * data,
    std::size_t size)
{
  std::ofstream file(path, std::ios::binary);
  if (!file.is_open()) {
    RCLCPP_ERROR_STREAM(get_logger(), "open file: " << path << "failed");
    return;
  }

  file.write(data, size);
  file.close();
}

void TestDumpComponent::read_dmabuf_to_file(const std::string & path,
    std::shared_ptr<lib_mem_dmabuf::DmaBuffer> dmabuf)
{
  if (dmabuf == nullptr) {
    RCLCPP_ERROR(get_logger(), "dmabuf is null");
    return;
  }

  if (!dmabuf->map() || !dmabuf->sync_start()) {
    RCLCPP_INFO(get_logger(), "read from dmabuf failed");
    return;
  }

  save_data_to_file(path, (char *)dmabuf->addr(), dmabuf->size());
  if (!dmabuf->sync_end() || !dmabuf->unmap()) {
    RCLCPP_INFO(get_logger(), "read from dmabuf failed");
    return;
  }
}

void TestDumpComponent::dump_msg(const std::shared_ptr<type::Image> msg)
{
  RCLCPP_INFO(get_logger(), "dump qrb_ros::transport::type::Image message");
  read_dmabuf_to_file(dump_path_, msg->dmabuf);
  RCLCPP_INFO(get_logger(), "dump success");
  sub_.reset();
}

void TestDumpComponent::dump_msg(const std::shared_ptr<sensor_msgs::msg::Image> msg)
{
  RCLCPP_INFO(get_logger(), "dump sensor_msgs::msg::Image message");
  save_data_to_file(dump_path_, (char *)msg->data.data(), msg->data.size());
  RCLCPP_INFO(get_logger(), "dump success");
  sub_.reset();
}

void TestDumpComponent::dump_msg(const std::shared_ptr<type::Imu> msg)
{
  RCLCPP_INFO(get_logger(), "dump qrb_ros::transport::type::Imu message");
  std::ofstream file(dump_path_);
  if (!file.is_open()) {
    RCLCPP_ERROR_STREAM(get_logger(), "open file: " << dump_path_ << "failed");
    return;
  }

  if (msg->acceleration == nullptr || msg->gyro == nullptr) {
    RCLCPP_ERROR(get_logger(), "imu data is null");
    return;
  }

  std::stringstream line;
  line << "angular_velocity:\n"
       << "x: " << msg->gyro->gyro.x << "\ny: " << msg->gyro->gyro.y << "\nz: " << msg->gyro->gyro.z
       << "\nlinear_acceleration:\n"
       << "x: " << msg->acceleration->acceleration.x << "\ny: " << msg->acceleration->acceleration.y
       << "\nz: " << msg->acceleration->acceleration.z << "\n";

  file.write(line.str().c_str(), line.str().size());
  file.close();

  RCLCPP_INFO(get_logger(), "dump success");
  sub_.reset();
}

void TestDumpComponent::dump_msg(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
{
  RCLCPP_INFO(get_logger(), "dump sensor_msgs::msg::Imu message");

  std::ofstream file(dump_path_);
  if (!file.is_open()) {
    RCLCPP_ERROR_STREAM(get_logger(), "open file: " << dump_path_ << "failed");
    return;
  }

  std::stringstream line;
  line << "angular_velocity:\n"
       << "x: " << msg->angular_velocity.x << "\ny: " << msg->angular_velocity.y
       << "\nz: " << msg->angular_velocity.z << "\nlinear_acceleration:\n"
       << "x: " << msg->linear_acceleration.x << "\ny: " << msg->linear_acceleration.y
       << "\nz: " << msg->linear_acceleration.z << "\n";

  file.write(line.str().c_str(), line.str().size());
  file.close();

  RCLCPP_INFO(get_logger(), "dump success");
  sub_.reset();
}

}  // namespace qrb_ros::transport

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::transport::TestDumpComponent)
