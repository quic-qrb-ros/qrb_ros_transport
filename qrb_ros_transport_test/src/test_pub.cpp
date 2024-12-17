// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <fstream>

#include "qrb_ros_transport_image_type/image.hpp"
#include "qrb_ros_transport_imu_type/imu.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace qrb_ros::transport
{

class TestPubComponent : public rclcpp::Node
{
public:
  explicit TestPubComponent(const rclcpp::NodeOptions & options);
  ~TestPubComponent();

private:
  std::unique_ptr<char[]> read_data_from_file(const std::string & path, std::size_t size);

  void publish_image(int width,
      int height,
      const std::string & encoding,
      int fps,
      const std::string & data_file);

  void publish_ros_image(int width,
      int height,
      const std::string & encoding,
      int fps,
      const std::string & data_file);

  void publish_imu(int fps);
  void publish_ros_imu(int fps);

  rclcpp::Publisher<type::Image>::SharedPtr image_pub_{ nullptr };
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_image_pub_{ nullptr };
  rclcpp::Publisher<type::Imu>::SharedPtr imu_pub_{ nullptr };
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ros_imu_pub_{ nullptr };

  std::shared_ptr<std::thread> pub_thread_{ nullptr };
};

TestPubComponent::TestPubComponent(const rclcpp::NodeOptions & options)
  : Node("TestPubComponent", options)
{
  auto input_file = this->declare_parameter("input", "/data/src.yuv");
  auto width = this->declare_parameter("width", 1920);
  auto height = this->declare_parameter("height", 1080);
  auto encoding = this->declare_parameter("encoding", "nv12");
  auto message_type = this->declare_parameter("test_type", "qrb_ros::transport::type::Image");
  auto topic_name = this->declare_parameter("topic_name", "image");
  auto fps = this->declare_parameter("fps", 30);

  RCLCPP_INFO(get_logger(),
      "=== Publish type: %s, width: %ld, height: %ld, encoding: %s, input: %s, topic_name: %s, "
      "fps: %ld",
      message_type.c_str(), width, height, encoding.c_str(), input_file.c_str(), topic_name.c_str(),
      fps);

  if (message_type == "qrb_ros::transport::type::Image") {
    image_pub_ = this->create_publisher<type::Image>(topic_name, 30);
    pub_thread_ = std::make_shared<std::thread>(
        &TestPubComponent::publish_image, this, width, height, encoding, fps, input_file);
  } else if (message_type == "sensor_msgs::msg::Image") {
    ros_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 30);
    pub_thread_ = std::make_shared<std::thread>(
        &TestPubComponent::publish_ros_image, this, width, height, encoding, fps, input_file);
  } else if (message_type == "qrb_ros::transport::type::Imu") {
    imu_pub_ = this->create_publisher<type::Imu>(topic_name, 30);
    pub_thread_ = std::make_shared<std::thread>(&TestPubComponent::publish_imu, this, fps);
  } else if (message_type == "sensor_msgs::msg::Imu") {
    ros_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_name, 30);
    pub_thread_ = std::make_shared<std::thread>(&TestPubComponent::publish_ros_imu, this, fps);
  }
}

TestPubComponent::~TestPubComponent()
{
  if (pub_thread_->joinable()) {
    pub_thread_->join();
  }
}

void TestPubComponent::publish_image(int width,
    int height,
    const std::string & encoding,
    int fps,
    const std::string & data_file)
{
  auto data_size = 0;
  auto src_step = 0;
  if (encoding == "nv12") {
    data_size = width * height * 1.5;
    src_step = width;
  } else if (encoding == "rgb8") {
    data_size = width * height * 3;
    src_step = width * 3;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "encoding: " << encoding << " not support");
    return;
  }

  auto data = read_data_from_file(data_file, data_size);
  if (data == nullptr) {
    return;
  }

  rclcpp::Rate rate(fps);
  while (rclcpp::ok()) {
    auto msg = std::make_unique<type::Image>();
    msg->width = width;
    msg->height = height;
    msg->encoding = encoding;

    auto buffer_size = image_utils::get_image_align_size(width, height, encoding);

    auto dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(buffer_size, "/dev/dma_heap/system");
    if (dmabuf == nullptr) {
      RCLCPP_ERROR_STREAM(get_logger(), "dmabuf alloc failed, size: " << buffer_size);
      return;
    }

    if (!image_utils::save_image_to_dmabuf(
            dmabuf, data.get(), width, height, src_step, encoding, true)) {
      RCLCPP_ERROR(get_logger(), "save image to dmabuf failed");
      return;
    }
    msg->dmabuf = dmabuf;
    msg->header.stamp = this->now();

    image_pub_->publish(std::move(msg));

    rate.sleep();
  }
}

void TestPubComponent::publish_ros_image(int width,
    int height,
    const std::string & encoding,
    int fps,
    const std::string & data_file)
{
  auto data_size = 0;
  auto src_step = 0;
  if (encoding == "nv12") {
    data_size = width * height * 1.5;
    src_step = width;
  } else if (encoding == "rgb8") {
    data_size = width * height * 3;
    src_step = width * 3;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "encoding: " << encoding << " not support");
    return;
  }

  auto data = read_data_from_file(data_file, data_size);
  if (data == nullptr) {
    return;
  }

  rclcpp::Rate rate(fps);
  while (rclcpp::ok()) {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->width = width;
    msg->height = height;
    msg->encoding = encoding;
    msg->step = src_step;

    msg->data.resize(data_size);
    std::memcpy(msg->data.data(), data.get(), data_size);

    msg->header.stamp = this->now();
    ros_image_pub_->publish(std::move(msg));

    rate.sleep();
  }
}

std::unique_ptr<char[]> TestPubComponent::read_data_from_file(const std::string & path,
    std::size_t size)
{
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    RCLCPP_ERROR_STREAM(get_logger(), "open file: " << path << "failed");
    return nullptr;
  }

  auto data = std::make_unique<char[]>(size);
  auto read_size = file.readsome(data.get(), size);

  if (read_size != (long)size) {
    RCLCPP_ERROR_STREAM(get_logger(), "read file size: " << read_size << " != " << size);
    file.close();
    return nullptr;
  }

  file.close();
  return data;
}

void TestPubComponent::publish_imu(int fps)
{
  rclcpp::Rate rate(fps);
  int i = 1;
  while (rclcpp::ok()) {
    auto msg = std::make_unique<type::Imu>();
    msg->acceleration = std::make_shared<sensors_event_t>();
    msg->gyro = std::make_shared<sensors_event_t>();

    msg->acceleration->acceleration.x = i;
    msg->acceleration->acceleration.y = i;
    msg->acceleration->acceleration.z = i;
    msg->gyro->gyro.x = i;
    msg->gyro->gyro.y = i;
    msg->gyro->gyro.z = i;

    msg->header.stamp = this->now();
    imu_pub_->publish(std::move(msg));

    i++;
    if (i == fps) {
      i = 0;
    }
    rate.sleep();
  }
}

void TestPubComponent::publish_ros_imu(int fps)
{
  rclcpp::Rate rate(fps);
  int i = 1;
  while (rclcpp::ok()) {
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    msg->linear_acceleration.x = i;
    msg->linear_acceleration.y = i;
    msg->linear_acceleration.z = i;

    msg->angular_velocity.x = i;
    msg->angular_velocity.y = i;
    msg->angular_velocity.z = i;

    msg->header.stamp = this->now();
    ros_imu_pub_->publish(std::move(msg));

    i++;
    if (i == fps) {
      i = 0;
    }
    rate.sleep();
  }
}

}  // namespace qrb_ros::transport

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::transport::TestPubComponent)
