#include "image_latency_benchmark/image_sub_test_node.hpp"

#include <iostream>
#include <sstream>
ImageSubTestNode::ImageSubTestNode() : Node("image_sub_test_node") {
  topic_name_ =
      this->declare_parameter<std::string>("topic_name", "test_image/image");
  window_size_ =
      static_cast<int>(this->declare_parameter<int>("window_size", 10));
  subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name_, rclcpp::SensorDataQoS(),
      std::bind(&ImageSubTestNode::check_latency, this, std::placeholders::_1));

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Initialzied image sub test node for topic: [" << topic_name_ << "].");
}

void ImageSubTestNode::check_latency(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  auto t_now =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();

  auto t_pub =
      std::chrono::nanoseconds{static_cast<uint64_t>(msg->header.stamp.sec) *
                                   1'000'000'000 +
                               static_cast<uint64_t>(msg->header.stamp.nanosec)}
          .count();

  auto pub_sub_latency = (t_now - t_pub);

  // Get the running average
  latencies_.push(pub_sub_latency);
  latencies_sum_ += pub_sub_latency;
  if (static_cast<int>(latencies_.size()) > window_size_) {
    latencies_sum_ -= latencies_.front();
    latencies_.pop();
  }

  auto avg_lat_ns = latencies_sum_ / latencies_.size();
  auto avg_lat_ms = static_cast<double>(avg_lat_ns) / 1'000'000.0;

  std::stringstream oss;
  oss << "Average Sub->Pub latency of last [" << latencies_.size()
      << "] msgs is " << avg_lat_ms << " ms.";

  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              oss.str());
}