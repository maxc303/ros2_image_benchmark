#ifndef IMAGE_LATENCY_BENCHMARK__IMAGE_SUB_TEST_NODE_HPP
#define IMAGE_LATENCY_BENCHMARK__IMAGE_SUB_TEST_NODE_HPP

#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
class ImageSubTestNode : public rclcpp::Node {
 public:
  explicit ImageSubTestNode();

 private:
  void check_latency(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  std::queue<long> latencies_;
  long latencies_sum_ = 0;

  int window_size_ = 10;

  std::string topic_name_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
};

#endif  // IMAGE_LATENCY_BENCHMARK__IMAGE_SUB_TEST_NODE_HPP
