#ifndef IMAGE_LATENCY_BENCHMARK__IMAGE_PUB_TEST_NODE_HPP
#define IMAGE_LATENCY_BENCHMARK__IMAGE_PUB_TEST_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
namespace image_latency_benchmark {

class ImagePubTestNode : public rclcpp::Node {
 public:
  explicit ImagePubTestNode(const rclcpp::NodeOptions& options);

 private:
  void publish_frame();

  int width_;
  int height_;
  int pub_freq_hz_;
  std::string topic_name_;
  float num_channels_ = 3;
  std::string encoding_ = "bgr8";
  sensor_msgs::msg::Image image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
#endif  // IMAGE_LATENCY_BENCHMARK__IMAGE_PUB_TEST_NODE_HPP
