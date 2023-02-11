#include "image_latency_benchmark/image_pub_test_node.hpp"

#include <sstream>
namespace image_latency_benchmark {

ImagePubTestNode::ImagePubTestNode(const rclcpp::NodeOptions& options)
    : Node("image_pub_test_node", options) {
  width_ = this->declare_parameter<int>("width", 3840);
  height_ = this->declare_parameter<int>("height", 2160);
  pub_freq_hz_ = this->declare_parameter<int>("pub_freq_hz", 10);
  num_channels_ = this->declare_parameter<int>("num_channels", 3);
  topic_name_ =
      this->declare_parameter<std::string>("topic_name", "test_image/image");

  double callback_interval = 1.0 / static_cast<double>(pub_freq_hz_);

  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(callback_interval),
      std::bind(&ImagePubTestNode::publish_frame, this));

  publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      topic_name_, rclcpp::SensorDataQoS());

  // Create dummy bgr image
  image_.encoding = encoding_;
  image_.header.frame_id = "map";
  image_.height = height_;
  image_.width = width_;
  image_.is_bigendian = true;
  image_.step = static_cast<int>(num_channels_ * width_);
  image_.data = std::vector<uint8_t>(
      static_cast<int>(width_ * height_ * num_channels_), 128);

  double data_size_megabytes =
      static_cast<double>(image_.data.size()) / 1'000'000.0;
  std::stringstream oss;
  oss << "Initialzied publisher for topic: [" << topic_name_ << "] at "
      << pub_freq_hz_ << "Hz, data size=" << data_size_megabytes << "MB.";

  RCLCPP_INFO_STREAM(this->get_logger(), oss.str());
};

void ImagePubTestNode::publish_frame() {
  msg_ptr_ = std::unique_ptr<sensor_msgs::msg::Image>(
      new sensor_msgs::msg::Image(image_));

  std::stringstream msg_address;
  msg_address << msg_ptr_.get();

  // Use header.frame id to store message pointer address
  msg_ptr_->header.frame_id = msg_address.str();

  auto t_now =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  msg_ptr_->header.stamp.sec = static_cast<int>(t_now / 1'000'000'000);
  msg_ptr_->header.stamp.nanosec = static_cast<int>(t_now % 1'000'000'000);
  publisher_->publish(std::move(msg_ptr_));
}
}  // namespace image_latency_benchmark
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(image_latency_benchmark::ImagePubTestNode);