#include "image_latency_benchmark/image_pub_test_node.hpp"

ImagePubTestNode::ImagePubTestNode() : Node("image_pub_test_node") {
  width_ = this->declare_parameter<int>("width", 1920);
  height_ = this->declare_parameter<int>("height", 1080);
  pub_freq_hz_ = this->declare_parameter<int>("pub_freq_hz", 10);
  num_channels_ = this->declare_parameter<int>("num_channls", 3);
  topic_name_ =
      this->declare_parameter<std::string>("topic_name", "pub_test_image");

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
};

void ImagePubTestNode::publish_frame() {
  auto t_now =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();

  image_.header.stamp.sec = static_cast<int>(t_now / 1'000'000'000);
  image_.header.stamp.nanosec = static_cast<int>(t_now % 1'000'000'000);

  publisher_->publish(image_);
}