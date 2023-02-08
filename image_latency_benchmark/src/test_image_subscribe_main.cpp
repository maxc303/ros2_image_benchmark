#include <rclcpp/rclcpp.hpp>

#include "image_latency_benchmark/image_sub_test_node.hpp"
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  rclcpp::spin(
      std::make_shared<image_latency_benchmark::ImageSubTestNode>(options));
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
