#include <rclcpp/rclcpp.hpp>

#include "image_latency_benchmark/image_pub_test_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePubTestNode>());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
