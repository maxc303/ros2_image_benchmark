#include <rclcpp/rclcpp.hpp>

#include "image_latency_benchmark/image_pub_test_node.hpp"
#include "image_latency_benchmark/image_sub_test_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto pub_node = std::make_shared<ImagePubTestNode>();
  auto sub_node = std::make_shared<ImageSubTestNode>();

  executor.add_node(pub_node);
  executor.add_node(sub_node);

  executor.spin();
  return EXIT_SUCCESS;
}
