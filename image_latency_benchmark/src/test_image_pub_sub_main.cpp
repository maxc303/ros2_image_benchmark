#include <rclcpp/rclcpp.hpp>

#include "image_latency_benchmark/image_pub_test_node.hpp"
#include "image_latency_benchmark/image_sub_test_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto pub_node =
      std::make_shared<image_latency_benchmark::ImagePubTestNode>(options);
  auto sub_node =
      std::make_shared<image_latency_benchmark::ImageSubTestNode>(options);

  executor.add_node(pub_node);
  executor.add_node(sub_node);

  executor.spin();
  return EXIT_SUCCESS;
}
