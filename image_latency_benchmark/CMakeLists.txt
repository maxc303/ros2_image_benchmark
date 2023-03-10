cmake_minimum_required(VERSION 3.8)
project(image_latency_benchmark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include>)


add_library(pub_sub_test_nodes 
  SHARED 
  src/image_sub_test_node.cpp 
  src/image_pub_test_node.cpp)
ament_target_dependencies(pub_sub_test_nodes rclcpp sensor_msgs rclcpp_components)

# Register components for composite tests
# Refs:
# https://docs.ros.org/en/foxy/Concepts/About-Composition.html#writing-a-component
# https://roscon.ros.org/2019/talks/roscon2019_composablenodes.pdf
rclcpp_components_register_nodes(
  pub_sub_test_nodes 
  "image_latency_benchmark::ImagePubTestNode" "image_latency_benchmark::ImageSubTestNode"
)
install(TARGETS pub_sub_test_nodes)


add_executable(test_image_pub src/test_image_publish_main.cpp)
target_link_libraries(test_image_pub pub_sub_test_nodes)

add_executable(test_image_sub src/test_image_subscribe_main.cpp)
target_link_libraries(test_image_sub pub_sub_test_nodes)

add_executable(test_image_pub_sub src/test_image_pub_sub_main.cpp)
target_link_libraries(test_image_pub_sub pub_sub_test_nodes)

find_package(OpenCV REQUIRED)
add_executable(test_yuv_bgr_conversion src/test_yuv_bgr_conversion.cpp)
target_link_libraries(test_yuv_bgr_conversion ${OpenCV_LIBS})
add_executable(test_cv_imencode src/test_cv_imencode.cpp)
target_link_libraries(test_cv_imencode ${OpenCV_LIBS})

install(TARGETS 
    test_image_pub 
    test_image_sub 
    test_image_pub_sub 
    test_yuv_bgr_conversion
    test_cv_imencode
  DESTINATION 
    lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
