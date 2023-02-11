#include <chrono>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/quality.hpp>

/**
 * @brief Run bgr <-> yuv420p conversions and report the average latency.
 *
 * @param image_bgr test image
 * @param num_tests
 */
void test_bgr_yuv_conversion(const cv::Mat& image_bgr, int num_tests = 100) {
  auto img_size = image_bgr.size();
  std::cout << "Test BGR<->YUV conversion with w=" << img_size.width
            << ", h=" << img_size.height << std::endl;

  cv::Mat image_iyuv;
  cv::Mat image_bgr_from_yuv;

  // BGR2YUV
  auto t_bgr2yuv_start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < num_tests; i++) {
    cv::cvtColor(image_bgr, image_iyuv, cv::COLOR_BGR2YUV_I420);
  }
  auto t_bgr2yuv_end = std::chrono::high_resolution_clock::now();

  // YUV2BGR
  auto t_yuv2bgr_start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < num_tests; i++) {
    cv::cvtColor(image_iyuv, image_bgr_from_yuv, cv::COLOR_YUV2BGR_I420);
  }
  auto t_yuv2bgr_end = std::chrono::high_resolution_clock::now();

  auto bgr2yuv_latency_ms =
      static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
                              t_bgr2yuv_end - t_bgr2yuv_start)
                              .count());
  auto yuv2bgr_latency_ms =
      static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
                              t_yuv2bgr_end - t_yuv2bgr_start)
                              .count());

  bgr2yuv_latency_ms /= num_tests;
  yuv2bgr_latency_ms /= num_tests;
  std::cout << " Tested conversion on " << num_tests << " images." << std::endl;
  std::cout << " - BGR to YUV_I420 time = " << bgr2yuv_latency_ms << "ms."
            << std::endl;
  std::cout << " - YUV_I420 to BGR time = " << yuv2bgr_latency_ms << "ms."
            << std::endl;
}

int main(int argc, char** argv) {
  if (argc > 1) {
    std::filesystem::path image_path = argv[1];
    std::cout << "Running test on test image: " << image_path << std::endl;
    auto test_image = cv::imread(image_path);
    if (test_image.empty()) {
      std::cerr << "Failed to load test image." << std::endl;
    }
    test_bgr_yuv_conversion(test_image);
  } else {
    std::cout << "No test image specified. Test conversion on grey image.\n";
    cv::Mat grey_image_bgr_fhd(1920, 1080, CV_8UC3, cv::Scalar(128, 128, 128));
    test_bgr_yuv_conversion(grey_image_bgr_fhd);
    cv::Mat grey_image_bgr_4k(3840, 2160, CV_8UC3, cv::Scalar(128, 128, 128));
    test_bgr_yuv_conversion(grey_image_bgr_4k);
  }
}