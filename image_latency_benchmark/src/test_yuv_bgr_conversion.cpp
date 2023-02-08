#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/quality.hpp>
void test_bgr_yuv_conversion(int width, int height, int num_tests = 100) {
  cv::Mat image_bgr(height, width, CV_8UC3, cv::Scalar(128, 128, 128));
  std::cout << "Test BGR<->YUV conversion with w=" << width << ", h=" << height
            << std::endl;

  cv::randu(image_bgr, cv::Scalar(100, 100, 100), cv::Scalar(200, 200, 200));

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

  std::cout << " - BGR to YUV_I420 time = " << bgr2yuv_latency_ms << "ms."
            << std::endl;
  std::cout << " - YUV_I420 to BGR time = " << yuv2bgr_latency_ms << "ms."
            << std::endl;
}

int main() {
  test_bgr_yuv_conversion(1920, 1080);
  test_bgr_yuv_conversion(3840, 2160);
}