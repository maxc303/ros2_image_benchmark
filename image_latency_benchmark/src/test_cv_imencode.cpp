#include <chrono>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/quality.hpp>

void test_imencode(const cv::Mat &image_bgr, int num_tests = 10) {
  auto img_size = image_bgr.size();

  double img_size_MB =
      static_cast<double>(img_size.width * img_size.height * 3) / 1'000'000.0;
  std::cout << "Test OpenCV imencode conversion with w=" << img_size.width
            << ", h=" << img_size.height
            << ", bgr8 raw data size=" << img_size_MB << " MB" << std::endl;

  std::vector<uchar> compressed_image;

  // Test JPG compression
  for (int quality_level = 10; quality_level <= 100; quality_level += 10) {
    auto t_jpeg_start = std::chrono::high_resolution_clock::now();
    std::vector<int> params = {static_cast<int>(cv::IMWRITE_JPEG_QUALITY),
                               quality_level};
    for (int i = 0; i < num_tests; i++) {
      cv::imencode(".jpg", image_bgr, compressed_image, params);
    }
    auto t_jpeg_end = std::chrono::high_resolution_clock::now();
    double compressed_size_MB =
        static_cast<double>(compressed_image.size()) / 1'000'000.0;

    auto jpg_latency_ms = static_cast<double>(
        std::chrono::duration_cast<std::chrono::milliseconds>(t_jpeg_end -
                                                              t_jpeg_start)
            .count());
    jpg_latency_ms /= num_tests;
    std::cout << "JPG Quality level=" << params[1]
              << ", compressed data size=" << compressed_size_MB
              << " MB, compression rate=" << compressed_size_MB / img_size_MB
              << ", avg time=" << jpg_latency_ms << "ms.\n";
  }

  // Test PNG compression
  for (int compression = 0; compression <= 9; compression++) {
    auto t_png_start = std::chrono::high_resolution_clock::now();
    std::vector<int> params = {static_cast<int>(cv::IMWRITE_PNG_COMPRESSION),
                               compression};
    for (int i = 0; i < num_tests; i++) {
      cv::imencode(".png", image_bgr, compressed_image, params);
    }
    auto t_png_end = std::chrono::high_resolution_clock::now();
    double compressed_size_MB =
        static_cast<double>(compressed_image.size()) / 1'000'000.0;
    auto png_latency_ms = static_cast<double>(
        std::chrono::duration_cast<std::chrono::milliseconds>(t_png_end -
                                                              t_png_start)
            .count());
    png_latency_ms /= num_tests;
    std::cout << "PNG Compression level=" << params[1]
              << ", compressed data size=" << compressed_size_MB
              << " MB, compression rate=" << compressed_size_MB / img_size_MB
              << ", avg time=" << png_latency_ms << "ms.\n";
  }
}

int main(int argc, char **argv) {
  if (argc > 1) {
    std::filesystem::path image_path = argv[1];
    std::cout << "Running test on test image: " << image_path << std::endl;
    auto test_image = cv::imread(image_path);
    if (test_image.empty()) {
      std::cerr << "Failed to load test image." << std::endl;
    }
    test_imencode(test_image);
  } else {
    std::cout << "No test image specified \n";
    std::cout << "Usage: [" << argv[0] << " <test_image_path>] \n";
  }
}