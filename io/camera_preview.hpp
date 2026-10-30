#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>
#include <thread>

namespace io {
// Viewing only. One pending image, no network/encoding waits on the camera reader.
class CameraPreview {
public:
  explicit CameraPreview(const std::string & socket_path = "");
  ~CameraPreview();
  void submit(const cv::Mat & image, std::chrono::steady_clock::time_point timestamp,
              int conversion = -1) noexcept;
private:
  void run() noexcept;
  std::string socket_path_;
  std::atomic<bool> connected_{false}, stop_{false};
  std::mutex mutex_;
  std::condition_variable wake_;
  cv::Mat pending_;
  int conversion_ = -1;
  std::chrono::steady_clock::time_point timestamp_, next_{};
  std::thread worker_;
};
}
