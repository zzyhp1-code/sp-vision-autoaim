#include "camera_preview.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

namespace io {
namespace {
std::string default_socket() {
  if (const char * path = std::getenv("SP_VISION_FPV_SOCKET")) return path;
  const auto uid = std::to_string(getuid());
  const auto runtime = "/run/user/" + uid;
  return (std::filesystem::is_directory(runtime) ? runtime : "/tmp") +
    "/sp-vision-fpv-" + uid + "/camera.sock";
}
bool send_bytes(int fd, const void * data, size_t size) {
  auto p = static_cast<const unsigned char *>(data);
  // Full-resolution BMP is ~5 MiB. Allow the receiver's cold encoder startup;
  // only this worker waits, while submit keeps replacing a single pending image.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
  while (size && std::chrono::steady_clock::now() < deadline) {
    const auto n = send(fd, p, size, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (n > 0) { p += n; size -= n; continue; }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      pollfd event{fd, POLLOUT, 0};
      poll(&event, 1, 5);
      continue;
    }
    return false;
  }
  return size == 0;
}
}
CameraPreview::CameraPreview(const std::string & path)
: socket_path_(path.empty() ? default_socket() : path), worker_(&CameraPreview::run, this) {}

CameraPreview::~CameraPreview() {
  stop_ = true;
  wake_.notify_one();
  worker_.join();
}

void CameraPreview::submit(const cv::Mat & image,
                          std::chrono::steady_clock::time_point timestamp, int conversion) noexcept {
  if (!connected_ || image.empty() ||
      (image.type() != CV_8UC3 && image.type() != CV_8UC1)) return;
  try {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    const auto now = std::chrono::steady_clock::now();
    if (!lock || now < next_) return;
    next_ = now + std::chrono::milliseconds(33);
    pending_ = image.clone();  // independent of later drawing/reuse of the camera buffer
    timestamp_ = timestamp;
    conversion_ = conversion;
    wake_.notify_one();
  } catch (...) { /* Preview allocation failure must not interrupt capture. */ }
}

void CameraPreview::run() noexcept {
  int fd = -1;
  while (!stop_) {
    try {
      if (fd < 0) {
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        if (socket_path_.size() >= sizeof(address.sun_path)) return;
        std::strcpy(address.sun_path, socket_path_.c_str());
        fd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd < 0 || connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address))) {
          if (fd >= 0) close(fd);
          fd = -1;
          std::unique_lock<std::mutex> lock(mutex_);
          wake_.wait_for(lock, std::chrono::milliseconds(500), [&] { return stop_.load(); });
          continue;
        }
        connected_ = true;
      }
      cv::Mat image;
      int conversion;
      std::chrono::steady_clock::time_point timestamp;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_for(lock, std::chrono::milliseconds(100), [&] { return stop_ || !pending_.empty(); });
        if (stop_) break;
        if (pending_.empty()) continue;
        image = std::move(pending_);
        timestamp = timestamp_;
        conversion = conversion_;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now < timestamp || now - timestamp > std::chrono::milliseconds(500)) continue;
      if (conversion >= 0) {
        cv::Mat converted;
        cv::cvtColor(image, converted, conversion);
        image = std::move(converted);
      }
      if (image.type() != CV_8UC3) continue;
      const double scale = std::min({1.0, 2048.0 / image.cols, 1536.0 / image.rows});
      cv::Mat small;
      const cv::Size size(std::max(2, int(image.cols * scale) / 2 * 2),
                          std::max(2, int(image.rows * scale) / 2 * 2));
      if (image.size() == size) small = image;
      else cv::resize(image, small, size, 0, 0, cv::INTER_AREA);
      // Local IPC is lossless: do not quantize dark pixels before the viewing lift.
      // Uncompressed BMP avoids JPEG artifacts and PNG compression CPU cost.
      std::vector<unsigned char> bitmap;
      if (!cv::imencode(".bmp", small, bitmap)) continue;
      unsigned char header[16] = {'S', 'P', 'F', '2'};
      const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count();
      for (int i = 0; i < 8; ++i) header[4+i] = (uint64_t(ns) >> (8*i)) & 255;
      for (int i = 0; i < 4; ++i) header[12+i] = (uint64_t(bitmap.size()) >> (8*i)) & 255;
      if (!send_bytes(fd, header, sizeof(header)) || !send_bytes(fd, bitmap.data(), bitmap.size())) {
        connected_ = false;
        close(fd);
        fd = -1;
      }
    } catch (...) {
      connected_ = false;
      if (fd >= 0) close(fd);
      fd = -1;
    }
  }
  connected_ = false;
  if (fd >= 0) close(fd);
}
}
