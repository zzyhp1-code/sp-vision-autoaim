#pragma once
#include <opencv2/core.hpp>
#include <algorithm>
#include <stdexcept>

namespace io {
struct CameraView {
  cv::Rect capture;
  cv::Rect algorithm; // relative to the captured sensor region
};
inline CameraView camera_view(int width, int height, int x, int y,
                             int preview_width, int preview_height, int increment = 8) {
  if (width <= 0 || height <= 0 || x < 0 || y < 0 || increment <= 0 ||
      preview_width < width || preview_height < height)
    throw std::invalid_argument("Preview must contain the calibrated camera region");
  const int cx = std::max(0, x - (preview_width-width)/2) / increment * increment;
  const int cy = std::max(0, y - (preview_height-height)/2) / increment * increment;
  CameraView result{{cx, cy, preview_width, preview_height}, {x-cx, y-cy, width, height}};
  if ((result.algorithm & cv::Rect(0,0,preview_width,preview_height)) != result.algorithm ||
      (result.algorithm.x & 1) || (result.algorithm.y & 1))
    throw std::invalid_argument("Invalid calibrated crop or Bayer parity");
  return result;
}
// Own the crop so Bayer border interpolation sees precisely the old image boundary.
inline cv::Mat algorithm_camera_raw(const cv::Mat & raw, const CameraView & view) {
  if (raw.size() != view.capture.size())
    throw std::runtime_error("Camera capture size differs from the requested view");
  return raw(view.algorithm).clone();
}
}
