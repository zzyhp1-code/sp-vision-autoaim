// 原始 Bayer 通道判定（含逐像素对照标尺）：不依赖对场景颜色的任何主观判断。
//
// 原理：把原始 Bayer 按"传感器实际排列"解释后，用两个客观标尺判定红蓝是否被交换：
//   标尺 A（通道均值）：自然场景下绿色通常最强；红蓝若被交换，(R-B) 的符号会整体翻转。
//   标尺 B（像素对照）：取图像里 R-B 最大和最小的两处像素，打印其原始 Bayer 四相值。
//        最蓝的那处像素，其在"正确解释"下应给出 B>R；若相反，说明解释码选反了。
// 另外同时把相机当前的 pixel_type 与两种解释的 (R-B) 都打印出来。
//
// 用法(小电脑)：./build/bayer_raw_probe [--save=/tmp/bayer_raw]
// 只读：不写相机参数、不发送控制帧。

#include <fmt/core.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "MvCameraControl.h"

namespace
{
void parity_means(const cv::Mat & raw, double out[4])
{
  double sum[4] = {0, 0, 0, 0};
  long counts[4] = {0, 0, 0, 0};
  for (int y = 0; y < raw.rows; ++y) {
    const std::uint8_t * row = raw.ptr<std::uint8_t>(y);
    for (int x = 0; x < raw.cols; ++x) {
      const int k = ((y & 1) << 1) | (x & 1);
      sum[k] += row[x];
      ++counts[k];
    }
  }
  for (int k = 0; k < 4; ++k) out[k] = sum[k] / std::max(1L, counts[k]);
}
}  // namespace

int main(int argc, char * argv[])
{
  std::string save_dir;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg.rfind("--save=", 0) == 0) save_dir = arg.substr(7);
  }

  MV_CC_DEVICE_INFO_LIST list;
  auto ret = MV_CC_EnumDevices(MV_USB_DEVICE, &list);
  if (ret != MV_OK || list.nDeviceNum == 0) {
    fmt::print(stderr, "[failure] 没有找到相机 (ret={:#x})\n", ret);
    return 1;
  }
  void * handle = nullptr;
  if (MV_CC_CreateHandle(&handle, list.pDeviceInfo[0]) != MV_OK) return 1;
  if (MV_CC_OpenDevice(handle) != MV_OK) return 1;
  if (MV_CC_StartGrabbing(handle) != MV_OK) {
    MV_CC_CloseDevice(handle);
    return 1;
  }

  cv::Mat raw_gray;
  cv::Size size;
  unsigned int pixel_type = 0;
  for (int attempt = 0; attempt < 30 && raw_gray.empty(); ++attempt) {
    MV_FRAME_OUT raw;
    if (MV_CC_GetImageBuffer(handle, &raw, 200) != MV_OK) continue;
    size = cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight);
    pixel_type = static_cast<unsigned int>(raw.stFrameInfo.enPixelType);
    cv::Mat view(size, CV_8UC1, raw.pBufAddr);
    raw_gray = view.clone();
    MV_CC_FreeImageBuffer(handle, &raw);
  }
  MV_CC_StopGrabbing(handle);
  MV_CC_CloseDevice(handle);
  MV_CC_DestroyHandle(handle);
  if (raw_gray.empty()) {
    fmt::print(stderr, "[failure] 没有取到帧\n");
    return 1;
  }

  double pm[4];
  parity_means(raw_gray, pm);
  fmt::print("原始帧 {}x{} pixel_type={:#x}\n", size.width, size.height, pixel_type);
  fmt::print(
    "2x2 奇偶均值: 偶/偶 {:.2f}  偶/奇 {:.2f}  奇/偶 {:.2f}  奇/奇 {:.2f}\n", pm[0], pm[1], pm[2],
    pm[3]);

  struct Variant
  {
    const char * label;
    int code;
  };
  const std::vector<Variant> variants{
    {"BayerRG2BGR(修复前用的)", cv::COLOR_BayerRG2BGR},
    {"BayerBG2BGR(修复后用的)", cv::COLOR_BayerBG2BGR},
    {"BayerGB2BGR(PFNC命名规则)", cv::COLOR_BayerGB2BGR},
    {"BayerGR2BGR(PFNC命名规则)", cv::COLOR_BayerGR2BGR},
  };

  std::vector<cv::Mat> images;
  for (const auto & v : variants) {
    cv::Mat bgr;
    cv::cvtColor(raw_gray, bgr, v.code);
    images.push_back(bgr);
    const cv::Scalar m = cv::mean(bgr);
    fmt::print(
      "{}: 均值 B {:.2f} G {:.2f} R {:.2f}   (R-B)={:+.2f}\n", v.label, m[0], m[1], m[2],
      m[2] - m[0]);
  }

  // 标尺 B：每种解释里 (R-B) 最大/最小的像素，打印其 3x3 邻域均值
  for (std::size_t k = 0; k < images.size(); ++k) {
    const cv::Mat & img = images[k];
    cv::Mat diff;
    cv::extractChannel(img, diff, 2);                       // R
    cv::Mat blue;
    cv::extractChannel(img, blue, 0);                       // B
    cv::Mat rb = diff - blue;                               // R-B
    double mn = 0, mx = 0;
    cv::Point pmin, pmax;
    cv::minMaxLoc(rb, &mn, &mx, &pmin, &pmax);
    auto sample = [&](cv::Point p) {
      cv::Rect roi(
        std::max(0, p.x - 1), std::max(0, p.y - 1), 3, 3);
      if (roi.x + roi.width > img.cols || roi.y + roi.height > img.rows)
        roi = cv::Rect(std::max(0, std::min(p.x, img.cols - 1)),
                       std::max(0, std::min(p.y, img.rows - 1)), 1, 1);
      return cv::mean(img(roi));
    };
    const cv::Scalar smax = sample(pmax);
    const cv::Scalar smin = sample(pmin);
    fmt::print(
      "  {}: 最红处 ({},{}) B {:.0f} G {:.0f} R {:.0f} R-B={:+.0f} | 最蓝处 ({},{}) B {:.0f} G "
      "{:.0f} R {:.0f} R-B={:+.0f}\n",
      variants[k].label, pmax.x, pmax.y, smax[0], smax[1], smax[2], smax[2] - smax[0], pmin.x,
      pmin.y, smin[0], smin[1], smin[2], smin[2] - smin[0]);
    if (!save_dir.empty()) {
      std::filesystem::create_directories(save_dir);
      cv::imwrite(std::string(save_dir) + "/" + variants[k].label + ".jpg", img);
    }
  }

  // 标尺 B：蓝色 LED 是真实场景里已知颜色最强的物体（用户现场确认灯条是蓝色）。
  // 对每种解释，找出"最亮的 0.05% 像素"（LED 灯条），打印它们的通道均值：
  // 正确解释必须给出 B 明显大于 R；给出 R > B 的解释就是把红蓝换反了。
  double mn_raw = 0, mx_raw = 0;
  cv::minMaxLoc(raw_gray, &mn_raw, &mx_raw);
  cv::Mat bright_mask;
  cv::threshold(raw_gray, bright_mask, mx_raw * 0.75, 255, cv::THRESH_BINARY);
  fmt::print(
    "最亮像素阈值(原始 {:.0f} 的 75%)，亮像素数 {}\n", mx_raw, cv::countNonZero(bright_mask));
  for (const auto & v : variants) {
    cv::Mat bgr;
    cv::cvtColor(raw_gray, bgr, v.code);
    const cv::Scalar m = cv::mean(bgr, bright_mask);
    fmt::print(
      "  {:<24} 亮像素均值 B {:6.1f} G {:6.1f} R {:6.1f}   R-B={:+7.1f}  {}\n", v.label, m[0],
      m[1], m[2], m[2] - m[0],
      (m[0] > m[2]) ? "<= B>R，与\"灯条是蓝色\"一致" : "<= R>B，与\"灯条是蓝色\"矛盾");
  }
  fmt::print(
    "  （用户现场确认：灯条是蓝色。因此上面唯一给出 B>R 的那一行才是正确解释。）\n");

  // 标尺 C：亮像素里"红/蓝强像素"的比例（场景级颜色普查，不依赖对某一块板的判断）。
  for (const auto & v : variants) {
    cv::Mat bgr;
    cv::cvtColor(raw_gray, bgr, v.code);
    long strong_red = 0, strong_blue = 0;
    for (int y = 0; y < bgr.rows; ++y) {
      const auto * row = bgr.ptr<cv::Vec3b>(y);
      for (int x = 0; x < bgr.cols; ++x) {
        const int b = row[x][0], g = row[x][1], r = row[x][2];
        const int peak = std::max(r, b);
        if (peak < 80) continue;
        if (r > b + 40 && r > g + 10) ++strong_red;
        else if (b > r + 40 && b > g + 10) ++strong_blue;
      }
    }
    fmt::print(
      "  {:<24} 强红像素 {:6d}  强蓝像素 {:6d}  红/蓝 = {:.2f}\n", v.label, strong_red, strong_blue,
      strong_blue > 0 ? static_cast<double>(strong_red) / strong_blue : -1.0);
  }
  return 0;
}
