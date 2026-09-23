#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>
#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(const std::string & config_path)
: exposure_us_(0.0),
  gain_(0.0),
  frame_rate_(150.0),
  width_(0),
  height_(0),
  white_balance_auto_(MV_BALANCEWHITE_AUTO_OFF),
  trigger_enable_(false),
  trigger_source_(MV_TRIGGER_SOURCE_LINE0),
  trigger_activation_(0),
  daemon_quit_(false),
  handle_(nullptr),
  device_open_(false),
  grabbing_(false),
  capturing_(false),
  capture_quit_(false),
  queue_(1),
  vid_(-1),
  pid_(-1)
{
  auto yaml = YAML::LoadFile(config_path);
  exposure_us_ = yaml["exposure_ms"].as<double>() * 1e3;
  gain_ = yaml["gain"].as<double>();
  const auto vid_pid = yaml["vid_pid"].as<std::string>();
  if (yaml["camera_fps"]) frame_rate_ = yaml["camera_fps"].as<double>();
  if (yaml["camera_width"]) width_ = yaml["camera_width"].as<int64_t>();
  if (yaml["camera_height"]) height_ = yaml["camera_height"].as<int64_t>();
  if (yaml["hikrobot_pixel_format"]) {
    pixel_format_ = yaml["hikrobot_pixel_format"].as<std::string>();
  }
  if (yaml["white_balance_auto"]) {
    const auto mode = yaml["white_balance_auto"].as<std::string>();
    if (mode == "once") {
      white_balance_auto_ = MV_BALANCEWHITE_AUTO_ONCE;
    } else if (mode == "continuous") {
      white_balance_auto_ = MV_BALANCEWHITE_AUTO_CONTINUOUS;
    } else if (mode != "off") {
      tools::logger()->warn(
        "Unknown white_balance_auto='{}'; using stable mode 'off'", mode);
    }
  }
  if (yaml["trigger_enable"]) trigger_enable_ = yaml["trigger_enable"].as<bool>();
  if (yaml["trigger_source"]) {
    trigger_source_ = yaml["trigger_source"].as<unsigned int>();
  }
  if (yaml["trigger_activation"]) {
    trigger_activation_ = yaml["trigger_activation"].as<unsigned int>();
  }

  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");

    capture_start();

    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);

      if (capturing_) continue;

      capture_stop();
      reset_usb();
      capture_start();
    }

    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void HikRobot::capture_start()
{
  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    return;
  }
  device_open_ = true;

  // Leave any persisted trigger mode before changing image-format nodes.
  set_enum_value("TriggerMode", MV_TRIGGER_MODE_OFF);
  if (width_ > 0 || height_ > 0) {
    set_int_value("OffsetX", 0);
    set_int_value("OffsetY", 0);
  }
  if (width_ > 0) set_int_value("Width", width_);
  if (height_ > 0) set_int_value("Height", height_);
  if (!pixel_format_.empty()) set_enum_value("PixelFormat", pixel_format_);

  set_enum_value("BalanceWhiteAuto", white_balance_auto_);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);

  if (trigger_enable_) {
    set_enum_value("TriggerSource", trigger_source_);
    set_enum_value("TriggerActivation", trigger_activation_);
    set_enum_value("TriggerMode", MV_TRIGGER_MODE_ON);
  } else {
    set_enum_value("TriggerMode", MV_TRIGGER_MODE_OFF);
    set_bool_value("AcquisitionFrameRateEnable", true);
    const auto fps_ret = MV_CC_SetFrameRate(handle_, static_cast<float>(frame_rate_));
    if (fps_ret != MV_OK) {
      tools::logger()->warn("MV_CC_SetFrameRate({}) failed: {:#x}", frame_rate_, fps_ret);
    }
  }

  log_camera_state();

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }
  grabbing_ = true;

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");

    capturing_ = true;

    MV_FRAME_OUT raw;

    while (!capture_quit_) {
      std::this_thread::sleep_for(1ms);

      unsigned int ret;
      unsigned int nMsec = 100;

      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();
      static bool logged_frame_format = false;
      if (!logged_frame_format) {
        tools::logger()->info(
          "[HikRobot] first frame: {}x{}, pixel_type={:#x}, frame_len={}",
          raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight,
          static_cast<unsigned int>(raw.stFrameInfo.enPixelType), raw.stFrameInfo.nFrameLen);
        logged_frame_format = true;
      }
      const cv::Size frame_size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight);
      cv::Mat rgb_img;

      // MVS 4.5 returns an error from MV_CC_ConvertPixelType for BayerRG8.
      // OpenCV 4.5 has only the ambiguous two-letter Bayer aliases. PFNC BayerRG8
      // is the four-letter RGGB pattern; RGGB->RGB is aliased to BayerRG2BGR.
      // The apparently opposite suffix is intentional. Keep the result in RGB.
      int bayer_to_rgb = -1;
      switch (raw.stFrameInfo.enPixelType) {
        case PixelType_Gvsp_BayerGR8: bayer_to_rgb = cv::COLOR_BayerGR2BGR; break;
        case PixelType_Gvsp_BayerRG8: bayer_to_rgb = cv::COLOR_BayerRG2BGR; break;
        case PixelType_Gvsp_BayerGB8: bayer_to_rgb = cv::COLOR_BayerGB2BGR; break;
        case PixelType_Gvsp_BayerBG8: bayer_to_rgb = cv::COLOR_BayerBG2BGR; break;
        default: break;
      }

      if (bayer_to_rgb >= 0) {
        cv::Mat raw_img(frame_size, CV_8UC1, raw.pBufAddr);
        cv::cvtColor(raw_img, rgb_img, bayer_to_rgb);
      } else if (raw.stFrameInfo.enPixelType == PixelType_Gvsp_Mono8) {
        cv::Mat raw_img(frame_size, CV_8UC1, raw.pBufAddr);
        cv::cvtColor(raw_img, rgb_img, cv::COLOR_GRAY2RGB);
      } else if (raw.stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed) {
        cv::Mat raw_img(frame_size, CV_8UC3, raw.pBufAddr);
        rgb_img = raw_img.clone();
      } else {
        tools::logger()->warn(
          "Unsupported HikRobot pixel type: {:#x}",
          static_cast<unsigned int>(raw.stFrameInfo.enPixelType));
        MV_CC_FreeImageBuffer(handle_, &raw);
        continue;
      }

      queue_.push({rgb_img, timestamp});

      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  if (handle_ == nullptr) return;

  if (grabbing_) {
    const auto ret = MV_CC_StopGrabbing(handle_);
    if (ret != MV_OK) tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
    grabbing_ = false;
  }

  if (device_open_) {
    const auto ret = MV_CC_CloseDevice(handle_);
    if (ret != MV_OK) tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
    device_open_ = false;
  }

  const auto ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
  handle_ = nullptr;
}

bool HikRobot::set_float_value(const std::string & name, double value)
{
  const auto ret = MV_CC_SetFloatValue(handle_, name.c_str(), static_cast<float>(value));
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return false;
  }
  return true;
}

bool HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  const auto ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return false;
  }
  return true;
}

bool HikRobot::set_enum_value(const std::string & name, const std::string & value)
{
  const auto ret = MV_CC_SetEnumValueByString(handle_, name.c_str(), value.c_str());
  if (ret != MV_OK) {
    tools::logger()->warn(
      "MV_CC_SetEnumValueByString(\"{}\", \"{}\") failed: {:#x}", name, value, ret);
    return false;
  }
  return true;
}

bool HikRobot::set_int_value(const std::string & name, int64_t value)
{
  const auto ret = MV_CC_SetIntValueEx(handle_, name.c_str(), value);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetIntValueEx(\"{}\", {}) failed: {:#x}", name, value, ret);
    return false;
  }
  return true;
}

bool HikRobot::set_bool_value(const std::string & name, bool value)
{
  const auto ret = MV_CC_SetBoolValue(handle_, name.c_str(), value);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetBoolValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return false;
  }
  return true;
}

void HikRobot::log_camera_state() const
{
  MVCC_INTVALUE_EX width{};
  MVCC_INTVALUE_EX height{};
  MVCC_ENUMVALUE pixel_format{};
  MVCC_ENUMVALUE white_balance{};
  MVCC_ENUMVALUE exposure_auto{};
  MVCC_ENUMVALUE gain_auto{};
  MVCC_ENUMVALUE trigger_mode{};
  MVCC_ENUMVALUE trigger_source{};
  MVCC_FLOATVALUE exposure{};
  MVCC_FLOATVALUE gain{};
  MVCC_FLOATVALUE fps{};

  const bool geometry_ok =
    MV_CC_GetIntValueEx(handle_, "Width", &width) == MV_OK &&
    MV_CC_GetIntValueEx(handle_, "Height", &height) == MV_OK &&
    MV_CC_GetEnumValue(handle_, "PixelFormat", &pixel_format) == MV_OK;
  if (geometry_ok) {
    tools::logger()->info(
      "[HikRobot][READBACK] image={}x{}, pixel_format={:#x}", width.nCurValue,
      height.nCurValue, pixel_format.nCurValue);
  } else {
    tools::logger()->warn("[HikRobot][READBACK] failed to read image geometry/format");
  }

  const bool exposure_ok =
    MV_CC_GetFloatValue(handle_, "ExposureTime", &exposure) == MV_OK &&
    MV_CC_GetFloatValue(handle_, "Gain", &gain) == MV_OK &&
    MV_CC_GetEnumValue(handle_, "ExposureAuto", &exposure_auto) == MV_OK &&
    MV_CC_GetEnumValue(handle_, "GainAuto", &gain_auto) == MV_OK &&
    MV_CC_GetEnumValue(handle_, "BalanceWhiteAuto", &white_balance) == MV_OK;
  if (exposure_ok) {
    tools::logger()->info(
      "[HikRobot][READBACK] exposure_us={:.1f} auto={}, gain={:.2f} auto={}, white_balance_auto={}",
      exposure.fCurValue, exposure_auto.nCurValue, gain.fCurValue, gain_auto.nCurValue,
      white_balance.nCurValue);
  } else {
    tools::logger()->warn("[HikRobot][READBACK] failed to read exposure/gain/white balance");
  }

  if (MV_CC_GetEnumValue(handle_, "TriggerMode", &trigger_mode) == MV_OK) {
    if (trigger_mode.nCurValue == MV_TRIGGER_MODE_ON &&
        MV_CC_GetEnumValue(handle_, "TriggerSource", &trigger_source) == MV_OK) {
      tools::logger()->info(
        "[HikRobot][READBACK] trigger=on, source={}, activation={}",
        trigger_source.nCurValue, trigger_activation_);
    } else {
      tools::logger()->info("[HikRobot][READBACK] trigger=off (continuous acquisition)");
    }
  }
  if (MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &fps) == MV_OK) {
    tools::logger()->info("[HikRobot][READBACK] acquisition_fps={:.2f}", fps.fCurValue);
  }
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io
