#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <memory>
#include <opencv2/opencv.hpp>
#include <stdexcept>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/xuc.hpp"
#include "tools/cmdline.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }"
  "{auto-interval-s | 0                        | 自动采样间隔(秒), 0=手动按键保存 }"
  "{auto-count      | 0                        | 自动采样总张数, 0=不限制 }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  if (!q_file) throw std::runtime_error("Cannot open quaternion output");
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{:.17g} {:.17g} {:.17g} {:.17g}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.flush();
  if (!q_file) throw std::runtime_error("Cannot save quaternion");
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder,
  int pattern_cols, int pattern_rows, bool use_cboard, bool use_xuc, double auto_interval_s,
  int auto_count)
{
  const auto settings = YAML::LoadFile(config_path);
  const auto stable_window = std::chrono::milliseconds(
    settings["capture_pose_stable_window_ms"] ? settings["capture_pose_stable_window_ms"].as<int>() : 500);
  const double stable_range = (settings["capture_pose_stable_range_deg"] ?
    settings["capture_pose_stable_range_deg"].as<double>() : 0.05) * CV_PI / 180.0;
  const double max_frame_age_ms = settings["capture_max_frame_age_ms"] ?
    settings["capture_max_frame_age_ms"].as<double>() : 100.0;
  if (stable_window.count() <= 0 || !std::isfinite(stable_range) || stable_range <= 0 ||
      !std::isfinite(max_frame_age_ms) || max_frame_age_ms <= 0) {
    throw std::runtime_error("Invalid capture stability/age settings");
  }
  // ===== capture helper: wide-span station grid =====
  // Station target = where the board centre should appear, as a fraction of the frame.
  // Six stations on a 3x2 grid span ~22 deg yaw x ~14 deg pitch, which is what the
  // hand-eye fit needs to be identifiable (a narrow ~9 deg cross is not).
  // Press 1-6 to select, 0 to turn the helper off. Tolerance +/-60 px per axis.
  const std::vector<cv::Point2f> station_frac = {
    {0.145f, 0.20f}, {0.50f, 0.20f}, {0.855f, 0.20f},
    {0.145f, 0.80f}, {0.50f, 0.80f}, {0.855f, 0.80f}};
  const double station_tol_px = 60.0;
  int station = -1;  // -1 = helper off
  cv::Size station_img_size;
  std::vector<cv::Point2f> station_targets;
  // ===== automatic sampling (static soak / drift test) =====
  // --auto-interval-s=30 --auto-count=20 saves 20 frames 30 s apart and exits.
  // The reported-pose stability gate is bypassed in auto mode on purpose: the
  // drift being measured IS a change of the reported pose while the rig is still.
  const bool auto_mode = auto_interval_s > 0.0 && auto_count > 0;
  auto last_auto_save = std::chrono::steady_clock::now() - std::chrono::hours(1);
  std::ofstream metadata(output_folder + "/samples.csv", std::ios::app);
  if (!metadata) throw std::runtime_error("Cannot open sample metadata");
  if (metadata.tellp() == 0) {
    metadata << "sample,image_host_ns,save_host_ns,save_unix_ms,imu_bracket_ms,yaw_rad,pitch_rad,stable_window_ms,stable_range_deg,timestamp_source,station\n";
  }
  metadata << std::setprecision(17);
  // CBoard 构造函数会阻塞等待 C 板 CAN 四元数帧。
  // 没有 C 板时（仅做内参标定）跳过，四元数以单位四元数代替。
  std::unique_ptr<io::CBoard> cboard;
  if (use_cboard) {
    cboard = std::make_unique<io::CBoard>(config_path);
  } else if (!use_xuc) {
    tools::logger()->warn("require_cboard=false: 未读取C板四元数，仅适用于内参标定");
  }
  std::unique_ptr<io::XucSender> xuc;
  if (use_xuc) {
    xuc = std::make_unique<io::XucSender>(config_path);
    if (!xuc->enabled()) {
      throw std::runtime_error("xuc_enable=true, but the XUC serial port could not be opened");
    }
    tools::logger()->info(
      "使用 XUC 串口姿态采集；capture 不发送任何控制或射击命令");
  }
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  cv::Size pattern_size(pattern_cols, pattern_rows);
  int count = 0;
  int failed_count = 0;

  tools::logger()->info("=== 棋盘格标定数据采集 ===");
  tools::logger()->info("标定板尺寸: {}x{} 内角点", pattern_cols, pattern_rows);
  tools::logger()->info("按 's' 键保存, 按 'q' 键退出");

  while (true) {
    camera.read(img, timestamp);
    if (station_targets.empty() || station_img_size != img.size()) {
      station_img_size = img.size();
      station_targets.clear();
      for (const auto & f : station_frac)
        station_targets.emplace_back(f.x * img.cols, f.y * img.rows);
    }
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    bool pose_valid = !cboard && !xuc;
    bool pose_stable = !xuc;
    io::XucSender::ImuPose pose{};
    if (cboard) {
      q = cboard->imu_at(timestamp);
      pose_valid = true;
    } else if (xuc && xuc->rx_valid() && xuc->imu_at(timestamp, pose)) {
      q = Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pose.pitch, Eigen::Vector3d::UnitY());
      q.normalize();
      pose_valid = true;
      pose_stable = xuc->imu_stable_at(timestamp, stable_window, stable_range);
    }

    // 在图像上显示欧拉角
    auto img_with_info = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * (180.0 / CV_PI);  // degree
    tools::draw_text(img_with_info, fmt::format("IMU Z(Yaw): {:.3f}", zyx[0]), {10, 30}, {255, 255, 0});
    tools::draw_text(img_with_info, fmt::format("IMU Y(Pitch): {:.3f}", zyx[1]), {10, 60}, {255, 255, 0});
    tools::draw_text(img_with_info, fmt::format("IMU X(Roll): {:.3f}", zyx[2]), {10, 90}, {255, 255, 0});
    if (!pose_valid) {
      tools::draw_text(img_with_info, "IMU INVALID - DO NOT SAVE", {10, 150}, {0, 0, 255}, 1.0, 2);
    } else if (!pose_stable) {
      tools::draw_text(img_with_info, "HOLD STILL BEFORE SAVING", {10, 150}, {0, 0, 255}, 1.0, 2);
    } else {
      tools::draw_text(img_with_info, "READY TO SAVE", {10, 150}, {0, 255, 0}, 1.0, 2);
    }

    // Camera frames are BGR (the driver converts Bayer to BGR now).  Detect on an
    // explicitly converted grayscale image; display and image files are also BGR.
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // 检测棋盘格角点
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(
      gray, pattern_size, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
      // 亚像素精化（提高精度）
      cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

      // 显示检测到的角点
      cv::drawChessboardCorners(img_with_info, pattern_size, corners, found);
      tools::draw_text(img_with_info, "Chessboard OK!", {10, 120}, {0, 255, 0}, 1.0, 2);
      failed_count = 0;
    } else {
      failed_count++;
      tools::draw_text(img_with_info, fmt::format("NOT found ({})", failed_count),
                       {10, 120}, {0, 0, 255}, 0.8, 2);
    }

    // 站位辅助：目标框 + 棋盘中心偏差（仅在按 1-4 选择站位后显示）
    if (station >= 0 && station < static_cast<int>(station_targets.size())) {
      const auto target = station_targets[station];
      const cv::Point tgt(cvRound(target.x), cvRound(target.y));
      const int tol = static_cast<int>(station_tol_px);
      cv::rectangle(
        img_with_info, {tgt.x - tol, tgt.y - tol}, {tgt.x + tol, tgt.y + tol},
        {255, 255, 255}, 1, cv::LINE_AA);
      cv::line(img_with_info, {tgt.x - 16, tgt.y}, {tgt.x + 16, tgt.y}, {255, 255, 0}, 1, cv::LINE_AA);
      cv::line(img_with_info, {tgt.x, tgt.y - 16}, {tgt.x, tgt.y + 16}, {255, 255, 0}, 1, cv::LINE_AA);
      if (found) {
        cv::Point2f center(0, 0);
        for (const auto & c : corners) center += c;
        center *= 1.0f / static_cast<float>(corners.size());
        const double dx = center.x - target.x;
        const double dy = center.y - target.y;
        const bool inside = std::abs(dx) <= station_tol_px && std::abs(dy) <= station_tol_px;
        const cv::Scalar mark = inside ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 96, 0);
        const cv::Point ctr(cvRound(center.x), cvRound(center.y));
        cv::circle(img_with_info, ctr, 6, mark, 2, cv::LINE_AA);
        cv::line(img_with_info, ctr, tgt, mark, 1, cv::LINE_AA);
        tools::draw_text(
          img_with_info,
          fmt::format("R{} dx={:+.0f} dy={:+.0f} {}", station + 1, dx, dy, inside ? "OK" : "MOVE"),
          {10, 180}, mark, 1.0, 2);
      } else {
        tools::draw_text(
          img_with_info, fmt::format("R{} board NOT found", station + 1), {10, 180},
          cv::Scalar(255, 96, 0), 1.0, 2);
      }
    }
    tools::draw_text(img_with_info, fmt::format("Captured: {}", count),
                     {10, img_with_info.rows - 20}, {255, 255, 255}, 1.0, 2);
    tools::draw_text(
      img_with_info,
      station >= 0 ? fmt::format("STATION R{}  (1-6 select / 0 off)", station + 1)
                   : fmt::format("station helper off (press 1-6)"),
      {10, img_with_info.rows - 50}, {255, 255, 255}, 0.7, 2);

    cv::Mat preview_bgr = img_with_info;  // 已经是 BGR
    cv::resize(preview_bgr, preview_bgr, {}, 0.5, 0.5);
    cv::imshow("Chessboard Capture - 's' to save, 'q' to quit", preview_bgr);

    auto key = cv::waitKey(1);
    if (key >= '1' && key <= '6') {
      station = key - '1';
      tools::logger()->info(
        "[STATION] R{} target=({:.0f},{:.0f}) px tol={:.0f} px", station + 1,
        station_targets[station].x, station_targets[station].y, station_tol_px);
    } else if (key == '0') {
      station = -1;
      tools::logger()->info("[STATION] helper off");
    }
    if (key == 'q') {
      break;
    }
    bool want_save = key == 's';
    if (auto_mode && count < auto_count &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_auto_save).count() >=
          auto_interval_s) {
      want_save = true;
    }
    if (want_save) {
      if (!found) {
        tools::logger()->warn("未检测到棋盘格，无法保存！");
        continue;
      }
      if (!pose_valid || (!pose_stable && !auto_mode)) {
        tools::logger()->warn("IMU pose missing or moving; refusing this sample");
        continue;
      }
      const auto save_time = std::chrono::steady_clock::now();
      const double age_ms = std::chrono::duration<double, std::milli>(save_time - timestamp).count();
      if (age_ms < 0 || age_ms > max_frame_age_ms) {
        tools::logger()->warn("Frame too old to save: {:.1f} ms", age_ms);
        continue;
      }

      // 保存图片和四元数
      count++;
      auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
      auto q_path = fmt::format("{}/{}.txt", output_folder, count);
      cv::Mat saved_bgr = img;  // 已经是 BGR
      if (!cv::imwrite(img_path, saved_bgr)) throw std::runtime_error("Cannot save image");
      write_q(q_path, q);
      metadata << count << ',' << std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count()
               << ',' << std::chrono::duration_cast<std::chrono::nanoseconds>(save_time.time_since_epoch()).count()
               << ',' << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
               << ',' << pose.bracket_ms << ',' << pose.yaw << ',' << pose.pitch
               << ',' << stable_window.count() << ',' << stable_range * 180.0 / CV_PI
               << ',' << (xuc ? "camera_and_imu_host_receive" : cboard ? "camera_host_receive_cboard" : "camera_host_receive_identity")
               << ',' << (station >= 0 ? station + 1 : 0) << '\n';
      metadata.flush();
      if (!metadata) throw std::runtime_error("Cannot save sample metadata");
      tools::logger()->info("[{}] Saved to {} (corners: {})", count, output_folder, corners.size());
      last_auto_save = std::chrono::steady_clock::now();
      if (auto_mode && count >= auto_count) {
        tools::logger()->info("[AUTO] sampling finished: {} frames", count);
        break;
      }
    }
  }

  tools::logger()->info("采集完成！共 {} 组数据", count);
  if (count < 10) {
    tools::logger()->warn("建议至少采集10-15组数据");
  }
}

int main(int argc, char * argv[])
{
  // 读取命令行参数（规整 "-o xxx" 这类空格写法，OpenCV CommandLineParser 只认 '=' 形式）
  auto parsed = tools::normalize_argv(argc, argv);
  cv::CommandLineParser cli(parsed.argc(), parsed.argv(), keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");
  const double auto_interval_s = cli.get<double>("auto-interval-s");
  const int auto_count = cli.get<int>("auto-count");

  // 新建输出文件夹
  std::filesystem::create_directories(output_folder);
  for (const auto & entry : std::filesystem::directory_iterator(output_folder)) {
    if (entry.path().extension() == ".jpg" || entry.path().extension() == ".txt") {
      throw std::runtime_error("Output folder already contains samples; choose an empty folder");
    }
  }

  // 读取标定板尺寸
  auto yaml = YAML::LoadFile(config_path);
  int pattern_cols = yaml["pattern_cols"].as<int>();
  int pattern_rows = yaml["pattern_rows"].as<int>();

  // 是否有 C 板/CAN（手眼标定必需；仅内参标定可关掉）
  bool use_cboard = true;
  if (yaml["require_cboard"]) use_cboard = yaml["require_cboard"].as<bool>();
  bool use_xuc = yaml["xuc_enable"] && yaml["xuc_enable"].as<bool>();
  if (use_cboard && use_xuc) {
    tools::logger()->warn("require_cboard 和 xuc_enable 同时为 true；优先使用 XUC 姿态");
    use_cboard = false;
  }

  // 主循环，保存图片和对应四元数
  capture_loop(
    config_path, output_folder, pattern_cols, pattern_rows, use_cboard, use_xuc, auto_interval_s,
    auto_count);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
