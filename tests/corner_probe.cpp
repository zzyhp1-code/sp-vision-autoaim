// 四角点核对工具：把 yolo26 的手眼标定前一级——装甲板四角点——逐帧导出成表。
//
// 目的: 装甲板四个角点是 PnP 与后续所有几何量的输入。目视叠层图只能看"像不像"，
//       静置时角点抖动多少、四点几何是否自洽、置信度是否够，必须靠逐帧数据统计。
//
// 数据流(与运行时一致):
//   相机 -> YOLO(yolo_name 配置, 默认 yolo26) -> Armor.points(4) + keypoint_confidence
//        -> [可选] 自适应角点细化(由 yolo26_refine_corners 决定) -> ratio/side_ratio/rectangular_error
//   本工具只读导出，不写任何控制帧，也不启动云台或发射。
//
// 用法(小电脑):
//   # 无窗口、不知道板在哪：先看实时状态行，按提示挪板，目标进画面中央后自动开始落盘
//   ./build/corner_probe -c=configs/standard.yaml --center --overlay-dir=/tmp/corner_frames \
//       --csv=/tmp/corner_center.csv
//   # 已确认位置，直接静置采固定帧数
//   ./build/corner_probe -c=configs/standard.yaml --frames=150 --no-display \
//       --csv=/tmp/corner_static.csv --overlay-dir=/tmp/corner_frames
//   ./build/corner_probe -c=<运行配置> --list        # 只打印面板字段
//
// 判读建议(静置数据):
//   - 单角抖动 rms 与四角中心抖动 rms: 静置时中心应很小(≈几 px 量级)；
//   - parallel_deg / rectangular_deg 中位数: 角点若压在灯条端点上，两轴应接近 0；
//   - keypoint_conf: 四点中最低的那个；min_keypoint_confidence 配置为 0 时不设卡，
//     该值就是唯一的网络侧质量信号；
//   - valid=1: 四点满足运行时的几何门槛(has_plausible_geometry + 配置比值的简化复算)。

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <fstream>
#include <list>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace
{
const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明 }"
  "{config-path c  | configs/standard.yaml    | yaml 配置文件路径 }"
  "{image          |                          | 只对单张图片做检测(不开相机，用于通道 A/B 对照) }"
  "{csv            |                          | 逐帧导出 CSV 路径 }"
  "{overlay-dir    |                          | 叠层图输出目录(每帧一张) }"
  "{frames         | 0                        | 采集帧数上限，0 表示不限 }"
  "{min-conf       | 0.0                      | 只导出 confidence 不低于该值的检测 }"
  "{live           |                          | 连续输出状态行(无窗口时用来找板) }"
  "{center         |                          | 等目标进入画面中央后才开始落盘(隐含 --live) }"
  "{center-margin  | 0.28                     | 判定“已居中”时中心离图像边的比例阈值 }"
  "{no-display     |                          | 不显示窗口(无图形会话时必须加) }"
  "{list           |                          | 只打印检测面板，不写入任何文件 }";

struct Metrics
{
  std::vector<cv::Point2f> points;
  double conf = 0.0, kpt = 0.0, color_score = 0.0;
  int color_source = 0, class_id = -1, valid = 0;
  std::string name, type;
  double left = 0, right = 0, top = 0, bottom = 0;
  double ratio = 0, side_ratio = 0, width_balance = 0;
  double rectangular_deg = 0, parallel_deg = 0, roll_deg = 0;
  double center_x = 0, center_y = 0;
  std::string reason;
};

double angle_deg(const cv::Point2f & v)
{
  return std::atan2(v.y, v.x) * 180.0 / CV_PI;
}

double abs_remainder_deg(double deg, double period)
{
  double r = std::remainder(deg, period);
  return std::abs(r);
}

// 与运行时同构的几何量：定义抄自 yolo26.cpp refine_corners / has_plausible_geometry。
Metrics measure(
  const auto_aim::Armor & armor, double min_ratio, double max_ratio, double max_side_ratio,
  double max_rectangular_error_deg)
{
  Metrics m;
  m.points = armor.points;
  m.conf = armor.confidence;
  m.kpt = armor.keypoint_confidence;
  m.color_score = armor.color_score;
  m.color_source = armor.color_source;
  m.class_id = armor.class_id;
  // class_id 是网络原始类别下标；name/type 才是映射后的语义，核对时看后者。
  m.name = auto_aim::ARMOR_NAMES[static_cast<int>(armor.name)];
  m.type = auto_aim::ARMOR_TYPES[static_cast<int>(armor.type)];
  if (armor.points.size() != 4) {
    m.reason = "points!=4";
    return m;
  }
  const auto & p = armor.points;
  m.left = cv::norm(p[3] - p[0]);
  m.right = cv::norm(p[2] - p[1]);
  m.top = cv::norm(p[1] - p[0]);
  m.bottom = cv::norm(p[2] - p[3]);
  const double height = 0.5 * (m.left + m.right);
  const double width = 0.5 * (m.top + m.bottom);
  m.side_ratio = std::max(m.left, m.right) / std::max(1.0, std::min(m.left, m.right));
  m.width_balance = std::max(m.top, m.bottom) / std::max(1.0, std::min(m.top, m.bottom));
  m.ratio = std::max(m.top, m.bottom) / std::max(1e-3, std::max(m.left, m.right));

  const cv::Point2f left_center = (p[0] + p[3]) * 0.5F;
  const cv::Point2f right_center = (p[1] + p[2]) * 0.5F;
  const double roll = std::atan2(right_center.y - left_center.y, right_center.x - left_center.x);
  m.roll_deg = roll * 180.0 / CV_PI;
  const auto perpendicular_error = [roll](const cv::Point2f & v) {
    return abs_remainder_deg(angle_deg(v) - roll * 180.0 / CV_PI - 90.0, 180.0);
  };
  m.rectangular_deg = std::max(perpendicular_error(p[3] - p[0]), perpendicular_error(p[2] - p[1]));
  m.parallel_deg = abs_remainder_deg(angle_deg(p[1] - p[0]) - angle_deg(p[2] - p[3]), 180.0);

  const cv::Point2f center = (p[0] + p[1] + p[2] + p[3]) * 0.25F;
  m.center_x = center.x;
  m.center_y = center.y;

  const double area = std::abs(cv::contourArea(p));
  std::vector<std::string> reasons;
  if (!(width >= 3.0 && height >= 3.0)) reasons.emplace_back("size");
  const double aspect = width / std::max(1e-3, height);
  if (!(aspect >= 0.6 && aspect <= 8.0)) reasons.emplace_back("aspect");
  if (!(m.side_ratio <= 2.2)) reasons.emplace_back("side_balance");
  if (!(m.width_balance <= 2.5)) reasons.emplace_back("width_balance");
  if (!(area >= 0.18 * width * height)) reasons.emplace_back("area");
  if (!(m.ratio >= min_ratio && m.ratio <= max_ratio)) reasons.emplace_back("ratio_cfg");
  if (!(m.side_ratio <= max_side_ratio)) reasons.emplace_back("side_ratio_cfg");
  if (!(m.rectangular_deg <= max_rectangular_error_deg)) reasons.emplace_back("rect_cfg");
  m.valid = reasons.empty() ? 1 : 0;
  for (std::size_t i = 0; i < reasons.size(); ++i) m.reason += (i ? "|" : "") + reasons[i];
  return m;
}

cv::Mat overlay(const cv::Mat & img, const Metrics & m, int frame, double fps)
{
  auto out = img.clone();
  const std::array<const char *, 4> names{"LT", "RT", "RB", "LB"};
  for (std::size_t i = 0; i < m.points.size(); ++i) {
    const auto & q = m.points[i];
    cv::circle(out, q, 3, m.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), -1);
    cv::putText(
      out, names[i], q + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255),
      1);
  }
  if (m.points.size() == 4) {
    std::vector<cv::Point> quad(m.points.begin(), m.points.end());
    cv::polylines(out, quad, true, m.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 2);
    cv::line(out, (m.points[0] + m.points[3]) * 0.5F, (m.points[1] + m.points[2]) * 0.5F,
             cv::Scalar(255, 128, 0), 1);
  }
  const std::string head = fmt::format(
    "frame {}  fps {:.1f}  conf {:.2f}  kpt {:.2f}  valid {}", frame, fps, m.conf, m.kpt,
    m.valid);
  const std::string geo = fmt::format(
    "ratio {:.3f}  side {:.3f}  wbal {:.3f}  rect {:.2f}deg  par {:.2f}deg  center ({:.1f},{:.1f})",
    m.ratio, m.side_ratio, m.width_balance, m.rectangular_deg, m.parallel_deg, m.center_x,
    m.center_y);
  tools::draw_text(out, head, {12, 28}, {0, 255, 0});
  tools::draw_text(out, geo, {12, 56}, {0, 255, 0});
  if (!m.reason.empty()) tools::draw_text(out, "reject: " + m.reason, {12, 84}, {0, 165, 255});
  return out;
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const auto config_path = cli.get<std::string>("config-path");
  const auto csv_path = cli.get<std::string>("csv");
  const auto overlay_dir = cli.get<std::string>("overlay-dir");
  const int frame_limit = cli.get<int>("frames");
  const double min_conf = cli.get<double>("min-conf");
  const bool live = cli.has("live") || cli.has("center");
  const double center_margin = std::clamp(cli.get<double>("center-margin"), 0.02, 0.49);
  const bool center_only = cli.has("center");
  const bool display = !cli.has("no-display") && !cli.has("list");
  const bool list_only = cli.has("list");

  auto yaml = YAML::LoadFile(config_path);
  const double min_ratio = yaml["min_armor_ratio"] ? yaml["min_armor_ratio"].as<double>() : 1.0;
  const double max_ratio = yaml["max_armor_ratio"] ? yaml["max_armor_ratio"].as<double>() : 5.0;
  const double max_side_ratio = yaml["max_side_ratio"] ? yaml["max_side_ratio"].as<double>() : 1.5;
  const double max_rect =
    yaml["max_rectangular_error"] ? yaml["max_rectangular_error"].as<double>() : 25.0;

  if (list_only) {
    fmt::print("面板字段(每帧每个检测一行):\n");
    fmt::print("  frame,det,class_id,name,type,confidence,keypoint_conf,color_source,color_score,valid,reason,\n");
    fmt::print("  x0..x3,y0..y3,left_px,right_px,top_px,bottom_px,ratio,side_ratio,width_balance,\n");
    fmt::print("  rectangular_deg,parallel_deg,roll_deg,center_x,center_y\n");
    fmt::print(
      "运行时几何门槛: ratio[{:.2f},{:.2f}] side_ratio<={:.2f} rectangular<={:.1f}deg\n",
      min_ratio, max_ratio, max_side_ratio, max_rect);
    return 0;
  }

  if (display) {
    // 无图形会话时 imshow 会直接抛异常，这里显式提示而不是让程序莫名其妙退出。
    const char * disp = std::getenv("DISPLAY");
    if (disp == nullptr) {
      fmt::print(
        stderr,
        "[warning] 没有 DISPLAY，已自动关闭显示；需要看叠层图请加 --overlay-dir 导出图片。\n");
    }
  }

  if (!overlay_dir.empty()) std::filesystem::create_directories(overlay_dir);
  std::ofstream csv;
  if (!csv_path.empty()) {
    csv.open(csv_path);
    if (!csv) {
      fmt::print(stderr, "[failure] 无法写入 CSV: {}\n", csv_path);
      return 1;
    }
    csv << "frame,det,class_id,name,type,confidence,keypoint_conf,color_source,color_score,valid,reason,"
        << "x0,y0,x1,y1,x2,y2,x3,y3,"
        << "left_px,right_px,top_px,bottom_px,ratio,side_ratio,width_balance,"
        << "rectangular_deg,parallel_deg,roll_deg,center_x,center_y\n";
  }

  tools::Exiter exiter;
  const auto image_path = cli.get<std::string>("image");
  const bool image_mode = !image_path.empty();
  io::Camera * camera = nullptr;
  if (!image_mode) camera = new io::Camera(config_path);
  auto_aim::YOLO yolo(config_path, display);

  int frame = 0;
  int kept = 0;
  int valid_frames = 0;
  bool centered_done = false;
  double fps = 0.0;
  std::string pending_overlay;  // --center 下不断刷新同一张，保证随时能看到最新居中画面
  auto last = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    cv::Mat img;
    std::chrono::steady_clock::time_point ts;
    if (image_mode) {
      // Windows/中文路径下 imread 走 ANSI，改用 imdecode
      std::ifstream fh(image_path, std::ios::binary);
      std::vector<unsigned char> buf((std::istreambuf_iterator<char>(fh)), std::istreambuf_iterator<char>());
      if (!buf.empty()) img = cv::imdecode(buf, cv::IMREAD_COLOR);
      if (img.empty()) {
        fmt::print(stderr, "[failure] 读不到图片: {}\n", image_path);
        return 1;
      }
    } else {
      camera->read(img, ts);
    }
    if (img.empty()) break;

    const auto armors = yolo.detect(img, frame);
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last).count();
    if (dt > 0) fps = 1.0 / dt;
    last = now;

    int det = 0;

    // --center: 先整帧判定“是否已居中”，居中后才开始落盘。
    // 现场如果没有可视窗口，就不该先录一堆偏靶数据；居中之前只刷新一张最新叠层图。
    const double margin_x = center_margin * img.cols;
    const double margin_y = center_margin * img.rows;
    const bool in_center = [&]() {
      for (const auto & armor : armors) {
        if (armor.confidence < min_conf) continue;
        const auto m = measure(armor, min_ratio, max_ratio, max_side_ratio, max_rect);
        if (!m.valid) continue;
        if (
          m.center_x >= margin_x && m.center_x <= img.cols - margin_x &&
          m.center_y >= margin_y && m.center_y <= img.rows - margin_y)
          return true;
      }
      return false;
    }();
    const bool record = !center_only || (centered_done && in_center);

    for (const auto & armor : armors) {
      if (armor.confidence < min_conf) continue;
      const auto m = measure(armor, min_ratio, max_ratio, max_side_ratio, max_rect);
      if (!record) {
        if (m.valid && in_center && !overlay_dir.empty()) {
          // 已经进了画面中央但还没开始落盘：刷新同一张图，方便你随时调出来确认位置。
          pending_overlay = "latest_center.jpg";
          const auto drawn = overlay(img, m, frame, fps);
          cv::imwrite((std::filesystem::path(overlay_dir) / pending_overlay).string(), drawn);
        }
        continue;
      }
      if (m.valid) valid_frames += 1;
      if (csv) {
        csv << frame << ',' << det << ',' << m.class_id << ',' << m.name << ',' << m.type << ','
            << m.conf << ',' << m.kpt << ','
            << m.color_source << ',' << m.color_score << ',' << m.valid << ',' << m.reason;
        for (int i = 0; i < 4; ++i) {
          const cv::Point2f q =
            i < static_cast<int>(m.points.size()) ? m.points[i] : cv::Point2f(0, 0);
          csv << ',' << q.x << ',' << q.y;
        }
        csv << ',' << m.left << ',' << m.right << ',' << m.top << ',' << m.bottom << ',' << m.ratio
            << ',' << m.side_ratio << ',' << m.width_balance << ',' << m.rectangular_deg << ','
            << m.parallel_deg << ',' << m.roll_deg << ',' << m.center_x << ',' << m.center_y << '\n';
      }
      fmt::print(
        "[{:5d}.{}] {} {} conf {:.3f} kpt {:.6f} valid {} ratio {:.3f} side {:.3f} rect {:.2f} "
        "par {:.2f} center ({:.1f},{:.1f}) {}\n",
        frame, det, m.name, m.type, m.conf, m.kpt, m.valid, m.ratio, m.side_ratio, m.rectangular_deg,
        m.parallel_deg, m.center_x, m.center_y, m.reason);
      if (!overlay_dir.empty()) {
        const auto drawn = overlay(img, m, frame, fps);
        cv::imwrite(
          (std::filesystem::path(overlay_dir) / fmt::format("frame_{:05d}_{}.jpg", frame, det))
            .string(),
          drawn);
      }
      ++det;
      ++kept;
    }
    std::fflush(stdout);

    if (live && (frame % 20 == 0)) {
      // 无窗口时靠这几行找板：位置、画面占比、以及该往哪个方向挪。
      if (armors.empty()) {
        fmt::print("[live {:5d}] 画面里没有检测到装甲板\n", frame);
      } else {
        for (const auto & armor : armors) {
          const auto m = measure(armor, min_ratio, max_ratio, max_side_ratio, max_rect);
          const char * dx =
            m.center_x < margin_x ? "向左" : (m.center_x > img.cols - margin_x ? "向右" : "已居中");
          const char * dy =
            m.center_y < margin_y ? "向上" : (m.center_y > img.rows - margin_y ? "向下" : "已居中");
          fmt::print(
            "[live {:5d}] {} {} conf {:.3f} kpt {:.6f} center ({:6.1f},{:6.1f}) 画面占比 "
            "X {:.2f} Y {:.2f} ⇒ 调整: {} / {} {}\n",
            frame, m.name, m.type, m.conf, m.kpt, m.center_x, m.center_y, m.center_x / img.cols,
            m.center_y / img.rows, dx, dy, m.valid ? "valid" : m.reason);
        }
        if (!pending_overlay.empty() && !overlay_dir.empty())
          fmt::print("[live] 已进入中央的叠层图: {}/{}\n", overlay_dir, pending_overlay);
      }
      std::fflush(stdout);
    }

    if (center_only && in_center && !centered_done) {
      centered_done = true;
      fmt::print(
        "[center] 第 {} 帧判定目标已进入画面中央（中心容差 {:.0f}%），从此开始落盘\n", frame,
        center_margin * 100.0);
      std::fflush(stdout);
    }

    if (display && std::getenv("DISPLAY") != nullptr) {
      cv::Mat shown = img.clone();
      for (const auto & armor : armors) {
        const auto m = measure(armor, min_ratio, max_ratio, max_side_ratio, max_rect);
        shown = overlay(shown, m, frame, fps);
      }
      tools::draw_text(
        shown, fmt::format("kept {}  'q' quit", kept), {12, shown.rows - 16}, {255, 255, 255});
      cv::resize(shown, shown, {}, 0.6, 0.6);
      cv::imshow("corner_probe", shown);
      if (cv::waitKey(1) == 'q') break;
    }

    ++frame;
    // --center 时的帧数上限从“进入中央之后”才开始计，否则挪板的时间会把名额用完。
    const bool counting = !center_only || centered_done;
    if (frame_limit > 0 && counting && kept >= frame_limit) break;
    if (image_mode) break;  // 单图模式只跑一帧
  }

  delete camera;

  if (csv) csv.close();
  fmt::print(
    "完成: 帧 {}，导出检测 {} 条（其中几何自洽 {} 条）{}\n", frame, kept, valid_frames,
    csv_path.empty() ? "" : ("，CSV: " + csv_path));
  return 0;
}
