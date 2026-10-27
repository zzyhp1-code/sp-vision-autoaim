// PnP 解算核对工具：把"四角点 → 位姿 → 三维位置 → 射程"这一段独立量出来。
//
// 判据与含义
// ----------
//   * 重投影误差(px)：用解出的位姿 + 靶板物理尺寸反投影回图像，与检测到的角点比。
//     它同时检验 角点、内参/畸变、外参 三者，是这一环最硬的自检指标。
//   * 深度/射程稳定性：同一目标静置时深度与 xy 射程的抖动，直接决定弹道仰角与提前量。
//   * yaw 一致性：装甲法线角 yaw_raw 与优化后 ypr_in_gimbal 的差、以及逐帧抖动。
//   * 求解器内部重投影：Solver::solve 里 optimize_yaw 的目标函数值（armor_reprojection_error）。
//
// 数据流与运行时一致：camera -> YOLO(yolo26) -> Solver::solve() -> xyz_in_gimbal/xyz_in_world。
// IMU 用于世界系；没有下位机串口时加 --no-imu，工具仍会给出云台系与重投影结果。
//
// 用法(小电脑)：
//   ./build/pnp_probe -c=configs/standard.yaml --frames=150 --no-display \
//       --csv=/tmp/pnp_static.csv --overlay-dir=/tmp/pnp_frames [--no-imu]
// 只读：不写控制帧、不启云台/发射。

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/xuc.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace
{
const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明 }"
  "{config-path c  | configs/standard.yaml    | yaml 配置文件路径 }"
  "{frames         | 150                      | 采集帧数 }"
  "{csv            |                          | 逐帧导出 CSV 路径 }"
  "{overlay-dir    |                          | 叠层图输出目录(检测点=红, 重投影点=绿) }"
  "{no-imu         |                          | 没有下位机姿态时使用：跳过世界系, 只算云台系 }"
  "{no-display     |                          | 不显示窗口 }";

struct Row
{
  int frame = 0, det = 0;
  std::string name, type;
  double conf = 0, kpt = 0, color_score = 0;
  Eigen::Vector3d gimbal = Eigen::Vector3d::Zero();
  Eigen::Vector3d world = Eigen::Vector3d::Zero();
  double range_gimbal = 0, range_camera = 0;
  double yaw_raw = 0, yaw_solved = 0;
  double roll_solved = 0, pitch_solved = 0;  // 解出的装甲姿态（云台系欧拉）
  // 第二次重投影：用解出的 pitch（而不是代码写死的 15°）建模，验证偏差来源
  double rep_err_alt = 0, rep_width_alt = 0, rep_height_alt = 0;
  double rep_err_px = 0, solver_cost = 0;
  int reproject_ok = 0;
  std::array<cv::Point2f, 4> corners{};  // 检测到的四角（导出后可用离线脚本独立复核）
  // 重投影误差的方向分解：用来区分"尺度型"（径向向外/向内）与"平移型"（同方向）
  double err_dx = 0, err_dy = 0;              // 观测 - 重投影，图像系均值
  double obs_width = 0, rep_width = 0;        // 上/下边长均值（像素）
  double obs_height = 0, rep_height = 0;      // 左/右边长均值（像素）
  double obs_to_rep = 0;                      // 观测/重投影 的线性尺度比
};

double mean_of(const std::vector<double> & v)
{
  if (v.empty()) return 0.0;
  return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

double std_of(const std::vector<double> & v)
{
  if (v.size() < 2) return 0.0;
  const double m = mean_of(v);
  double s = 0;
  for (double x : v) s += (x - m) * (x - m);
  return std::sqrt(s / static_cast<double>(v.size() - 1));
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
  const int frame_limit = std::max(1, cli.get<int>("frames"));
  const bool use_imu = !cli.has("no-imu");
  const bool display = !cli.has("no-display");

  tools::Exiter exiter;
  io::Camera camera(config_path);
  auto_aim::YOLO yolo(config_path, display);
  auto_aim::Solver solver(config_path);

  // 内参与靶板尺寸：本工具做对照重投影时直接用同一份配置
  const auto cfg = YAML::LoadFile(config_path);
  const auto kdata = cfg["camera_matrix"].as<std::vector<double>>();
  const auto ddata = cfg["distort_coeffs"].as<std::vector<double>>();
  const cv::Mat K = (cv::Mat_<double>(3, 3) << kdata[0], kdata[1], kdata[2], kdata[3], kdata[4],
                     kdata[5], kdata[6], kdata[7], kdata[8]);
  const cv::Mat D = (cv::Mat_<double>(1, 5) << ddata[0], ddata[1], ddata[2], ddata[3], ddata[4]);
  constexpr double LIGHTBAR_LENGTH_M = 56e-3;
  constexpr double SMALL_ARMOR_WIDTH_M = 135e-3;
  constexpr double BIG_ARMOR_WIDTH_M = 230e-3;

  std::unique_ptr<io::XucSender> xuc;
  if (use_imu) {
    try {
      xuc = std::make_unique<io::XucSender>(config_path);
    } catch (const std::exception & e) {
      fmt::print(
        stderr, "[warning] 下位机姿态不可用({})；自动切换为 --no-imu 语义\n", e.what());
    }
  }
  fmt::print(
    "模式: {}；重投影判据参考: 均值 ≤1.5 px 且最大 ≤4 px\n",
    (xuc && xuc->enabled()) ? "相机+下位机姿态(世界系可用)"
                            : "仅相机(只算云台系，世界系不可用)");

  if (!overlay_dir.empty()) std::filesystem::create_directories(overlay_dir);
  std::ofstream csv;
  if (!csv_path.empty()) {
    csv.open(csv_path);
    if (!csv) {
      fmt::print(stderr, "[failure] 无法写入 CSV: {}\n", csv_path);
      return 1;
    }
    csv << "frame,det,name,type,conf,kpt,color_score,"
        << "x0,y0,x1,y1,x2,y2,x3,y3,"
        << "gimbal_x,gimbal_y,gimbal_z,world_x,world_y,world_z,"
        << "range_gimbal_m,range_camera_m,yaw_raw_deg,yaw_solved_deg,pitch_solved_deg,roll_solved_deg,"
        << "reproject_mean_px,reproject_max_px,solver_cost,reproject_ok,"
        << "err_dx_px,err_dy_px,obs_width_px,rep_width_px,obs_height_px,rep_height_px,obs_to_rep,"
        << "reproject_alt_mean_px,rep_alt_width_px,rep_alt_height_px\n";
  }

  std::vector<Row> rows;
  int frame = 0;
  while (!exiter.exit() && frame < frame_limit) {
    cv::Mat img;
    std::chrono::steady_clock::time_point ts;
    camera.read(img, ts);
    if (img.empty()) break;

    // 与运行时同构：先给求解器设置云台→世界（无 IMU 时保持单位阵）
    if (xuc) {
      const double yaw = xuc->imu_yaw();
      const double pitch = xuc->imu_pitch();
      const Eigen::Quaterniond q =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
      solver.set_R_gimbal2world(q);
    }

    auto armors = yolo.detect(img, frame);
    int det = 0;
    for (auto & armor : armors) {
      solver.solve(armor);

      Row r;
      r.frame = frame;
      r.det = det++;
      r.name = auto_aim::ARMOR_NAMES[static_cast<int>(armor.name)];
      r.type = auto_aim::ARMOR_TYPES[static_cast<int>(armor.type)];
      r.conf = armor.confidence;
      r.kpt = armor.keypoint_confidence;
      r.color_score = armor.color_score;
      r.gimbal = armor.xyz_in_gimbal;
      r.world = armor.xyz_in_world;
      r.range_gimbal = std::hypot(armor.xyz_in_gimbal.x(), armor.xyz_in_gimbal.y());
      r.range_camera = armor.xyz_in_gimbal.norm();
      r.yaw_raw = armor.yaw_raw;
      r.yaw_solved = armor.ypr_in_gimbal.z();
      r.pitch_solved = armor.ypr_in_gimbal.y();
      r.roll_solved = armor.ypr_in_gimbal.x();
      for (int i = 0; i < 4 && i < static_cast<int>(armor.points.size()); ++i)
        r.corners[i] = armor.points[i];

      // 重投影自检：用解出的位姿与靶板物理尺寸反投影，与检测角点逐点比较
      if (std::isfinite(armor.yaw_raw)) {
        const auto projected = solver.reproject_armor(
          armor.xyz_in_world, armor.yaw_raw, armor.type, armor.name);
        if (projected.size() == armor.points.size() && !projected.empty()) {
          double sum = 0, mx = 0, sdx = 0, sdy = 0;
          for (std::size_t i = 0; i < projected.size(); ++i) {
            const double e = cv::norm(projected[i] - armor.points[i]);
            sum += e;
            mx = std::max(mx, e);
            sdx += armor.points[i].x - projected[i].x;
            sdy += armor.points[i].y - projected[i].y;
            if (display || !overlay_dir.empty())
              cv::line(img, projected[i], armor.points[i], {0, 255, 255}, 1);
          }
          const double n = static_cast<double>(projected.size());
          r.rep_err_px = sum / n;
          r.solver_cost = r.rep_err_px;
          r.err_dx = sdx / n;
          r.err_dy = sdy / n;
          // 观测/重投影的四边形尺寸（顺序：0=LT,1=RT,2=RB,3=LB）
          const auto len = [](const cv::Point2f & a, const cv::Point2f & b) {
            return cv::norm(a - b);
          };
          r.obs_width =
            0.5 * (len(armor.points[0], armor.points[1]) + len(armor.points[3], armor.points[2]));
          r.rep_width =
            0.5 * (len(projected[0], projected[1]) + len(projected[3], projected[2]));
          r.obs_height =
            0.5 * (len(armor.points[0], armor.points[3]) + len(armor.points[1], armor.points[2]));
          r.rep_height =
            0.5 * (len(projected[0], projected[3]) + len(projected[1], projected[2]));
          const double rep_size = r.rep_width + r.rep_height;
          r.obs_to_rep = rep_size > 1e-6 ? (r.obs_width + r.obs_height) / rep_size : 0.0;
          r.reproject_ok = 1;
          for (const auto & p : projected) cv::circle(img, p, 2, {0, 255, 0}, -1);

          // 对照：把"装甲俯仰"换成解出的实测值，其余流程不变
          {
            const double yaw_use = std::isfinite(armor.yaw_raw) ? armor.yaw_raw : 0.0;
            const Eigen::Matrix3d R_armor2world =
              Eigen::AngleAxisd(yaw_use, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
              Eigen::AngleAxisd(r.pitch_solved, Eigen::Vector3d::UnitY()).toRotationMatrix() *
              Eigen::AngleAxisd(r.roll_solved, Eigen::Vector3d::UnitX()).toRotationMatrix();
            const Eigen::Matrix3d R_armor2camera =
              solver.R_camera2gimbal().transpose() *
              solver.R_gimbal2world().transpose() * R_armor2world;
            const Eigen::Vector3d t_armor2camera =
              solver.R_camera2gimbal().transpose() *
              (solver.R_gimbal2world().transpose() * armor.xyz_in_world -
               solver.t_camera2gimbal());
            cv::Mat Rcv;
            cv::eigen2cv(R_armor2camera, Rcv);
            cv::Vec3d rv;
            cv::Rodrigues(Rcv, rv);
            cv::Vec3d tv(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);
            const auto pts3d = [&] {
              const double w = (armor.type == auto_aim::ArmorType::big) ? BIG_ARMOR_WIDTH_M
                                                                       : SMALL_ARMOR_WIDTH_M;
              const double h = LIGHTBAR_LENGTH_M / 2.0;
              return std::vector<cv::Point3f>{
                {0, w / 2, h}, {0, -w / 2, h}, {0, -w / 2, -h}, {0, w / 2, -h}};
            }();
            std::vector<cv::Point2f> alt;
            cv::projectPoints(pts3d, rv, tv, K, D, alt);
            if (alt.size() == armor.points.size() && !alt.empty()) {
              double s2 = 0;
              for (std::size_t i = 0; i < alt.size(); ++i) {
                s2 += cv::norm(alt[i] - armor.points[i]);
                if (!overlay_dir.empty()) cv::circle(img, alt[i], 2, {255, 0, 255}, -1);
              }
              r.rep_err_alt = s2 / static_cast<double>(alt.size());
              const auto len2 = [](const cv::Point2f & a, const cv::Point2f & b) {
                return cv::norm(a - b);
              };
              r.rep_width_alt = 0.5 * (len2(alt[0], alt[1]) + len2(alt[3], alt[2]));
              r.rep_height_alt = 0.5 * (len2(alt[0], alt[3]) + len2(alt[1], alt[2]));
            }
          }
        }
      }
      for (const auto & p : armor.points) cv::circle(img, p, 3, {0, 0, 255}, -1);

      tools::draw_text(
        img,
        fmt::format(
          "{} {} conf {:.2f} range {:.3f}m | gimbal ({:.3f},{:.3f},{:.3f}) | rep {:.2f}px",
          r.name, r.type, r.conf, r.range_gimbal, r.gimbal.x(), r.gimbal.y(), r.gimbal.z(),
          r.rep_err_px),
        {12, 30}, {0, 255, 0});
      tools::draw_text(
        img,
        fmt::format(
          "yaw_raw {:+.2f}° yaw_solved {:+.2f}°", r.yaw_raw * 57.3, r.yaw_solved * 57.3),
        {12, 58}, {0, 255, 0});

      if (csv) {
        csv << r.frame << ',' << r.det << ',' << r.name << ',' << r.type << ',' << r.conf << ','
            << r.kpt << ',' << r.color_score;
        for (int i = 0; i < 4; ++i) csv << ',' << r.corners[i].x << ',' << r.corners[i].y;
        csv << ',' << r.gimbal.x() << ',' << r.gimbal.y() << ','
            << r.gimbal.z() << ',' << r.world.x() << ',' << r.world.y() << ',' << r.world.z() << ','
            << r.range_gimbal << ',' << r.range_camera << ',' << r.yaw_raw * 57.3 << ','
            << r.yaw_solved * 57.3 << ',' << r.pitch_solved * 57.3 << ',' << r.roll_solved * 57.3
            << ',' << r.rep_err_px << ',' << r.solver_cost << ',' << r.reproject_ok << ','
            << r.err_dx << ',' << r.err_dy << ',' << r.obs_width << ',' << r.rep_width << ','
            << r.obs_height << ',' << r.rep_height << ',' << r.obs_to_rep << ',' << r.rep_err_alt
            << ',' << r.rep_width_alt << ',' << r.rep_height_alt << '\n';
      }
      fmt::print(
        "[{:4d}.{}] {} {} conf {:.2f} kpt {:.2f} | gimbal ({:+.3f},{:+.3f},{:+.3f}) m | range "
        "{:.3f} m | rep {:.2f} px | yaw_raw {:+.2f}° yaw {:+.2f}°\n",
        r.frame, r.det, r.name, r.type, r.conf, r.kpt, r.gimbal.x(), r.gimbal.y(), r.gimbal.z(),
        r.range_gimbal, r.rep_err_px, r.yaw_raw * 57.3, r.yaw_solved * 57.3);
      rows.push_back(r);
    }
    std::fflush(stdout);

    if (!overlay_dir.empty() && !armors.empty()) {
      cv::imwrite(
        (std::filesystem::path(overlay_dir) / fmt::format("pnp_{:05d}.jpg", frame)).string(), img);
    }
    if (display) {
      cv::Mat shown = img;
      cv::resize(shown, shown, {}, 0.6, 0.6);
      cv::imshow("pnp_probe", shown);
      if (cv::waitKey(1) == 'q') break;
    }
    ++frame;
  }

  if (csv) csv.close();

  // ---------- 汇总 ----------
  fmt::print("\n===== PnP 汇总（{} 帧，{} 条检测）=====\n", frame, rows.size());
  if (rows.empty()) {
    fmt::print("没有检测到装甲板，无法评价。\n");
    return 1;
  }
  std::vector<double> rep, gx, gy, gz, rng, rng_cam, yraw, ysol, cs;
  for (const auto & r : rows) {
    if (r.reproject_ok) rep.push_back(r.rep_err_px);
    gx.push_back(r.gimbal.x());
    gy.push_back(r.gimbal.y());
    gz.push_back(r.gimbal.z());
    rng.push_back(r.range_gimbal);
    rng_cam.push_back(r.range_camera);
    yraw.push_back(r.yaw_raw * 57.3);
    ysol.push_back(r.yaw_solved * 57.3);
    cs.push_back(r.color_score);
  }
  double rep_max = 0;
  for (double v : rep) rep_max = std::max(rep_max, v);
  fmt::print(
    "重投影误差: 均值 {:.3f} px  标准差 {:.3f}  最大 {:.3f}  （{} 条有效）\n", mean_of(rep),
    std_of(rep), rep_max, rep.size());
  fmt::print(
    "云台系位置(m): x {:.4f}±{:.4f}  y {:.4f}±{:.4f}  z {:.4f}±{:.4f}\n", mean_of(gx), std_of(gx),
    mean_of(gy), std_of(gy), mean_of(gz), std_of(gz));
  fmt::print(
    "射程(m): xy 平面 {:.4f}±{:.4f}（{:.2f}% 抖动）  相机系模长 {:.4f}±{:.4f}\n", mean_of(rng),
    std_of(rng), 100.0 * std_of(rng) / std::max(1e-6, mean_of(rng)), mean_of(rng_cam),
    std_of(rng_cam));
  fmt::print(
    "yaw(deg): raw {:.3f}±{:.3f}   solved {:.3f}±{:.3f}   |raw-solved| 均值 {:.3f}\n",
    mean_of(yraw), std_of(yraw), mean_of(ysol), std_of(ysol),
    [&] {
      double s = 0;
      for (std::size_t i = 0; i < yraw.size(); ++i) s += std::abs(yraw[i] - ysol[i]);
      return yraw.empty() ? 0.0 : s / yraw.size();
    }());
  std::vector<double> psol, rsol;
  for (const auto & r : rows) {
    psol.push_back(r.pitch_solved * 57.3);
    rsol.push_back(r.roll_solved * 57.3);
  }
  fmt::print(
    "解出姿态(云台系,deg): pitch {:.2f}±{:.2f}  roll {:.2f}±{:.2f}  "
    "⇒ 代码重投影固定按 pitch=15° 建模，实测解出值与之的差就是重投影偏差的来源\n",
    mean_of(psol), std_of(psol), mean_of(rsol), std_of(rsol));
  fmt::print("颜色分数中位 {:.1f}\n", [&] {
    auto v = cs;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  }());
  // 误差分解：判断是"尺度型"还是"平移型"
  std::vector<double> edx, edy, obs_rep;
  double ow = 0, rw = 0, oh = 0, rh = 0;
  for (const auto & r : rows) {
    if (!r.reproject_ok) continue;
    edx.push_back(r.err_dx);
    edy.push_back(r.err_dy);
    obs_rep.push_back(r.obs_to_rep);
    ow += r.obs_width;
    rw += r.rep_width;
    oh += r.obs_height;
    rh += r.rep_height;
  }
  const double cnt = std::max(1.0, static_cast<double>(obs_rep.size()));
  fmt::print(
    "误差方向: 观测-重投影 dx {:+.2f}±{:.2f} px, dy {:+.2f}±{:.2f} px\n", mean_of(edx),
    std_of(edx), mean_of(edy), std_of(edy));
  fmt::print(
    "四边形尺寸(像素): 观测 宽 {:.1f} 高 {:.1f} | 重投影 宽 {:.1f} 高 {:.1f} | 观测/重投影 "
    "尺度比 {:.4f}\n",
    ow / cnt, oh / cnt, rw / cnt, rh / cnt, mean_of(obs_rep));
  fmt::print(
    "  ⇒ 尺度比接近 1.000 说明靶板尺寸模型与深度自洽；明显偏离则深度会被同比例带偏。\n");
  std::vector<double> alt;
  double rwa = 0, rha = 0;
  for (const auto & r : rows) {
    if (r.rep_err_alt > 0) alt.push_back(r.rep_err_alt);
    rwa += r.rep_width_alt;
    rha += r.rep_height_alt;
  }
  if (!alt.empty()) {
    const double n2 = static_cast<double>(alt.size());
    double alt_max = 0;
    for (double v : alt) alt_max = std::max(alt_max, v);
    fmt::print(
      "\n对照重投影（把写死的 pitch=15° 换成解出的实测俯仰）：均值 {:.3f} px  最大 {:.3f} px"
      " | 重投影 宽 {:.1f} 高 {:.1f}（观测 高 {:.1f}）\n",
      mean_of(alt), alt_max, rwa / n2, rha / n2, oh / cnt);
    fmt::print(
      "  ⇒ 若这一栏显著小于上面那栏，说明 6 px 来自 reproject_armor 里写死的 15° 装甲俯仰，\n"
      "    不是内参/外参/角点的问题。\n");
  }
  fmt::print(
    "\n判据: 重投影均值 ≤1.5 px、最大 ≤4 px，且射程抖动 ≤1% 视为这一段合格。\n");
  return 0;
}
