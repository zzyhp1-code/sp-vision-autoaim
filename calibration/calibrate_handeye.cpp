#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <cmath>
#include <fstream>
#include <algorithm>
#include <limits>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "tools/cmdline.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{no-display      |                          | 批处理模式，不逐帧等待按键}"
  "{no-refine       |                          | 仅输出原始Tsai结果}"
  "{self-test       |                          | 离线合成数据验证联合优化}"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float square_size)
{
  std::vector<cv::Point3f> centers_3d;

  // 棋盘格内角点坐标（左上角为原点）
  for (int i = 0; i < pattern_size.height; i++)
    for (int j = 0; j < pattern_size.width; j++)
      centers_3d.push_back({j * square_size, i * square_size, 0});

  return centers_3d;
}

Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream q_file(q_path);
  double w = 0, x = 0, y = 0, z = 0;
  if (!(q_file >> w >> x >> y >> z)) return Eigen::Quaterniond(0, 0, 0, 0);
  return {w, x, y, z};
}

struct ReprojectionProblem
{
  cv::Mat K, D;
  std::vector<cv::Point3d> object;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<cv::Mat> gimbal;
};

using Parameters = Eigen::Matrix<double, 12, 1>;

cv::Mat rotation(const Parameters & x, int offset)
{
  cv::Mat r = (cv::Mat_<double>(3, 1) << x[offset], x[offset + 1], x[offset + 2]);
  cv::Mat R;
  cv::Rodrigues(r, R);
  return R;
}

cv::Mat translation(const Parameters & x, int offset)
{
  return (cv::Mat_<double>(3, 1) << x[offset], x[offset + 1], x[offset + 2]);
}

void put_pose(Parameters & x, int offset, const cv::Mat & R, const cv::Mat & t)
{
  cv::Mat r;
  cv::Rodrigues(R, r);
  for (int i = 0; i < 3; ++i) {
    x[offset + i] = r.at<double>(i);
    x[offset + 3 + i] = t.at<double>(i);
  }
}

// Fit one rigid camera mount and one fixed board pose to all measured pixels.
// Intrinsics and the measured IMU poses remain fixed, in mm/radians internally.
bool residuals(const ReprojectionProblem & p, const Parameters & x, Eigen::VectorXd & errors)
{
  errors.resize(p.gimbal.size() * p.object.size() * 2);
  const cv::Mat Rc = rotation(x, 0), tc = translation(x, 3);
  const cv::Mat Rb = rotation(x, 6), tb = translation(x, 9);
  int n = 0;
  for (size_t i = 0; i < p.gimbal.size(); ++i) {
    const cv::Mat Rwc = Rc.t() * p.gimbal[i].t();
    const cv::Mat R = Rwc * Rb;
    const cv::Mat t = Rc.t() * (p.gimbal[i].t() * tb - tc);
    for (const auto & point : p.object) {
      const double z = R.at<double>(2, 0) * point.x + R.at<double>(2, 1) * point.y + t.at<double>(2);
      if (!std::isfinite(z) || z <= 0) return false;
    }
    cv::Mat r;
    cv::Rodrigues(R, r);
    std::vector<cv::Point2d> projected;
    cv::projectPoints(p.object, r, t, p.K, p.D, projected);
    for (size_t j = 0; j < projected.size(); ++j) {
      errors[n++] = projected[j].x - p.corners[i][j].x;
      errors[n++] = projected[j].y - p.corners[i][j].y;
    }
  }
  return errors.allFinite();
}

void print_errors(const char * label, const Eigen::VectorXd & e)
{
  double sum = 0, maximum = 0;
  for (int i = 0; i < e.size(); i += 2) {
    const double d = e.segment<2>(i).norm();
    sum += d;
    maximum = std::max(maximum, d);
  }
  fmt::print("{} reprojection: mean={:.6f} px, max={:.6f} px\n", label, sum / (e.size() / 2), maximum);
}

bool refine(const ReprojectionProblem & p, cv::Mat & Rc, cv::Mat & tc,
            const std::vector<cv::Mat> & rvecs, const std::vector<cv::Mat> & tvecs)
{
  Parameters x;
  put_pose(x, 0, Rc, tc);
  Eigen::Matrix3d Rsum = Eigen::Matrix3d::Zero();
  cv::Mat tsum = cv::Mat::zeros(3, 1, CV_64F);
  for (size_t i = 0; i < rvecs.size(); ++i) {
    cv::Mat R;
    cv::Rodrigues(rvecs[i], R);
    cv::Mat world_R = p.gimbal[i] * Rc * R;
    Eigen::Matrix3d E;
    cv::cv2eigen(world_R, E);
    Rsum += E;
    tsum += p.gimbal[i] * (Rc * tvecs[i] + tc);
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(Rsum, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d sign = Eigen::Matrix3d::Identity();
  sign(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
  cv::Mat Rboard;
  Eigen::Matrix3d mean_R = svd.matrixU() * sign * svd.matrixV().transpose();
  cv::eigen2cv(mean_R, Rboard);
  put_pose(x, 6, Rboard, tsum / static_cast<double>(rvecs.size()));
  Eigen::VectorXd e;
  if (!x.allFinite() || !residuals(p, x, e)) return false;
  print_errors("Initial", e);
  double cost = e.squaredNorm(), lambda = 1e-3;
  for (int iteration = 0; iteration < 80; ++iteration) {
    Eigen::MatrixXd J(e.size(), 12);
    for (int j = 0; j < 12; ++j) {
      const double step = (j % 6 < 3) ? 1e-6 : 1e-3;
      Parameters plus = x, minus = x;
      plus[j] += step; minus[j] -= step;
      Eigen::VectorXd ep, em;
      if (!residuals(p, plus, ep) || !residuals(p, minus, em)) return false;
      J.col(j) = (ep - em) / (2 * step);
    }
    const Eigen::Matrix<double, 12, 12> H = J.transpose() * J;
    const Parameters gradient = J.transpose() * e;
    bool accepted = false, converged = false;
    for (int attempt = 0; attempt < 12; ++attempt) {
      Eigen::Matrix<double, 12, 12> damped = H;
      for (int j = 0; j < 12; ++j) damped(j, j) += lambda * std::max(1e-12, H(j, j));
      const Parameters delta = damped.ldlt().solve(-gradient);
      Eigen::VectorXd trial;
      const Parameters candidate = x + delta;
      if (delta.allFinite() && residuals(p, candidate, trial) && trial.squaredNorm() < cost) {
        const double improved = cost - trial.squaredNorm();
        converged = improved < 1e-10 * std::max(1.0, cost);
        x = candidate; e = trial; cost = e.squaredNorm();
        lambda = std::max(1e-12, lambda / 3);
        accepted = true;
        break;
      }
      lambda *= 10;
    }
    if (!accepted || converged) break;
  }
  print_errors("Refined", e);
  Rc = rotation(x, 0);
  tc = translation(x, 3);
  return true;
}

int self_test()
{
  ReprojectionProblem p;
  p.K = (cv::Mat_<double>(3, 3) << 1000, 0, 640, 0, 1000, 480, 0, 0, 1);
  p.D = cv::Mat::zeros(5, 1, CV_64F);
  for (const auto & v : centers_3d({9, 7}, 20)) p.object.emplace_back(v.x, v.y, v.z);
  Parameters truth = Parameters::Zero();
  truth << .03, -.04, .02, 80, 10, 25, .06, .03, -.02, -100, -60, 1000;
  const cv::Mat Rc = rotation(truth, 0), tc = translation(truth, 3);
  std::vector<cv::Mat> rvecs, tvecs;
  for (int i = 0; i < 25; ++i) {
    Parameters g = Parameters::Zero();
    g[0] = (i % 5 - 2) * .08; g[1] = (i / 5 - 2) * .07; g[2] = .03 * std::sin(i);
    p.gimbal.push_back(rotation(g, 0));
    const cv::Mat R = Rc.t() * p.gimbal.back().t() * rotation(truth, 6);
    const cv::Mat t = Rc.t() * (p.gimbal.back().t() * translation(truth, 9) - tc);
    cv::Mat r;
    cv::Rodrigues(R, r);
    std::vector<cv::Point2f> points;
    std::vector<cv::Point2d> projected;
    cv::projectPoints(p.object, r, t, p.K, p.D, projected);
    for (const auto & v : projected) points.emplace_back(v.x, v.y);
    p.corners.push_back(points); rvecs.push_back(r); tvecs.push_back(t);
  }
  Parameters seed = truth;
  seed[0] += .04; seed[1] -= .03; seed[3] += 20; seed[5] -= 15;
  cv::Mat fitted_R = rotation(seed, 0), fitted_t = translation(seed, 3);
  if (!refine(p, fitted_R, fitted_t, rvecs, tvecs)) return 3;
  cv::Mat dr;
  cv::Rodrigues(fitted_R * Rc.t(), dr);
  const double angle = cv::norm(dr), position = cv::norm(fitted_t - tc);
  const bool pass = angle < 1e-5 && position < .01;
  fmt::print("JOINT_FIT_SELF_TEST={} rotation_error_rad={:.9g} translation_error_mm={:.9g}\n",
             pass ? "PASS" : "FAIL", angle, position);
  return pass ? 0 : 3;
}

void load(
  const std::string & input_folder, const std::string & config_path,
  std::vector<double> & R_gimbal2imubody_data, std::vector<cv::Mat> & R_gimbal2world_list,
  std::vector<cv::Mat> & t_gimbal2world_list, std::vector<cv::Mat> & rvecs,
  std::vector<cv::Mat> & tvecs, bool display, ReprojectionProblem & problem)
{
  // 读取yaml参数
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto square_size_mm = yaml["square_size_mm"].as<double>();
  R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

  cv::Size pattern_size(pattern_cols, pattern_rows);
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());
  cv::Matx33d camera_matrix(camera_matrix_data.data());
  cv::Mat distort_coeffs(distort_coeffs_data);
  problem.K = cv::Mat(camera_matrix).clone();
  problem.D = distort_coeffs.clone();
  const auto object = centers_3d(pattern_size, square_size_mm);
  for (const auto & v : object) problem.object.emplace_back(v.x, v.y, v.z);

  for (int i = 1; true; i++) {
    // 读取图片和对应四元数
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto q_path = fmt::format("{}/{}.txt", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) break;
    Eigen::Quaterniond q = read_q(q_path);
    if (!std::isfinite(q.norm()) || q.norm() < 0.99 || q.norm() > 1.01) {
      fmt::print(stderr, "[failure] 四元数无效: {}\n", q_path);
      continue;
    }
    q.normalize();

    // 计算云台的欧拉角
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    Eigen::Matrix3d R_gimbal2world =
      R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
    Eigen::Vector3d ypr = tools::eulers(R_gimbal2world, 2, 1, 0) * 57.3;  // degree

    // 在图片上显示云台的欧拉角，用来检验R_gimbal2imubody是否正确
    auto drawing = img.clone();
    tools::draw_text(drawing, fmt::format("yaw   {:.2f}", ypr[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("pitch {:.2f}", ypr[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("roll  {:.2f}", ypr[2]), {40, 120}, {0, 0, 255});

    // 识别棋盘格
    std::vector<cv::Point2f> corners;
    auto success = cv::findChessboardCorners(
      img, pattern_size, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (success) {
      // 亚像素精化
      cv::Mat gray;
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    }

    if (display) {
      cv::drawChessboardCorners(drawing, pattern_size, corners, success);
      cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
      cv::imshow("Press any to continue", drawing);
      cv::waitKey(0);
    }

    // 输出识别结果
    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (!success) continue;

    // 计算所需的数据
    cv::Mat t_gimbal2world = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat R_gimbal2world_cv;
    cv::eigen2cv(R_gimbal2world, R_gimbal2world_cv);
    cv::Mat rvec, tvec;
    auto corners_3d = centers_3d(pattern_size, square_size_mm);
    if (!cv::solvePnP(
      corners_3d, corners, camera_matrix, distort_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE)) continue;

    // Symmetric boards may reverse corner numbering. Correct the observations
    // and re-solve the complete pose, including its translated board origin.
    if (!rvecs.empty()) {
      cv::Mat previous, current, delta;
      cv::Rodrigues(rvecs.back(), previous);
      cv::Rodrigues(rvec, current);
      cv::Rodrigues(current * previous.t(), delta);
      if (cv::norm(delta) > CV_PI / 2) {
        std::reverse(corners.begin(), corners.end());
        if (!cv::solvePnP(corners_3d, corners, camera_matrix, distort_coeffs,
                         rvec, tvec, false, cv::SOLVEPNP_IPPE)) continue;
        fmt::print("Corrected reversed board numbering: {}\n", img_path);
      }
    }

    // 记录所需的数据
    R_gimbal2world_list.emplace_back(R_gimbal2world_cv);
    t_gimbal2world_list.emplace_back(t_gimbal2world);
    rvecs.emplace_back(rvec);
    tvecs.emplace_back(tvec);
    problem.corners.push_back(corners);
    problem.gimbal.push_back(R_gimbal2world_cv);
  }

}

void print_yaml(
  const std::vector<double> & R_gimbal2imubody_data, const cv::Mat & R_camera2gimbal,
  const cv::Mat & t_camera2gimbal, const Eigen::Vector3d & ypr)
{
  YAML::Emitter result;
  std::vector<double> R_camera2gimbal_data(
    R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
  std::vector<double> t_camera2gimbal_data(
    t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

  result << YAML::BeginMap;
  result << YAML::Key << "R_gimbal2imubody";
  result << YAML::Value << YAML::Flow << R_gimbal2imubody_data;
  result << YAML::Newline;
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "相机同理想情况的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree", ypr[0], ypr[1], ypr[2]));
  result << YAML::Key << "R_camera2gimbal";
  result << YAML::Value << YAML::Flow << R_camera2gimbal_data;
  result << YAML::Key << "t_camera2gimbal";
  result << YAML::Value << YAML::Flow << t_camera2gimbal_data;
  result << YAML::Newline;
  result << YAML::EndMap;

  fmt::print("\n{}\n", result.c_str());
}

int main(int argc, char * argv[])
{
  // 读取命令行参数（规整 "-c xxx" 这类空格写法，OpenCV CommandLineParser 只认 '=' 形式）
  auto parsed = tools::normalize_argv(argc, argv);
  cv::CommandLineParser cli(parsed.argc(), parsed.argv(), keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  if (cli.has("self-test")) return self_test();
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  const bool display = !cli.has("no-display");

  // 从输入文件夹中加载标定所需的数据
  std::vector<double> R_gimbal2imubody_data;
  std::vector<cv::Mat> R_gimbal2world_list, t_gimbal2world_list;
  std::vector<cv::Mat> rvecs, tvecs;
  ReprojectionProblem problem;
  load(
    input_folder, config_path, R_gimbal2imubody_data, R_gimbal2world_list, t_gimbal2world_list,
    rvecs, tvecs, display, problem);
  if (rvecs.size() < 5) {
    fmt::print(stderr, "有效手眼标定图像不足: {}（至少需要 5 张）\n", rvecs.size());
    return 2;
  }
  fmt::print("有效手眼标定图像: {}\n", rvecs.size());

  // 手眼标定
  cv::Mat R_camera2gimbal, t_camera2gimbal;
  cv::calibrateHandEye(
    R_gimbal2world_list, t_gimbal2world_list, rvecs, tvecs, R_camera2gimbal, t_camera2gimbal);
  if (!cli.has("no-refine") &&
      !refine(problem, R_camera2gimbal, t_camera2gimbal, rvecs, tvecs)) {
    fmt::print(stderr, "Joint reprojection refinement failed; no result emitted\n");
    return 3;
  }
  t_camera2gimbal /= 1e3;  // mm to m

  // 计算相机同理想情况的偏角
  Eigen::Matrix3d R_camera2gimbal_eigen;
  cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
  Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
  Eigen::Vector3d ypr = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;  // degree

  // 输出yaml
  print_yaml(R_gimbal2imubody_data, R_camera2gimbal, t_camera2gimbal, ypr);
}
