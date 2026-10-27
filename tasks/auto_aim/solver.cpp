#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
constexpr double LIGHTBAR_LENGTH = 56e-3;     // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

namespace
{
bool finite_points(const std::vector<cv::Point2f> & points)
{
  return points.size() == 4 && std::all_of(points.begin(), points.end(), [](const auto & point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
  });
}

bool valid_image_quad(const std::vector<cv::Point2f> & points)
{
  if (!finite_points(points)) return false;
  double max_distance_squared = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    for (std::size_t j = i + 1; j < 4; ++j) {
      const cv::Point2d difference = cv::Point2d(points[i]) - cv::Point2d(points[j]);
      max_distance_squared = std::max(max_distance_squared, difference.dot(difference));
    }
  }
  if (max_distance_squared <= 1e-8) return false;

  // Use relative area so this only rejects numerical degeneracy, without adding
  // a distance-dependent pixel-size gate. A crossing/concave quad is not an armor.
  const double minimum_cross = 1e-6 * max_distance_squared;
  double previous_cross = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    const cv::Point2d edge = cv::Point2d(points[(i + 1) % 4]) - cv::Point2d(points[i]);
    const cv::Point2d next = cv::Point2d(points[(i + 2) % 4]) - cv::Point2d(points[(i + 1) % 4]);
    const double cross = edge.x * next.y - edge.y * next.x;
    if (!std::isfinite(cross) || std::abs(cross) <= minimum_cross) return false;
    if (i > 0 && std::signbit(cross) != std::signbit(previous_cross)) return false;
    previous_cross = cross;
  }
  return true;
}

void invalidate_pose(Armor & armor)
{
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  armor.xyz_in_gimbal.setConstant(invalid);
  armor.xyz_in_world.setConstant(invalid);
  armor.ypr_in_gimbal.setConstant(invalid);
  armor.ypr_in_world.setConstant(invalid);
  armor.ypd_in_world.setConstant(invalid);
  armor.yaw_raw = invalid;
}

bool finite_pose(const Armor & armor)
{
  return armor.xyz_in_gimbal.allFinite() && armor.xyz_in_world.allFinite() &&
    armor.ypr_in_gimbal.allFinite() && armor.ypr_in_world.allFinite() &&
    armor.ypd_in_world.allFinite() && std::isfinite(armor.yaw_raw);
}
}  // namespace

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  if (R_gimbal2imubody_data.size() != 9 || R_camera2gimbal_data.size() != 9 ||
      t_camera2gimbal_data.size() != 3) {
    throw std::runtime_error("extrinsic rotations/translations must have 9/3 elements");
  }
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

  // 验证数据大小
  if (camera_matrix_data.size() != 9) {
    throw std::runtime_error("camera_matrix must have exactly 9 elements");
  }
  if (distort_coeffs_data.size() != 5) {
    throw std::runtime_error("distort_coeffs must have exactly 5 elements");
  }

  // 将数据复制到成员数组中（确保数据生命周期）
  std::copy(camera_matrix_data.begin(), camera_matrix_data.end(), camera_matrix_data_.begin());
  std::copy(distort_coeffs_data.begin(), distort_coeffs_data.end(), distort_coeffs_data_.begin());

  // 创建 cv::Mat 包装器，指向成员数组数据
  camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_data_.data());
  distort_coeffs_ = cv::Mat(1, 5, CV_64F, distort_coeffs_data_.data());
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

bool Solver::valid_camera_pose(
  const cv::Vec3d & rvec, const cv::Vec3d & tvec, ArmorType type)
{
  if (type != ArmorType::small && type != ArmorType::big) return false;
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(rvec[i]) || !std::isfinite(tvec[i])) return false;
  }
  constexpr double minimum_depth_m = 1e-6;
  if (tvec[2] <= minimum_depth_m) return false;
  try {
    cv::Matx33d rotation;
    cv::Rodrigues(rvec, rotation);
    for (const double value : rotation.val) {
      if (!std::isfinite(value)) return false;
    }
    const auto & object_points = type == ArmorType::big ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    for (const auto & point : object_points) {
      const cv::Vec3d camera_point = rotation * cv::Vec3d(point.x, point.y, point.z) + tvec;
      if (!std::isfinite(camera_point[0]) || !std::isfinite(camera_point[1]) ||
          !std::isfinite(camera_point[2]) || camera_point[2] <= minimum_depth_m) return false;
    }
    return true;
  } catch (const cv::Exception &) {
    return false;
  }
}

// Raw PnP is shared by normal solving and the outpost diagnostic.
bool Solver::solve_pose(Armor & armor) const
{
  if (!valid_image_quad(armor.points) ||
      (armor.type != ArmorType::small && armor.type != ArmorType::big) ||
      !R_camera2gimbal_.allFinite() || !t_camera2gimbal_.allFinite() ||
      !R_gimbal2world_.allFinite() || !cv::checkRange(camera_matrix_) ||
      !cv::checkRange(distort_coeffs_) || camera_matrix_data_[0] <= 0.0 ||
      camera_matrix_data_[4] <= 0.0) return false;
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec(0, 0, 0), tvec(0, 0, 0);
  if (!cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE) || !valid_camera_pose(rvec, tvec, armor.type)) return false;

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  armor.yaw_raw = armor.ypr_in_world[0];
  return finite_pose(armor);
}

bool Solver::solve(Armor & armor) const
{
  invalidate_pose(armor);
  Armor candidate = armor;
  try {
    if (!solve_pose(candidate)) return false;

    // 平衡不做yaw优化，因为pitch假设不成立
    const bool is_balance = (candidate.type == ArmorType::big) &&
      (candidate.name == ArmorName::three || candidate.name == ArmorName::four ||
       candidate.name == ArmorName::five);
    if (!is_balance && !optimize_yaw(candidate)) return false;
    if (!finite_pose(candidate)) return false;

    armor.xyz_in_gimbal = candidate.xyz_in_gimbal;
    armor.xyz_in_world = candidate.xyz_in_world;
    armor.ypr_in_gimbal = candidate.ypr_in_gimbal;
    armor.ypr_in_world = candidate.ypr_in_world;
    armor.ypd_in_world = candidate.ypd_in_world;
    armor.yaw_raw = candidate.yaw_raw;
    return true;
  } catch (const cv::Exception &) {
    return false;
  }
}

std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name,
  const Eigen::Matrix3d * gimbal2world) const
{
  const auto & rotation = gimbal2world ? *gimbal2world : R_gimbal2world_;
  if (!xyz_in_world.allFinite() || !std::isfinite(yaw) || !rotation.allFinite() ||
      !R_camera2gimbal_.allFinite() || !t_camera2gimbal_.allFinite()) return {};
  try {
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // clang-format off
    const Eigen::Matrix3d R_armor2world {
      {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
      {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
      {         -sin_pitch,        0,           cos_pitch}
    };
    // clang-format on

    // get R_armor2camera t_armor2camera
    const Eigen::Vector3d & t_armor2world = xyz_in_world;
    Eigen::Matrix3d R_armor2camera =
      R_camera2gimbal_.transpose() * rotation.transpose() * R_armor2world;
    Eigen::Vector3d t_armor2camera =
      R_camera2gimbal_.transpose() * (rotation.transpose() * t_armor2world - t_camera2gimbal_);

    // get rvec tvec
    cv::Vec3d rvec;
    cv::Mat R_armor2camera_cv;
    cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, rvec);
    cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);
    if (!valid_camera_pose(rvec, tvec, type)) return {};

    // reproject
    std::vector<cv::Point2f> image_points;
    const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
    if (!finite_points(image_points)) return {};
    return image_points;
  } catch (const cv::Exception &) {
    return {};
  }
}

double Solver::oupost_reprojection_error(Armor armor, const double & pitch)
{
  const double invalid_error = std::numeric_limits<double>::infinity();
  if (!std::isfinite(pitch)) return invalid_error;
  try {
    if (!solve_pose(armor)) return invalid_error;
    const auto & object_points =
      (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

    auto yaw = armor.ypr_in_world[0];
    auto xyz_in_world = armor.xyz_in_world;

    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // clang-format off
    const Eigen::Matrix3d _R_armor2world {
      {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
      {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
      {         -sin_pitch,        0,           cos_pitch}
    };
    // clang-format on

    // get R_armor2camera t_armor2camera
    const Eigen::Vector3d & t_armor2world = xyz_in_world;
    Eigen::Matrix3d _R_armor2camera =
      R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
    Eigen::Vector3d t_armor2camera =
      R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    // get rvec tvec
    cv::Vec3d _rvec;
    cv::Mat R_armor2camera_cv;
    cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, _rvec);
    cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);
    if (!valid_camera_pose(_rvec, _tvec, armor.type)) return invalid_error;

    // reproject
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);
    if (!finite_points(image_points)) return invalid_error;

    auto error = 0.0;
    for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
    return error;
  } catch (const cv::Exception &) {
    return invalid_error;
  }
}

bool Solver::optimize_yaw(Armor & armor) const
{
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  constexpr double SEARCH_RANGE = 140;  // degree
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  auto min_error = std::numeric_limits<double>::infinity();
  auto best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
  return std::isfinite(min_error) && std::isfinite(best_yaw);
}

double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  if (!finite_points(image_points) || !finite_points(armor.points))
    return std::numeric_limits<double>::infinity();
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

// 世界坐标到像素坐标的转换
// std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
// {
//   Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
//   Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

//   cv::Mat R_world2camera_cv;
//   cv::Mat rvec;
//   cv::Mat tvec;
//   cv::eigen2cv(R_world2camera, R_world2camera_cv);
//   cv::Rodrigues(R_world2camera_cv, rvec);
//   cv::eigen2cv(t_world2camera, tvec);

//   std::vector<cv::Point3f> valid_world_points;
//   for (const auto & world_point : worldPoints) {
//     Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
//     Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

//     if (camera_point.z() > 0) {
//       valid_world_points.push_back(world_point);
//     }
//   }
//   // 如果没有有效点，返回空vector
//   if (valid_world_points.empty()) {
//     return std::vector<cv::Point2f>();
//   }
//   std::vector<cv::Point2f> pixelPoints;
//   cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
//   return pixelPoints;
// }
// }  // namespace auto_aim


// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}
}  // namespace auto_aim
