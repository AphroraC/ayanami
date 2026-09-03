// SimpleProgressChecker — 简单进展检查模块。
//
// 纯 C++ (零 ROS): time_period 时间窗内从基线移动 required_movement 距离或
// movement_angle_required 转角才判有进展, 否则无进展 (卡住)。

#include "simple_progress_checker.hpp"

#include <ayanami/utility/config.hpp>

#include <cmath>

namespace ayanami {
namespace modules {

namespace {
constexpr double k_pi = 3.14159265358979323846;
}

SimpleProgressChecker::SimpleProgressChecker() {
  // 参数在构造时经 glim 式显式 Config 读取并拷贝进成员 (无 thread_local 注入,
  // 配置不可读时保持默认值): 主 config.json 的 global 单层索引
  // config_progress_checker → 本插件 json 文件。
  try {
    const Config file(GlobalConfig::getConfigPath("config_progress_checker"));
    param.required_movement =
        file.param<double>("progress_checker", "required_movement",
                           param.required_movement);
    param.movement_angle_required =
        file.param<double>("progress_checker", "movement_angle_required",
                           param.movement_angle_required);
    param.time_period = file.param<double>("progress_checker", "time_period",
                                           param.time_period);
  } catch (...) {
    // 配置不可读时保持默认值。
  }
}

void SimpleProgressChecker::reset() {
  initialized = false;
  reset_pending.store(true);
}

namespace {

// Eigen 四元数 → 2D 偏航角 [-pi, pi] (normalize 到偏航主值)。
double yawOf(const ayanami::geometry_msgs::Pose& pose) {
  const Eigen::Quaterniond& q = pose.orientation;
  double yaw = std::atan2(
      2.0 * (q.w() * q.z() + q.x() * q.y()),
      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  return yaw;
}

double normalizeAngle(double angle) {
  while (angle > k_pi) angle -= 2.0 * k_pi;
  while (angle < -k_pi) angle += 2.0 * k_pi;
  return angle;
}

}  // namespace

bool SimpleProgressChecker::check(
    const ayanami::geometry_msgs::PoseStamped& current_pose) {
  if (!initialized || reset_pending.load()) {
    base_pose = current_pose;
    base_time = std::chrono::steady_clock::now();
    initialized = true;
    reset_pending.store(false);
    return true;
  }

  const Eigen::Vector3d delta =
      current_pose.pose.position - base_pose.pose.position;
  const double moved = delta.head<2>().norm();
  const double angle_moved =
      std::abs(normalizeAngle(yawOf(current_pose.pose) - yawOf(base_pose.pose)));

  const double elapsed = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - base_time)
                             .count();

  if (moved >= param.required_movement ||
      angle_moved >= param.movement_angle_required) {
    base_pose = current_pose;
    base_time = std::chrono::steady_clock::now();
    return true;
  }

  if (elapsed > param.time_period) {
    return false;  // 无进展 (卡住)
  }
  return true;
}

}  // namespace modules
}  // namespace ayanami
