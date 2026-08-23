// SimpleProgressChecker — 简单进展检查模块。
//
// 纯 C++ (零 ROS): time_period 时间窗内从基线移动 required_movement 距离或
// movement_angle_required 转角才判有进展, 否则无进展 (卡住)。

#include <ayanami/modules/simple_progress_checker/simple_progress_checker.hpp>

#include <ayanami/utility/config.hpp>

#include <cmath>

namespace ayanami {
namespace modules {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

SimpleProgressChecker::SimpleProgressChecker() { loadConfig(); }

void SimpleProgressChecker::loadConfig() {
  try {
    const ayanami::ComponentConfig holder("config_progress_checker");
    const auto& file = holder.get();
    config_.required_movement =
        file.param<double>("progress_checker", "required_movement",
                           config_.required_movement);
    config_.movement_angle_required =
        file.param<double>("progress_checker", "movement_angle_required",
                           config_.movement_angle_required);
    config_.time_period = file.param<double>("progress_checker", "time_period",
                                             config_.time_period);
  } catch (...) {
    // 配置不可读时保持默认值。
  }
}

void SimpleProgressChecker::reset() {
  initialized_ = false;
  reset_pending_.store(true);
}

namespace {

// Eigen 四元数 → 2D 偏航角 [-pi, pi] (normalize 到偏航主值)。
double yawOf(const Eigen::Isometry3d& iso) {
  const Eigen::Quaterniond q(iso.rotation());
  double yaw = std::atan2(
      2.0 * (q.w() * q.z() + q.x() * q.y()),
      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  return yaw;
}

double normalizeAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

}  // namespace

bool SimpleProgressChecker::check(
    const ayanami::geometry_msgs::PoseStamped& current_pose) {
  if (!initialized_ || reset_pending_.load()) {
    base_pose_ = current_pose;
    base_time_ = std::chrono::steady_clock::now();
    initialized_ = true;
    reset_pending_.store(false);
    return true;
  }

  const Eigen::Vector3d delta =
      current_pose.pose.translation() - base_pose_.pose.translation();
  const double moved = delta.head<2>().norm();
  const double angle_moved =
      std::abs(normalizeAngle(yawOf(current_pose.pose) - yawOf(base_pose_.pose)));

  const double elapsed = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - base_time_)
                             .count();

  if (moved >= config_.required_movement ||
      angle_moved >= config_.movement_angle_required) {
    base_pose_ = current_pose;
    base_time_ = std::chrono::steady_clock::now();
    return true;
  }

  if (elapsed > config_.time_period) {
    return false;  // 无进展 (卡住)
  }
  return true;
}

}  // namespace modules
}  // namespace ayanami
