// Task 23: modules/costmap_pid_controller — 3 轴 PID 局部控制器模块。
//
// 纯 C++ (零 ROS): 继承 costmap::CostmapController (costmap 族 controller,
// 只读 pose+path, 算法不读图但按族契约实现 initialize)。
// 参考 _legacy pid_controller 语义重写, 收尾历史五项控制修复:
// - computeVelocityCommands 入口 cmd_vel 强制清零 (脏内存 bug)
// - 差速模式仅 yaw 控制直接朝向 target (不取负号); 全向选最近车体方向角
// - 同 goal 重复 setPlan 不重置 PID 状态 (plan_signature 比较)
// - linear 按 max_acc/max_decel 分开限幅; angular 受 max_angular_accel 限幅
// - distanceDecay 临近 goal 减速; 位置进入 goal_checker XY 容差后 linear=0,
//   仅 angular PID 对齐 goal 朝向; 到达判定由 goal_checker 宣布 (isGoalReached
//   已从接口移除, 控制器自身不判到达)。

#include "costmap_pid_controller.hpp"

#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>

namespace ayanami {
namespace modules {

namespace {

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double shortestAngularDistance(double from, double to) {
  return normalizeAngle(to - from);
}

double yawOf(const geometry_msgs::PoseStamped& pose) {
  const Eigen::Matrix3d& r = pose.pose.linear();
  return std::atan2(r(1, 0), r(0, 0));
}

double distance2d(const geometry_msgs::PoseStamped& a,
                  const geometry_msgs::PoseStamped& b) {
  const Eigen::Vector3d& pa = a.pose.translation();
  const Eigen::Vector3d& pb = b.pose.translation();
  return std::hypot(pa.x() - pb.x(), pa.y() - pb.y());
}

// 粗路径签名 (3.5 minor carry): 仅末位 goal (x, y) × 1000 + path size, 不逐
// pose hash。re-smooth 后同 goal 同 size 的 plan 得到相同签名 → setPlan 不重置
// PID 状态 (防止 re-smooth 触发 PID reset stutter); 不同 goal 签名不同 (重置)。
uint64_t computePlanSignature(const nav_msgs::Path& plan) {
  if (plan.poses.empty()) {
    return 0;
  }
  auto& goal = plan.poses.back();
  int64_t gx =
      static_cast<int64_t>(std::round(goal.pose.translation().x() * 1000.0));
  int64_t gy =
      static_cast<int64_t>(std::round(goal.pose.translation().y() * 1000.0));
  return (std::hash<int64_t>()(gx) ^ std::hash<int64_t>()(gy)) ^
         (static_cast<uint64_t>(plan.poses.size()) << 4);
}

}  // namespace

void CostmapPidController::PidAxis::reset() {
  integral = 0.0;
  last_error = 0.0;
  has_last_error = false;
}

double CostmapPidController::PidAxis::compute(double error, double dt) {
  dt = std::max(dt, 1e-6);
  integral += error * dt;
  if (i_max > 0.0) {
    integral = std::clamp(integral, -i_max, i_max);
  }
  const double derivative = has_last_error ? (error - last_error) / dt : 0.0;
  last_error = error;
  has_last_error = true;
  return kp * error + ki * integral + kd * derivative;
}

CostmapPidController::CostmapPidController() { loadConfig(); }

void CostmapPidController::initialize(
    const std::string& name, ayanami::costmap::CostmapContext::Ptr ctx) {
  // 纯 path tracker 不读图: 满足 costmap 族契约 (spec §5.5) 但不持有地图。
  // config 在构造函数 loadConfig 完成, name/ctx 无需持久化。
  (void)name;
  (void)ctx;
}

void CostmapPidController::loadConfig() {
  try {
    const ayanami::ComponentConfig holder("config_controller");
    const auto& file = holder.get();
    config_.max_v = file.param<double>("costmap_pid_controller", "max_v",
                                       config_.max_v);
    config_.max_w = file.param<double>("costmap_pid_controller", "max_w",
                                       config_.max_w);
    config_.max_acc = file.param<double>("costmap_pid_controller", "max_acc",
                                         config_.max_acc);
    config_.max_decel = file.param<double>("costmap_pid_controller",
                                           "max_decel", config_.max_decel);
    config_.max_angular_accel =
        file.param<double>("costmap_pid_controller", "max_angular_accel",
                           config_.max_angular_accel);
    config_.kp_v = file.param<double>("costmap_pid_controller", "kp_v",
                                      config_.kp_v);
    config_.ki_v = file.param<double>("costmap_pid_controller", "ki_v",
                                      config_.ki_v);
    config_.kd_v = file.param<double>("costmap_pid_controller", "kd_v",
                                      config_.kd_v);
    config_.kp_w = file.param<double>("costmap_pid_controller", "kp_w",
                                      config_.kp_w);
    config_.ki_w = file.param<double>("costmap_pid_controller", "ki_w",
                                      config_.ki_w);
    config_.kd_w = file.param<double>("costmap_pid_controller", "kd_w",
                                      config_.kd_w);
    config_.dist_decay = file.param<double>("costmap_pid_controller",
                                            "dist_decay", config_.dist_decay);
    config_.goal_tolerance =
        file.param<double>("costmap_pid_controller", "goal_tolerance",
                           config_.goal_tolerance);
    config_.slide_window = file.param<double>("costmap_pid_controller",
                                              "slide_window",
                                              config_.slide_window);
    config_.default_dt = file.param<double>("costmap_pid_controller",
                                            "default_dt", config_.default_dt);
    config_.heading_deadband =
        file.param<double>("costmap_pid_controller", "heading_deadband",
                           config_.heading_deadband);
    config_.rotate_in_place_angle =
        file.param<double>("costmap_pid_controller", "rotate_in_place_angle",
                           config_.rotate_in_place_angle);
    config_.mode = file.param<std::string>("costmap_pid_controller", "mode",
                                           config_.mode);
  } catch (...) {
    // 配置不可读时保持默认值。
  }

  pid_x_.kp = config_.kp_v;
  pid_x_.ki = config_.ki_v;
  pid_x_.kd = config_.kd_v;
  pid_y_.kp = config_.kp_v;
  pid_y_.ki = config_.ki_v;
  pid_y_.kd = config_.kd_v;
  pid_w_.kp = config_.kp_w;
  pid_w_.ki = config_.ki_w;
  pid_w_.kd = config_.kd_w;
}

bool CostmapPidController::setPlan(const nav_msgs::Path& plan) {
  if (plan.poses.empty()) {
    return false;
  }
  const uint64_t signature = computePlanSignature(plan);
  plan_ = plan;
  goal_ = plan.poses.back();
  canceled_.store(false);
  if (signature != plan_signature_) {
    // 新 goal (或首帧): 重置跟踪状态, 确保 setPlan 首帧从 0 起步 (不携带旧速度)。
    plan_signature_ = signature;
    resetTrackingState();
    last_track_idx_ = 1;
  }
  return true;
}

FailureCode CostmapPidController::computeVelocityCommands(
    const geometry_msgs::PoseStamped& pose,
    const geometry_msgs::TwistStamped& velocity,
    geometry_msgs::TwistStamped& cmd_vel, std::string& message,
    const ayanami::abstract::AbstractGoalChecker::Ptr& goal_checker) {
  // 历史脏内存 bug: 入口强制清零后填充。
  cmd_vel = geometry_msgs::TwistStamped{};
  cmd_vel.twist = geometry_msgs::Twist{};
  cmd_vel.header = pose.header;
  message.clear();

  if (plan_.poses.empty()) {
    message = "PID controller has no plan";
    return FailureCode::NoValidCommand;
  }
  if (canceled_.load()) {
    resetTrackingState();
    message = "PID controller canceled";
    return FailureCode::Canceled;
  }

  last_pose_ = pose;
  const double dt = std::max(config_.default_dt, 1e-6);

  // 到达判定由 goal_checker 全权负责: PID 只读其容差 (position/angle 字段)。
  // 空指针防御: 回退到 config 的 goal_tolerance (仅作停线速度的 XY 边界)。
  const double xy_tol =
      goal_checker ? goal_checker->getTolerances().pose.position
                   : config_.goal_tolerance;

  // ---- 位置已进入 goal XY 容差: 停止线速度, 仅 angular PID 对齐 goal 朝向 ----
  // (goal_checker 在 execution 层最终宣布到达)
  if (distance2d(last_pose_, goal_) < xy_tol) {
    const double yaw_err =
        shortestAngularDistance(yawOf(pose), yawOf(goal_));
    const double target_theta =
        std::abs(yaw_err) < config_.heading_deadband ? 0.0 : yaw_err;
    double angular_w = pid_w_.compute(target_theta, dt);
    angular_w = std::clamp(angular_w, -config_.max_w, config_.max_w);
    angular_w = limitAcceleration(last_angular_vel_, angular_w,
                                  config_.max_angular_accel,
                                  config_.max_angular_accel, dt);
    last_angular_vel_ = angular_w;
    last_linear_vel_ = 0.0;
    cmd_vel.twist.linear.x() = 0.0;
    cmd_vel.twist.linear.y() = 0.0;
    cmd_vel.twist.angular.z() = angular_w;
    message = "PID controller reached goal position, aligning heading";
    return FailureCode::None;
  }

  // ---- 选择局部跟踪目标 (最近点 + lookahead 窗口) ----
  const double robot_yaw = yawOf(pose);
  size_t nearest = 0;
  double min_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < plan_.poses.size(); ++i) {
    const double d = distance2d(pose, plan_.poses[i]);
    if (d < min_dist) {
      min_dist = d;
      nearest = i;
    }
  }
  // 向前推进到第一个 >= slide_window 的点 (或末点), 避免在路径点处
  // lookahead 窗口塌缩导致目标距离≈0 的停车停顿。
  size_t target_idx = nearest;
  while (target_idx + 1 < plan_.poses.size() &&
         distance2d(pose, plan_.poses[target_idx]) < config_.slide_window) {
    ++target_idx;
  }
  last_track_idx_ = target_idx;

  const Eigen::Vector3d rel =
      plan_.poses[target_idx].pose.translation() - pose.pose.translation();
  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);
  const double local_x = c * rel.x() + s * rel.y();   // 前向
  const double local_y = -s * rel.x() + c * rel.y();  // 左向

  double target_theta = std::atan2(local_y, local_x);
  if (config_.mode == "omni") {
    // 全向底盘: 选最近车体运动方向角 (0 / ±90° / 180°)。
    std::map<double, double> candidates;
    for (const double reference : {0.0, kPi / 2.0, kPi, -kPi / 2.0}) {
      const double diff = shortestAngularDistance(reference, target_theta);
      candidates.emplace(std::abs(diff), diff);
    }
    target_theta = candidates.begin()->second;
  }
  // 差速底盘: 仅 yaw 控制直接朝向 target (不取负号)。
  if (std::abs(target_theta) < config_.heading_deadband) {
    target_theta = 0.0;
  }

  // ---- 三轴 PID ----
  double linear_x = 0.0;
  double linear_y = 0.0;
  if (config_.mode == "omni") {
    linear_x = pid_x_.compute(local_x, dt);
    linear_y = pid_y_.compute(local_y, dt);
  } else {
    const double target_dist = std::hypot(local_x, local_y);
    const double heading_scale =
        std::abs(target_theta) > config_.rotate_in_place_angle
            ? 0.0
            : std::max(0.0, std::cos(target_theta));
    linear_x = pid_x_.compute(target_dist, dt) * heading_scale;
    linear_y = 0.0;
  }
  double angular_w = pid_w_.compute(target_theta, dt);

  // 线速度上限。
  double cur_v = std::hypot(linear_x, linear_y);
  if (cur_v > config_.max_v) {
    const double k = config_.max_v / cur_v;
    linear_x *= k;
    linear_y *= k;
    cur_v = config_.max_v;
  }

  // 角速度: max_w 钳制 + max_angular_accel 限幅。
  angular_w = std::clamp(angular_w, -config_.max_w, config_.max_w);
  angular_w = limitAcceleration(last_angular_vel_, angular_w,
                                config_.max_angular_accel,
                                config_.max_angular_accel, dt);
  last_angular_vel_ = angular_w;

  // 线速度: distanceDecay 临近 goal 减速 + max_acc/max_decel 分开限幅。
  double desired_vel = 0.0;
  if (cur_v > 1e-9) {
    const double goal_dist = distance2d(pose, goal_);
    const double decayed = distanceDecay(goal_dist, cur_v);
    desired_vel = limitAcceleration(last_linear_vel_, decayed,
                                    config_.max_acc, config_.max_decel, dt);
    const double coeff = desired_vel / cur_v;
    linear_x *= coeff;
    linear_y *= coeff;
    last_linear_vel_ = desired_vel;
  } else {
    last_linear_vel_ = 0.0;
  }

  cmd_vel.twist.linear.x() = linear_x;
  cmd_vel.twist.linear.y() = linear_y;
  cmd_vel.twist.angular.z() = angular_w;

  message = "PID controller generated velocity command";
  return FailureCode::None;
}

bool CostmapPidController::cancel() {
  canceled_.store(true);
  return true;
}

void CostmapPidController::resetTrackingState() {
  pid_x_.reset();
  pid_y_.reset();
  pid_w_.reset();
  last_linear_vel_ = 0.0;
  last_angular_vel_ = 0.0;
}

double CostmapPidController::limitAcceleration(double last_vel, double cmd_vel,
                                               double max_acc, double max_decel,
                                               double dt) const {
  // 加速用 max_acc, 减速用 max_decel 分开限幅。
  const double limit =
      cmd_vel >= last_vel ? std::max(0.0, max_acc) : std::max(0.0, max_decel);
  const double max_change = limit * std::max(dt, 0.0);
  return std::max(last_vel - max_change,
                  std::min(cmd_vel, last_vel + max_change));
}

double CostmapPidController::distanceDecay(double distance,
                                           double target_velocity) const {
  if (distance < config_.dist_decay) {
    return (distance / std::max(config_.dist_decay, 1e-9)) * target_velocity;
  }
  return target_velocity;
}

}  // namespace modules
}  // namespace ayanami
