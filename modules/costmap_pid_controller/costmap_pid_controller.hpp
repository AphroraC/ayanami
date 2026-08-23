#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 3 轴 PID 局部控制器模块 (spec §5.5 controller costmap 族)。
 *
 * 纯 path tracker: 只读 robot pose + path, 算法本身不读图; 但按 costmap 族
 * 严格约束继承 costmap::CostmapController 并实现 initialize(name,
 * CostmapContext::Ptr) (costmap 宿主只接受 Costmap* 类型)。参考 _legacy
 * pid_controller 重写。
 *
 * 历史五项控制修复收尾:
 * - computeVelocityCommands 入口 cmd_vel 强制清零 (脏内存 bug)
 * - 差速模式仅 yaw 控制直接朝向 target (不取负号); 全向选最近车体方向角
 * - 同 goal 重复 setPlan 不重置 PID 状态 (plan_signature 比较)
 * - linear 按 max_acc/max_decel 分开限幅; angular 受 max_angular_accel 限幅
 * - distanceDecay 临近 goal 减速; 位置进入 goal_checker 的 XY 容差后线速度置零,
 *   仅 angular PID 对齐 goal 朝向; 到达判定完全交给 goal_checker
 *   (getTolerances 的 position/angle 字段), 控制器自身不再宣布到达。
 */
class CostmapPidController : public ayanami::costmap::CostmapController {
 public:
  using Ptr = std::shared_ptr<CostmapPidController>;

  CostmapPidController();

  virtual ~CostmapPidController() = default;

  /// @brief costmap 族初始化契约。算法不读图, 仅标记已注入 (配置仍在构造期
  /// loadConfig)。@param ctx 经 costmap 宿主注入的 CostmapContext。
  void initialize(const std::string& name,
                  ayanami::costmap::CostmapContext::Ptr ctx) override;

  FailureCode computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                      const geometry_msgs::TwistStamped& velocity,
                                      geometry_msgs::TwistStamped& cmd_vel,
                                      std::string& message,
                                      const ayanami::abstract::AbstractGoalChecker::Ptr& goal_checker) override;

  bool setPlan(const nav_msgs::Path& plan) override;

  bool cancel() override;

 private:
  struct Config {
    double max_v = 2.0;
    double max_w = 1.0;
    double max_acc = 1.0;
    double max_decel = 1.0;
    double max_angular_accel = 1.0;
    double kp_v = 1.0;
    double ki_v = 0.0;
    double kd_v = 0.0;
    double kp_w = 1.0;
    double ki_w = 0.0;
    double kd_w = 0.0;
    double dist_decay = 2.0;
    double goal_tolerance = 0.1;
    double slide_window = 1.0;
    double default_dt = 0.05;
    double heading_deadband = 0.08;
    double rotate_in_place_angle = 0.45;
    std::string mode = "diff";  // "omni" | "diff"
  };

  // 单轴 PID: 持 integral/last_error (reset 由 resetTrackingState 统一触发)。
  struct PidAxis {
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.0;
    double i_max = 1.0;
    double integral = 0.0;
    double last_error = 0.0;
    bool has_last_error = false;

    void reset();
    double compute(double error, double dt);
  };

  void loadConfig();
  void resetTrackingState();
  double limitAcceleration(double last_vel, double cmd_vel, double max_acc,
                           double max_decel, double dt) const;
  double distanceDecay(double distance, double target_velocity) const;

  Config config_;
  nav_msgs::Path plan_;
  geometry_msgs::PoseStamped goal_;
  geometry_msgs::PoseStamped last_pose_;
  PidAxis pid_x_;
  PidAxis pid_y_;
  PidAxis pid_w_;
  double last_linear_vel_ = 0.0;
  double last_angular_vel_ = 0.0;
  size_t last_track_idx_ = 1;
  uint64_t plan_signature_ = 0;
  std::atomic<bool> canceled_{false};
};

}  // namespace modules
}  // namespace ayanami
