#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 3 轴 PID 局部控制器模块 (spec §5.5 controller costmap 族)。
 *
 * 纯 path tracker: 只读 robot pose + path, 算法本身不读图; 但按 costmap 族
 * 严格约束继承 costmap::CostmapController 并实现 configure(context, map) +
 * computeVelocityCommand (costmap 宿主只接受 Costmap* 类型)。参考 _legacy
 * pid_controller 重写。
 *
 * 历史五项控制修复收尾:
 * - computeVelocityCommand 入口 cmd_vel 强制清零 (脏内存 bug)
 * - 差速模式仅 yaw 控制直接朝向 target (不取负号); 全向选最近车体方向角
 * - 同 goal 重复 setPlan 不重置 PID 状态 (plan_signature 比较)
 * - linear 按 max_acc_x/max_decel_x 分开限幅; angular 受 max_acc_theta 限幅
 * - distanceDecay 临近 goal 减速; 位置进入 param.goal_tolerance 的 XY 容差后
 *   线速度置零, 仅 angular PID 对齐 goal 朝向; 到达判定完全交给 execution 层
 *   goal_checker (控制器自身不再宣布到达, 不接收 goal_checker 参数)。
 */
class CostmapPidController final : public ayanami::costmap::CostmapController {
 public:
  using Ptr = std::shared_ptr<CostmapPidController>;

  CostmapPidController();

  virtual ~CostmapPidController() = default;

  /// @brief costmap 族注入契约。算法不读图, 仅持有地图模块指针; 参数经
  /// PluginContext 注入的自身配置读取。
  void configure(const PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  ControllerResultT computeVelocityCommand(
      const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity) override;

  bool setPlan(const nav_msgs::Path& plan) override;

  bool cancel() override;

 private:
  struct CostmapPidControllerParam {
    double max_vel_x = 2.0;
    double max_vel_theta = 1.0;
    double max_acc_x = 1.0;
    double max_decel_x = 1.0;
    double max_acc_theta = 1.0;
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

  void resetTrackingState();
  double limitAcceleration(double last_vel, double cmd_vel, double max_acc,
                           double max_decel, double dt) const;
  double distanceDecay(double distance, double target_velocity) const;

  CostmapPidControllerParam param;
  nav_msgs::Path plan;
  geometry_msgs::PoseStamped goal;
  geometry_msgs::PoseStamped last_pose;
  PidAxis pid_x;
  PidAxis pid_y;
  PidAxis pid_w;
  double last_linear_vel = 0.0;
  double last_angular_vel = 0.0;
  size_t last_track_idx = 1;
  uint64_t plan_signature = 0;
  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
