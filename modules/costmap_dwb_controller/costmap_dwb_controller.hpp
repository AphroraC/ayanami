// costmap_dwb_controller.hpp — nav2 dwb_core::DWBLocalPlanner 的 ayanami 迁移。
//
// 算法逻辑与 nav2_dwb_controller (dwb_core) 逐函数对应:
//   configure        → initialize(name, CostmapContext)
//   setPlan          → setPlan (Path → Path2D, reset critics + generator)
//   computeVelocityCommands → computeVelocityCommands (ayanami 签名)
//   prepareGlobalPlan / coreScoringAlgorithm / scoreTrajectory /
//   transformGlobalPlan / loadCritics 保留原名与语义
//
// 替换点 (零 ROS):
//   - pluginlib::ClassLoader → 静态注册表 (registration.hpp)
//   - tf2_ros::Buffer + nav_2d_utils::tf_help → TransformStore
//   - rclcpp 参数 → ayanami::Config (json 段 dwb_plugin_name, 键为点分全名)
//   - DWBPublisher (ROS 发布) → 删除 (调试可视化改 callback 槽可选, 先省略)
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d.hpp>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/utility/config.hpp>

#include "dwb_types.hpp"
#include "trajectory_critic.hpp"
#include "trajectory_generator.hpp"

namespace ayanami {
namespace modules {

using dwb_core::TrajectoryCritic;
using dwb_core::TrajectoryGenerator;

class CostmapDwBLocalPlanner : public ayanami::costmap::CostmapController {
public:
  using Ptr = std::shared_ptr<CostmapDwBLocalPlanner>;

  /**
   * @brief Constructor. 注册全部内置 critic / generator 到注册表。
   */
  CostmapDwBLocalPlanner();

  virtual ~CostmapDwBLocalPlanner() = default;

  /// @brief costmap 族初始化契约 (等价 nav2 configure): 读配置、创建
  /// trajectory generator 并 initialize、loadCritics。
  void initialize(const std::string& name,
                  ayanami::costmap::CostmapContext::Ptr ctx) override;

  /// @brief nav2 setPlan 等价。
  bool setPlan(const ayanami::nav_msgs::Path& plan) override;

  /// @brief nav2 computeVelocityCommands 等价 (ayanami 签名)。
  FailureCode computeVelocityCommands(
      const ayanami::geometry_msgs::PoseStamped& pose,
      const ayanami::geometry_msgs::TwistStamped& velocity,
      ayanami::geometry_msgs::TwistStamped& cmd_vel, std::string& message,
      const ayanami::abstract::AbstractGoalChecker::Ptr& goal_checker) override;

  bool cancel() override { return false; }

  /// @brief Score a given command. Can be used for testing.
  virtual dwb_msgs::TrajectoryScore scoreTrajectory(
      const dwb_msgs::Trajectory2D& traj, double best_score = -1);

  /// @brief Limits the maximum linear speed of the robot.
  void setSpeedLimit(const double& speed_limit, const bool& percentage) {
    if (traj_generator_) {
      traj_generator_->setSpeedLimit(speed_limit, percentage);
    }
  }

protected:
  /// @brief 2D 核心 (nav2 computeVelocityCommands 2D 版)。
  virtual nav_2d_msgs::Twist2DStamped computeVelocityCommands(
      const nav_2d_msgs::Pose2DStamped& pose,
      const nav_2d_msgs::Twist2D& velocity,
      std::shared_ptr<dwb_msgs::LocalPlanEvaluation>& results);

  void prepareGlobalPlan(const nav_2d_msgs::Pose2DStamped& pose,
                         nav_2d_msgs::Path2D& transformed_plan,
                         nav_2d_msgs::Pose2DStamped& goal_pose,
                         bool publish_plan = true);

  virtual dwb_msgs::TrajectoryScore coreScoringAlgorithm(
      const ::geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D velocity,
      std::shared_ptr<dwb_msgs::LocalPlanEvaluation>& results);

  virtual nav_2d_msgs::Path2D transformGlobalPlan(
      const nav_2d_msgs::Pose2DStamped& pose);

  /// @brief 等价 nav2 resolveCriticClassName (保留 suffix/命名空间补全逻辑)。
  std::string resolveCriticClassName(std::string base_name);

  virtual void loadCritics();

  nav_2d_msgs::Path2D global_plan_;  ///< Saved Global Plan
  bool prune_plan_;
  double prune_distance_;
  bool debug_trajectory_details_;
  double transform_tolerance_{0.1};
  bool shorten_transformed_plan_;
  double forward_prune_distance_;

  std::vector<std::string> default_critic_namespaces_;

  // Plugin handling (静态注册表)
  TrajectoryGenerator::Ptr traj_generator_;
  std::vector<TrajectoryCritic::Ptr> critics_;

  std::string dwb_plugin_name_;

  bool short_circuit_trajectory_evaluation_;

  // ayanami 注入
  ayanami::costmap::CostmapContext::Ptr ctx_;
  costmap_2d::Costmap2D* costmap_{nullptr};
  std::vector<::geometry_msgs::Point> footprint_spec_;
};

}  // namespace modules
}  // namespace ayanami
