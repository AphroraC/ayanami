// costmap_dwb_controller.hpp — nav2 dwb_core::DWBLocalPlanner 的 ayanami 迁移。
//
// 算法逻辑与 nav2_dwb_controller (dwb_core) 逐函数对应:
//   configure        → configure(PluginContext, map)
//   setPlan          → setPlan (Path → Path2D, reset critics + generator)
//   computeVelocityCommands → computeVelocityCommand (abstract 签名;
//                             插件经持有的地图模块加锁读 local grid)
//   prepareGlobalPlan / coreScoringAlgorithm / scoreTrajectory /
//   transformGlobalPlan / loadCritics 保留原名与语义
//
// 锁纪律: 每个控制周期经地图模块 lockLocal() 持 shared 锁直读 local grid。控制器在周期内持有视图,
// 快照 (shared_ptr) 使 grid 存活整个打分周期, 并在 prepare 前把当前 grid /
// footprint 绑定到各 critic (critic::setCostmap)。不持任何 map 写锁 ——
// 移除旧实现的 ctx->local_costmap->mutex() 内部锁。
//
// 替换点 (零 ROS):
//   - pluginlib::ClassLoader → 静态注册表 (registration.hpp)
//   - tf2_ros::Buffer + nav_2d_utils::tf_help → TransformStore 单例
//   - rclcpp 参数 → ayanami::Config (json 段 dwb_plugin_name, 键为点分全名)
//   - DWBPublisher (ROS 发布) → 删除 (调试可视化改 callback 槽可选, 先省略)
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d.hpp>

#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/core/costmap_view.hpp>
#include <ayanami/utility/config.hpp>

#include "dwb_types.hpp"
#include "trajectory_critic.hpp"
#include "trajectory_generator.hpp"

namespace ayanami {
namespace modules {

using dwb_core::TrajectoryCritic;
using dwb_core::TrajectoryGenerator;

class CostmapDwBLocalPlanner final : public ayanami::costmap::CostmapController {
public:
  using Ptr = std::shared_ptr<CostmapDwBLocalPlanner>;

  /**
   * @brief Constructor. 注册全部内置 critic / generator 到注册表。
   */
  CostmapDwBLocalPlanner();

  virtual ~CostmapDwBLocalPlanner() = default;

  /// @brief costmap 族注入契约 (等价 nav2 configure): 经 PluginContext 读
  /// 自身配置、创建 trajectory generator 并 initialize、loadCritics。
  void configure(const PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  /// @brief nav2 setPlan 等价。
  bool setPlan(const ayanami::nav_msgs::Path& plan) override;

  /// @brief nav2 computeVelocityCommands 等价 (abstract 签名; 内部加锁读图)。
  ControllerResultT computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped& pose,
      const ayanami::geometry_msgs::TwistStamped& velocity) override;

  bool cancel() override { return false; }

  /// @brief Score a given command. Can be used for testing.
  virtual dwb_msgs::TrajectoryScore scoreTrajectory(
      const dwb_msgs::Trajectory2D& traj, double best_score = -1);

  /// @brief Limits the maximum linear speed of the robot.
  void setSpeedLimit(const double& speed_limit, const bool& percentage) {
    if (traj_generator) {
      traj_generator->setSpeedLimit(speed_limit, percentage);
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

  /// @brief 模块参数 (json 段 = controller.name; configure 时经
  /// PluginContext 注入的自身配置读取并拷贝进成员)。
  struct CostmapDwBLocalPlannerParam {
    double transform_tolerance{0.1};
    bool prune_plan{true};
    double prune_distance{2.0};
    double forward_prune_distance{2.0};
    bool debug_trajectory_details{false};
    bool short_circuit_trajectory_evaluation{true};
    bool shorten_transformed_plan{true};
    std::string trajectory_generator_name{
        "dwb_plugins::StandardTrajectoryGenerator"};
    std::vector<std::string> default_critic_namespaces;
    std::vector<std::string> critics;
  };

  nav_2d_msgs::Path2D global_plan;  ///< Saved Global Plan
  CostmapDwBLocalPlannerParam param;

  /// @brief 本插件配置 (configure 时由 PluginContext 移入; loadCritics
  /// 复用同一实例, 无 thread_local 注入)。
  std::unique_ptr<ayanami::Config> config_;

  std::vector<TrajectoryCritic::Ptr> critics;

  // Plugin handling (静态注册表)
  TrajectoryGenerator::Ptr traj_generator;

  std::string dwb_plugin_name;

  // ayanami 注入 (地图模块 const 指针 + 当前周期持锁读视图)
  ayanami::costmap::CostmapMapContext::ConstPtr map_;
  ayanami::costmap::CostmapView local_view;
  const costmap_2d::Costmap2D* local_grid{nullptr};
  std::string local_frame;
  std::vector<::geometry_msgs::Point> footprint_spec;
};

}  // namespace modules
}  // namespace ayanami
