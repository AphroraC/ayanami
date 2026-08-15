#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/components/navigation_server_base.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"
#include "ayanami/costmap/components/costmap_controller_execution.hpp"
#include "ayanami/costmap/components/costmap_planner_execution.hpp"
#include "ayanami/costmap/components/costmap_recovery_execution.hpp"
#include "ayanami/costmap/core/costmap_controller.hpp"
#include "ayanami/costmap/core/costmap_planner.hpp"
#include "ayanami/costmap/core/costmap_recovery.hpp"
#include "ayanami/costmap/core/costmap_smoother.hpp"
#include "ayanami/messages/sensor_msgs.hpp"

namespace costmap_2d_layers {
class SimpleObstacleLayer;
}

namespace ayanami {
namespace costmap {

/**
 * @brief costmap 特化的导航服务 (spec §5.4, 对标 MBF navfn/planner 组合)。
 *
 * 在 NavigationServerBase (无地图, spec §5.3) 之上:
 * - 覆写虚 initialize*Plugin: dynamic_pointer_cast 到 costmap 族接口
 *   (CostmapPlanner/Smoother/Controller/Recovery) 并注入 costmap_context_;
 *   失败 (非 costmap 族插件) spdlog::error + return false (不入表)。
 * - 覆写虚 new*Execution 工厂: 返回带 CostmapWrapper 的 Costmap*Execution
 *   (planner 绑 global / controller 绑 local / recovery 绑 global+local)。
 * - configureCoreMaps(config_path) 读 config_costmap 构 global/local
 *   LayeredCostmap: global = StaticLayer+InflationLayer;
 *   local = rolling SimpleObstacleLayer+InflationLayer。
 * - 构造期注册 NavigationCallbacks::on_obstacle_pointcloud/laserscan →
 *   handleObstacle* 经 TransformStore 把传感器帧点落 SimpleObstacleLayer。
 *
 * Ruling #7 (load-bearing): 动态障碍更新前 clearArea 上一帧观察窗口 (含旧
 * 膨胀带), 让 InflationLayer 在 recompute window 内重算, 障碍移走后无 stale
 * 膨胀带残留。
 */
class CostmapNavigationServer
    : public ayanami::components::NavigationServerBase {
 public:
  using Ptr = std::shared_ptr<CostmapNavigationServer>;

  explicit CostmapNavigationServer();
  ~CostmapNavigationServer() override;

  /// @brief 读 config_costmap 构 global/local costmap (StaticLayer +
  /// InflationLayer + SimpleObstacleLayer), 并注入 map_context_。
  void configureCoreMaps(const std::string& config_path);

  // ---- 虚 initialize*Plugin 覆写: costmap 族注入 ----
  bool initializePlannerPlugin(
      const std::string& name, ayanami::abstract::AbstractPlanner::Ptr) override;
  bool initializeSmootherPlugin(
      const std::string& name, ayanami::abstract::AbstractSmoother::Ptr) override;
  bool initializeControllerPlugin(
      const std::string& name,
      ayanami::abstract::AbstractController::Ptr) override;
  bool initializeRecoveryPlugin(
      const std::string& name, ayanami::abstract::AbstractRecovery::Ptr) override;

  // ---- 虚 new*Execution 工厂覆写: 返回带地图的 Costmap 版 ----
  std::unique_ptr<ayanami::components::PlannerExecution> newPlannerExecution(
      const std::string& name,
      ayanami::abstract::AbstractPlanner::Ptr) override;
  std::unique_ptr<ayanami::components::ControllerExecution>
  newControllerExecution(
      const std::string& name,
      ayanami::abstract::AbstractController::Ptr) override;
  std::unique_ptr<ayanami::components::RecoveryExecution> newRecoveryExecution(
      const std::string& name,
      ayanami::abstract::AbstractRecovery::Ptr) override;

  // ---- 障碍观测信号接入口 ----
  /// @brief 把点云 (sensor frame) 经 TransformStore 转到 local costmap frame,
  /// 落 SimpleObstacleLayer 并触发 updateMap (含 Ruling #7 旧带清理)。
  void handleObstaclePointCloud(const ayanami::sensor_msgs::PointCloud2& cloud);
  /// @brief 把激光 ranges 转成点后走同一落图管线。
  void handleObstacleLaserScan(const ayanami::sensor_msgs::LaserScan& scan);

  // ---- accessors (测试 / 宿主) ----
  CostmapContext::Ptr costmapContext() const { return costmap_context_; }
  costmap_2d::CostmapWrapper::Ptr globalCostmap() const {
    return costmap_context_ ? costmap_context_->global_costmap_ : nullptr;
  }
  costmap_2d::CostmapWrapper::Ptr localCostmap() const {
    return costmap_context_ ? costmap_context_->local_costmap_ : nullptr;
  }

 private:
  /// @brief 把 frame_id 下的点经 TransformStore 落图 (handleObstacle* 共用)。
  void markObstaclesInFrame(const std::string& frame_id,
                            const std::vector<Eigen::Vector3d>& points);

  CostmapContext::Ptr costmap_context_;
  // local wrapper 持有的 SimpleObstacleLayer (configureCoreMaps 注入)。
  costmap_2d_layers::SimpleObstacleLayer* obstacle_layer_{nullptr};
  double local_inflation_radius_{0.6};
  double local_resolution_{0.05};
  // Ruling #7: 上一帧 local frame 障碍点 (用于动态更新前 clearArea)。
  std::vector<Eigen::Vector2d> prev_obstacles_local_;
  bool has_prev_obstacles_{false};
  // 插件名 → 绑定 costmap wrapper (planner 绑 global / controller 绑 local)。
  std::map<std::string, costmap_2d::CostmapWrapper::Ptr>
      planner_name_to_costmap_;
  std::map<std::string, costmap_2d::CostmapWrapper::Ptr>
      controller_name_to_costmap_;
  int pointcloud_cb_id_{-1};
  int laserscan_cb_id_{-1};
};

}  // namespace costmap
}  // namespace ayanami
