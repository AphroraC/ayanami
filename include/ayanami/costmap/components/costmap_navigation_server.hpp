#pragma once

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/navigation_server_base.hpp"
#include "ayanami/costmap/components/costmap_action.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
#include "ayanami/costmap/core/costmap_controller.hpp"
#include "ayanami/costmap/core/costmap_planner.hpp"
#include "ayanami/costmap/core/costmap_recovery.hpp"
#include "ayanami/costmap/core/costmap_smoother.hpp"
#include "ayanami/messages/sensor_msgs.hpp"

namespace costmap_2d_layers {
class SimpleObstacleLayer;
class StaticLayer;
}

namespace ayanami {
namespace costmap {

/**
 * @brief costmap 特化的导航服务 (spec §5.4, 对标 MBF navfn/planner 组合)。
 *
 * 在 NavigationServerBase (无地图, spec §5.3) 之上:
 * - 覆写虚 initialize*Plugin: dynamic_pointer_cast 到 costmap 族接口
 *   (CostmapPlanner/Smoother/Controller/Recovery); 失败 (非 costmap 族插件)
 *   spdlog::error + return false (不入表)。算法插件只接收名称初始化, 不注入
 *   CostmapContext —— 地图访问全部经 execution 加锁直读。
 * - 覆写 createActions(): 生产路径直接构造 Costmap*Action (带 CostmapContext
 *   + execution 配置: checker/速度上限/机器人状态源), 取代基类的
 *   new*Execution 虚工厂间接层。
 * - configureCoreMaps(config_path) 读 config_costmap 构 global/local
 *   LayeredCostmap: global = StaticLayer+InflationLayer;
 *   local = rolling SimpleObstacleLayer+InflationLayer; 每次变更 (障碍落图 /
 *   命令端口 clear) 都由本 server 作为唯一写者 updateMap → 发布不可变 epoch。
 * - 障碍观测不走全局回调槽: 宿主 (CostmapNavigation) 把 setPointcloud/
 *   setLaserScan 直递到公共 handleObstacle* 方法落图 (输入/输出分离纪律)。
 * - RobotStateController::instance() 帧名 + velocity_timeout 在 configureCoreMaps
 *   一次性配置; controller execution 经其读最新 TF 位姿 + 最新速度 (过期零速)。
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
  /// InflationLayer + SimpleObstacleLayer), 并注入 map_context。
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

  // ---- createActions 覆写: 生产路径 = Costmap*Action (costmap_action) ----
  void createActions() override;

  // ---- 障碍观测信号接入口 ----
  /// @brief 把点云 (sensor frame) 经 TransformStore 转到 local costmap frame,
  /// 落 SimpleObstacleLayer 并触发 updateMap (含 Ruling #7 旧带清理)。
  void handleObstaclePointCloud(const ayanami::sensor_msgs::PointCloud2& cloud);
  /// @brief 把激光 ranges 转成点后走同一落图管线。
  void handleObstacleLaserScan(const ayanami::sensor_msgs::LaserScan& scan);

  // ---- accessors (测试 / 宿主) ----
   CostmapContext::Ptr costmapContext() const { return costmap_context; }
   CostmapContext::ConstPtr costmapContextConst() const {
     return costmap_context ? costmap_context->getSharedConstPtr() : nullptr;
   }
  costmap_2d::CostmapWrapper::Ptr globalCostmap() const {
    return costmap_context ? costmap_context->globalCostmap() : nullptr;
  }
  costmap_2d::CostmapWrapper::Ptr localCostmap() const {
    return costmap_context ? costmap_context->localCostmap() : nullptr;
  }

  /// @brief 地图变更命令端口 (清除等写命令; 恢复插件经此提交)。
  CostmapCommandPort::Ptr commandPort() const { return command_port; }

  /// @brief 执行一条清代价地图命令 (唯一写者路径: 变更层 + updateMap →
  /// 发布新 epoch)。供命令端口实现与测试复用。
  bool executeClearCommand(const ClearCostmapCommand& command);

 private:
  /// @brief 把 frame_id 下 (stamp 时刻) 的点经 TransformStore 落图
  /// (handleObstacle* 共用)。stamp 用传感器消息 header.stamp 查询 TF。
  void markObstaclesInFrame(const std::string& frame_id,
                            const ayanami::Timestamp& stamp,
                            const std::vector<Eigen::Vector3d>& points,
                            const std::vector<Eigen::Vector3d>& clearing_points = {});

  CostmapContext::Ptr costmap_context;
  CostmapCommandPort::Ptr command_port;

  // TF 链路缺失告警节流 (每次传感器回调都可能失败; 只按固定间隔提示, 避免
  // 5.5Hz 激光每帧刷屏)。同一链路 (local/global) 缺 TF 持续超过间隔才再告警。
  std::atomic<std::chrono::steady_clock::time_point> local_tf_warn_at{};
  std::atomic<std::chrono::steady_clock::time_point> global_tf_warn_at{};

  // 机器人位姿缓存 (local rolling 窗口滚动用): 位姿查询短暂失败时用最后
  // 一次成功值滚动窗口, 绝不回退到 (0,0,0) —— 否则滚动代价地图窗口会被锚
  // 回世界原点, 机器人走出窗口后 dwb 无路可走 (原地卡顿)。
  bool has_last_local_robot{false};
  double last_local_robot_x{0.0}, last_local_robot_y{0.0};
  double last_local_robot_yaw{0.0};
  // Each map owns an obstacle layer because observations must be transformed
  // into that map's frame independently.
  costmap_2d_layers::SimpleObstacleLayer* global_obstacle_layer{nullptr};
  costmap_2d_layers::SimpleObstacleLayer* local_obstacle_layer{nullptr};
  costmap_2d_layers::StaticLayer* global_static_layer{nullptr};
  double global_inflation_radius{0.6};
  double local_inflation_radius{0.6};
  double global_resolution{0.05};
  double local_resolution{0.05};
  double local_raytrace_range{3.0};
  // Previous observations are stored in each map frame for bounded clearing.
  std::vector<Eigen::Vector2d> prev_obstacles_global;
  std::vector<Eigen::Vector2d> prev_obstacles_local;
  bool has_prev_obstacles_global{false};
  bool has_prev_obstacles{false};
};

}  // namespace costmap
}  // namespace ayanami
