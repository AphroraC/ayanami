#pragma once

#include <memory>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <utility>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/robot_state_controller.hpp"
#include "ayanami/map/abstract_map_context.hpp"
#include "ayanami/costmap/core/costmap_view.hpp"
#include "ayanami/messages/geometry_msgs.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief costmap 族"地图模块" (spec §5.2)。
 *
 * 聚合 global/local 两个 CostmapWrapper (global: StaticLayer+InflationLayer;
 * local: rolling SimpleObstacleLayer+InflationLayer)。
 *
 * 所有权与持有纪律:
 * - 所有权: 由宿主 (CostmapNavigation / CostmapNavigationServer) 持有
 *   CostmapContext::Ptr; 各族插件 (planner/smoother/controller/recovery) 经
 *   initialize(name, ConstPtr) 按需持有 ConstPtr (const 智能指针), 绝不改图。
 * - 读图: 插件经 lockGlobal()/lockLocal() 取 CostmapView (RAII shared 锁 +
 *   零拷贝 master grid 直读), 单次算法调用 (<10ms) 持锁; 地图写者等锁写入。
 * - 可变的 wrapper/layer 访问 (globalCostmap()/localCostmap()) 只提供给
 *   地图写者 (CostmapNavigationServer) 与 execution lifecycle
 *   (checkActivate/checkDeactivate/isCurrent); 规则插件绝不使用。
 * - 字段私有, 只暴露窄 accessor。
 */
class CostmapContext : public map::AbstractMapContext,
                       public std::enable_shared_from_this<CostmapContext> {
 public:
  using Ptr = std::shared_ptr<CostmapContext>;
  using ConstPtr = std::shared_ptr<const CostmapContext>;

  CostmapContext() = default;

  std::string mapFamily() const override { return "costmap"; }
  std::string globalFrame() const override { return global_frame; }
  std::string localFrame() const override { return local_frame; }
  std::string robotFrame() const override { return robot_frame; }
  bool isCurrent() const override { return mapsAreCurrent(); }
  std::uint64_t version() const override { return map_version; }
  void bumpVersion() { ++map_version; }
  std::uint32_t capabilities() const override {
    using Capability = map::MapCapability;
    return static_cast<std::uint32_t>(Capability::CollisionQuery) |
           static_cast<std::uint32_t>(Capability::GlobalPlanning) |
           static_cast<std::uint32_t>(Capability::LocalPlanning) |
           static_cast<std::uint32_t>(Capability::DynamicObstacleUpdate) |
           static_cast<std::uint32_t>(Capability::ClearRegion);
  }
  map::MapCommandPort::Ptr commandPort() const override { return nullptr; }
  bool getRobotState(geometry_msgs::PoseStamped& pose,
                     geometry_msgs::TwistStamped& velocity) const override {
    return components::RobotStateController::instance().getRobotState(
        pose, velocity);
  }

  // ---- 配置 (写者 / 测试注入) ----
  void setGlobalCostmap(costmap_2d::CostmapWrapper::Ptr wrapper) {
    global_costmap = std::move(wrapper);
  }
  void setLocalCostmap(costmap_2d::CostmapWrapper::Ptr wrapper) {
    local_costmap = std::move(wrapper);
  }
  void setFrames(std::string global_frame, std::string local_frame,
                 std::string robot_frame) {
    this->global_frame = std::move(global_frame);
    this->local_frame = std::move(local_frame);
    this->robot_frame = std::move(robot_frame);
  }

  /// @brief setup 期缓存不可变 footprint (configureCoreMaps 注入)。读者只从
  /// 此缓存取 footprint, 绝不读 live LayeredCostmap (写锁外读可变状态)。
  void setFootprint(costmap_2d::Footprint footprint) {
    footprint_ = std::move(footprint);
  }

  // ---- frames / freshness ----
  const std::string& getGlobalFrame() const { return global_frame; }
  const std::string& getRobotFrame() const { return robot_frame; }

  /// @brief 任一 wrapper 非 current 则 false; 无 wrapper 视为 current。
  bool mapsAreCurrent() const {
    return (!global_costmap || global_costmap->isCurrent()) &&
           (!local_costmap || local_costmap->isCurrent());
  }

  /// @brief 获取机器人在指定 frame 的位姿 (默认 global_frame; local 视图
  /// 传 local_frame, 保证 view.frame() 与 robot_pose 帧一致)。
  /// TransformStore, 无锁查询。
  bool getRobotPose(
      ayanami::geometry_msgs::PoseStamped& pose,
      const std::string& target_frame = "") const {
    const std::string& frame =
        target_frame.empty() ? global_frame : target_frame;
    Eigen::Isometry3d T;
    if (!ayanami::components::RobotStateController::instance().lookupTransform(
            frame, robot_frame, ayanami::Timestamp::latest(), T)) {
      return false;
    }
    pose = ayanami::geometry_msgs::PoseStamped::fromIsometry(frame, T);
    return true;
  }

  /// @brief configure 期缓存的不可变 footprint。
  const costmap_2d::Footprint& footprint() const { return footprint_; }

  // ---- 写者/execution lifecycle accessor (可变 map 仅写者触碰) ----
  costmap_2d::CostmapWrapper::Ptr globalCostmap() {
    return global_costmap;
  }
  std::shared_ptr<const costmap_2d::CostmapWrapper> globalCostmap() const {
    return global_costmap;
  }
  costmap_2d::CostmapWrapper::Ptr localCostmap() { return local_costmap; }
  std::shared_ptr<const costmap_2d::CostmapWrapper> localCostmap() const {
    return local_costmap;
  }

  /// @brief Return a const owner for external observers and map-family plugins.
  /// Map data must still be accessed through lockGlobal/lockLocal/readGlobal/
  /// readLocal so the underlying master grid cannot escape its lock scope.
  ConstPtr getSharedConstPtr() const {
    return shared_from_this();
  }

  // ---- 读者接口 (shared 锁 + 零拷贝) ----

  /// @brief 持 shared 锁的 global 读视图 (整次算法调用持有; 无 wrapper →
  /// invalid view)。
  CostmapView lockGlobal() const { return buildView(global_costmap, global_frame); }

  /// @brief 持 shared 锁的 local 读视图 (使用 local frame)。
  CostmapView lockLocal() const { return buildView(local_costmap, local_frame); }

  /// @brief Hold the selected map's read lock without copying the map. The
  /// returned guard must not outlive the plugin call that requested it.
  CostmapView lockGlobalForRead() const { return lockGlobal(); }

  CostmapView lockLocalForRead() const { return lockLocal(); }

  /// @brief Run a callback while the global master grid is protected
  /// (shared lock)。
  template <typename Fn>
  auto readGlobal(Fn&& fn) const
      -> decltype(std::forward<Fn>(fn)(
          std::declval<const costmap_2d::Costmap2D&>())) {
    using Result = decltype(std::forward<Fn>(fn)(
        std::declval<const costmap_2d::Costmap2D&>()));
    if (!global_costmap) {
      if constexpr (std::is_void_v<Result>) return;
      else return Result{};
    }
    std::shared_lock<std::shared_mutex> lock(global_costmap->mutex());
    const auto* grid = global_costmap->get().getCostmap();
    if (!grid) {
      if constexpr (std::is_void_v<Result>) return;
      else return Result{};
    }
    return std::forward<Fn>(fn)(static_cast<const costmap_2d::Costmap2D&>(*grid));
  }

  /// @brief Run a callback while the local master grid is protected
  /// (shared lock)。
  template <typename Fn>
  auto readLocal(Fn&& fn) const
      -> decltype(std::forward<Fn>(fn)(
          std::declval<const costmap_2d::Costmap2D&>())) {
    using Result = decltype(std::forward<Fn>(fn)(
        std::declval<const costmap_2d::Costmap2D&>()));
    if (!local_costmap) {
      if constexpr (std::is_void_v<Result>) return;
      else return Result{};
    }
    std::shared_lock<std::shared_mutex> lock(local_costmap->mutex());
    const auto* grid = local_costmap->get().getCostmap();
    if (!grid) {
      if constexpr (std::is_void_v<Result>) return;
      else return Result{};
    }
    return std::forward<Fn>(fn)(static_cast<const costmap_2d::Costmap2D&>(*grid));
  }

 private:
  /// @brief 持 shared 锁构建读视图: grid/frame 取自 wrapper; footprint 取
  /// configure 期不可变缓存; robot_pose 在视图自身 frame 解析 (local 视图
  /// = local_frame, global 视图 = global_frame), 经 RobotStateController
  /// 无锁解析。
  CostmapView buildView(
      const costmap_2d::CostmapWrapper::Ptr& wrapper,
      const std::string& frame) const {
    if (!wrapper) {
      return CostmapView{};
    }
    std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
    const auto* grid = wrapper->get().getCostmap();
    ayanami::geometry_msgs::PoseStamped robot_pose;
    const bool has_robot_pose = getRobotPose(robot_pose, frame);
    return CostmapView(std::move(lock), grid, frame, &footprint_,
                       std::move(robot_pose), has_robot_pose,
                       wrapper->isCurrent());
  }

  costmap_2d::CostmapWrapper::Ptr global_costmap;
  costmap_2d::CostmapWrapper::Ptr local_costmap;
  std::string global_frame{"map"};
  std::string local_frame{"map"};
  std::string robot_frame{"base_link"};
  std::uint64_t map_version{0};
  costmap_2d::Footprint footprint_;
};

}  // namespace costmap
}  // namespace ayanami
