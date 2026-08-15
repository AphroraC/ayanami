#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/core/map_context.hpp"
#include "ayanami/interfaces/transform_store.hpp"
#include "ayanami/messages/geometry_msgs.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief costmap 地图上下文 (spec §5.2 派生示例)。
 *
 * 聚合 global/local 两个 CostmapWrapper (global: StaticLayer+InflationLayer;
 * local: rolling SimpleObstacleLayer+InflationLayer), 并把 AbstractMapContext
 * 的语义覆写为 costmap 真实语义:
 * - isCurrent: 两个 wrapper 都 current (任一非 current 则 false)
 * - lock/unlock: 委托 global wrapper 的 recursive_mutex (可重入)
 * - getRobotPose: 经注入的 TransformStore 查 global_frame_ ← robot_frame_
 */
class CostmapContext : public ayanami::abstract::AbstractMapContext {
 public:
  using Ptr = std::shared_ptr<CostmapContext>;

  costmap_2d::CostmapWrapper::Ptr global_costmap_;
  costmap_2d::CostmapWrapper::Ptr local_costmap_;

  std::string global_frame_{"map"};
  std::string robot_frame_{"base_link"};

  /// @brief 由 T20 CostmapNavigationServer 注入 (TransformStore 单例)。
  ayanami::interfaces::TransformStore* transform_store_ = nullptr;

 public:
  CostmapContext() = default;

  const std::string& getGlobalFrame() const override { return global_frame_; }

  const std::string& getRobotFrame() const override { return robot_frame_; }

  /// @brief 任一 wrapper 非 current 则 false; 无 wrapper 视为 current。
  bool isCurrent() const override {
    return (!global_costmap_ || global_costmap_->isCurrent()) &&
           (!local_costmap_ || local_costmap_->isCurrent());
  }

  void lock() override {
    if (global_costmap_) {
      global_costmap_->mutex().lock();
    }
  }

  void unlock() override {
    if (global_costmap_) {
      global_costmap_->mutex().unlock();
    }
  }

  bool getRobotPose(
      ayanami::geometry_msgs::PoseStamped& pose) const override {
    if (!transform_store_) {
      return false;
    }
    Eigen::Isometry3d T;
    if (!transform_store_->lookupTransform(
            global_frame_, robot_frame_,
            ayanami::interfaces::TransformStore::fromSeconds(0.0), T)) {
      return false;
    }
    pose = ayanami::geometry_msgs::PoseStamped::fromIsometry(global_frame_, T);
    return true;
  }

  /// @brief global wrapper 的 master grid (未设置则为 nullptr)。
  costmap_2d::Costmap2D* globalCostmap2D() const {
    return global_costmap_ ? global_costmap_->get().getCostmap() : nullptr;
  }

  /// @brief local wrapper 的 master grid (未设置则为 nullptr)。
  costmap_2d::Costmap2D* localCostmap2D() const {
    return local_costmap_ ? local_costmap_->get().getCostmap() : nullptr;
  }
};

}  // namespace costmap
}  // namespace ayanami
