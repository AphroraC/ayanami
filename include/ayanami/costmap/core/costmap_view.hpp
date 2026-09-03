#pragma once

#include <memory>
#include <shared_mutex>
#include <string>
#include <utility>

#include <costmap_2d/costmap_2d.hpp>

#include "ayanami/messages/geometry_msgs.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 加锁读视图: 算法 (planner/smoother/controller/recovery) 单次调用的
 * 地图输入。
 *
 * 取代旧的 CostmapSnapshot (整图深拷贝/COW epoch) —— 视图 RAII 持有地图
 * shared 锁, grid 指针直指 master grid, 零拷贝。单次规划 <10ms, 地图写者
 * (≈10Hz) 等待读者释放后写入。
 *
 * - grid(): master grid 指针, 仅在本视图存活期内有效 (锁随视图析构释放)。
 * - footprint: 该 map 的车体 footprint (configure 期缓存, 不可变, 无需锁)。
 * - robot_pose: 视图构建时解析的机器人位姿 (TransformStore, 无锁查询)。
 * - frame / current: 元数据。
 */
class CostmapView {
 public:
  CostmapView() = default;
  CostmapView(CostmapView&&) = default;
  CostmapView& operator=(CostmapView&&) = default;
  CostmapView(const CostmapView&) = delete;
  CostmapView& operator=(const CostmapView&) = delete;

  CostmapView(std::shared_lock<std::shared_mutex> lock,
              const costmap_2d::Costmap2D* grid, std::string frame,
              const costmap_2d::Footprint* footprint,
              ayanami::geometry_msgs::PoseStamped robot_pose,
              bool has_robot_pose, bool current)
      : lock(std::move(lock)),
        grid_(grid),
        frame_(std::move(frame)),
        footprint_(footprint),
        robot_pose_(std::move(robot_pose)),
        has_robot_pose_(has_robot_pose),
        current_(current) {}

  /// @brief master grid (只读; 仅在本视图存活期内有效, 可能为 null)。
  const costmap_2d::Costmap2D* grid() const { return grid_; }

  /// @brief 视图帧名。
  const std::string& frame() const { return frame_; }

  /// @brief 车体 footprint (configure 期缓存, 不可变; 可能为 null)。
  const costmap_2d::Footprint* footprint() const { return footprint_; }

  /// @brief 机器人 (frame) 位姿。
  const ayanami::geometry_msgs::PoseStamped& robot_pose() const {
    return robot_pose_;
  }
  bool has_robot_pose() const { return has_robot_pose_; }

  /// @brief 构建时刻的地图新鲜度。
  bool current() const { return current_; }

  bool valid() const { return grid_ != nullptr; }

 private:
  std::shared_lock<std::shared_mutex> lock;
  const costmap_2d::Costmap2D* grid_{nullptr};
  std::string frame_;
  const costmap_2d::Footprint* footprint_{nullptr};
  ayanami::geometry_msgs::PoseStamped robot_pose_;
  bool has_robot_pose_{false};
  bool current_{true};
};

}  // namespace costmap
}  // namespace ayanami
