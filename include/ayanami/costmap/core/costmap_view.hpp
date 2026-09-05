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
        costmap_grid(grid),
        view_frame(std::move(frame)),
        cached_footprint(footprint),
        view_robot_pose(std::move(robot_pose)),
        robot_pose_valid(has_robot_pose),
        is_current(current) {}

  /// @brief master grid (只读; 仅在本视图存活期内有效, 可能为 null)。
  const costmap_2d::Costmap2D* grid() const { return costmap_grid; }

  /// @brief 视图帧名。
  const std::string& frame() const { return view_frame; }

  /// @brief 车体 footprint (configure 期缓存, 不可变; 可能为 null)。
  const costmap_2d::Footprint* footprint() const { return cached_footprint; }

  /// @brief 机器人 (frame) 位姿。
  const ayanami::geometry_msgs::PoseStamped& robot_pose() const {
    return view_robot_pose;
  }
  bool has_robot_pose() const { return robot_pose_valid; }

  /// @brief 构建时刻的地图新鲜度。
  bool current() const { return is_current; }

  bool valid() const { return costmap_grid != nullptr; }

 private:
  std::shared_lock<std::shared_mutex> lock;
  const costmap_2d::Costmap2D* costmap_grid{nullptr};
  std::string view_frame;
  const costmap_2d::Footprint* cached_footprint{nullptr};
  ayanami::geometry_msgs::PoseStamped view_robot_pose;
  bool robot_pose_valid{false};
  bool is_current{true};
};

}  // namespace costmap
}  // namespace ayanami
