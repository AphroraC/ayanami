#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/core/costmap_planner.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief A* 全局路径规划模块 (CostmapPlanner 族)。
 *
 * 在 costmap 全局图 (shared 锁零拷贝读视图) 上做标准 4/8 连通 A*:
 * - 阻断: cell cost >= INSCRIBED_INFLATED_OBSTACLE (Ruling #6, 253 起 lethal)
 * - 行走代价: step * (neutral_cost + cost_factor * cell_cost)
 * - goal 在障碍内: nearestValidGoal 在 tolerance 内搜索最近可行格 (近似路径)
 *
 * 地图访问纪律: configure(context, map) 注入自身配置 + 地图模块 const 智能
 * 指针; makePlan 内部经 map->lockGlobal() 持 shared 锁零拷贝直读 master grid,
 * 单次规划 (<10ms) 持锁, 地图写者等锁写入。
 */
class CostmapSmacPlanner final : public ayanami::costmap::CostmapPlanner {
 public:
  CostmapSmacPlanner() = default;
  // Ruling #3: 派生虚构补 override (基类 AbstractPlanner 已虚)
  ~CostmapSmacPlanner() override = default;

  void configure(const PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  PlannerResultT makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          double tolerance) override;

  bool cancel() override;
  bool setNewGoal(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  double tolerance) override;

 private:
  struct CostmapSmacPlannerParam {
    bool use_diagonal = true;
    double tolerance = 0.0;
    double cost_factor = 0.5;
    int lethal_cost = 254;
    double neutral_cost = 1.0;
    bool use_final_approach = false;
  };

  CostmapSmacPlannerParam param;

  ayanami::costmap::CostmapMapContext::ConstPtr map_;

  std::atomic<bool> canceled{false};
  std::atomic<uint64_t> goal_revision{0};
};

}  // namespace modules
}  // namespace ayanami
