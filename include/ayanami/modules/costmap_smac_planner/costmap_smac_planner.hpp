#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief A* 全局路径规划模块 (spec §5.5 CostmapPlanner 族, 参考 _legacy
 * SmacPlannerAStar 纯 C++ 重写)。
 *
 * 在注入的 CostmapContext (global LayeredCostmap) 上做标准 4/8 连通 A*:
 * - 阻断: cell cost >= INSCRIBED_INFLATED_OBSTACLE (Ruling #6, 253 起 lethal)
 * - 行走代价: step * (neutral_cost + cost_factor * cell_cost)
 * - goal 在障碍内: nearestValidGoal 在 tolerance 内搜索最近可行格 (近似路径)
 */
class CostmapSmacPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  CostmapSmacPlanner();
  // Ruling #3: 派生虚构补 override (基类 AbstractPlanner 已虚)
  ~CostmapSmacPlanner() override = default;

  void initialize(const std::string& name,
                  ayanami::costmap::CostmapContext::Ptr ctx) override;

  uint32_t makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal, double tolerance,
                    nav_msgs::Path& plan, double& cost,
                    std::string& message) override;

  bool cancel() override;

 private:
  struct Config {
    bool use_diagonal = true;
    double tolerance = 0.0;
    double cost_factor = 0.5;
    int lethal_cost = 254;
    double neutral_cost = 1.0;
    bool use_final_approach = false;
  };

  void loadConfig();

  ayanami::costmap::CostmapContext::Ptr costmap_context_;
  std::atomic<bool> canceled_{false};
  Config config_;
};

}  // namespace modules
}  // namespace ayanami
