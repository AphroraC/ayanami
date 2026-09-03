#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_planner.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 全局路径规划插件基类 (costmap族, spec §5.5, MBF 式分层)。
 *
 * 在 abstract::AbstractPlanner (零地图纯虚接口) 之上只新增地图注入点:
 * - initialize(name, map): 宿主持有地图模块所有权 (CostmapContext::Ptr),
 *   注入 const 智能指针 (ConstPtr) 由本插件按需持有; 插件绝不改图。
 * - makePlan(...): 即 abstract 纯虚签名, 无地图参数; 实现经持有的 map
 *   lockGlobal() (RAII shared 锁 + 零拷贝) 读图, 单次规划 (<10ms) 持锁,
 *   地图写者 (≈10Hz) 等锁写入。
 */
class CostmapPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  using Ptr = std::shared_ptr<CostmapPlanner>;

  virtual void initialize(const std::string& name,
                          CostmapContext::ConstPtr map) = 0;

  FailureCode makePlan(const ayanami::geometry_msgs::PoseStamped& start,
                       const ayanami::geometry_msgs::PoseStamped& goal,
                       double tolerance, ayanami::nav_msgs::Path& plan,
                       double& cost,
                       std::string& message) override = 0;
};

}  // namespace costmap
}  // namespace ayanami
