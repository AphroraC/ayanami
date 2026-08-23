#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_planner.hpp"

namespace ayanami {
namespace costmap {

// 前向声明: initialize 只接受 Ptr (shared_ptr), 无需完整类型。
class CostmapContext;

/**
 * @brief 全局路径规划插件基类 (costmap 族, spec §5.5)。
 *
 * 在 abstract::AbstractPlanner (零地图类型) 之上加地图族初始化:
 * initialize 注入 CostmapContext (global/local LayeredCostmap)。
 */
class CostmapPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  using Ptr = std::shared_ptr<CostmapPlanner>;

  virtual void initialize(const std::string& name, CostmapContext::Ptr) = 0;
};

}  // namespace costmap
}  // namespace ayanami
