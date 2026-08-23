#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_smoother.hpp"

namespace ayanami {
namespace costmap {

// 前向声明: initialize 只接受 Ptr (shared_ptr), 无需完整类型。
class CostmapContext;

/**
 * @brief 路径平滑插件基类 (costmap 族, spec §5.5)。
 *
 * 在 abstract::AbstractSmoother 之上加地图族初始化 (CostmapContext 注入)。
 */
class CostmapSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  using Ptr = std::shared_ptr<CostmapSmoother>;

  virtual void initialize(const std::string& name, CostmapContext::Ptr) = 0;
};

}  // namespace costmap
}  // namespace ayanami
