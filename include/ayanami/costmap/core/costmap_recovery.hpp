#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_recovery.hpp"

namespace ayanami {
namespace costmap {

// 前向声明: initialize 只接受 Ptr (shared_ptr), 无需完整类型。
class CostmapContext;

/**
 * @brief 恢复行为插件基类 (costmap 族, spec §5.5)。
 *
 * 在 abstract::AbstractRecovery 之上加地图族初始化 (CostmapContext 注入)。
 */
class CostmapRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  using Ptr = std::shared_ptr<CostmapRecovery>;

  virtual void initialize(const std::string& name, CostmapContext::Ptr) = 0;
};

}  // namespace costmap
}  // namespace ayanami
