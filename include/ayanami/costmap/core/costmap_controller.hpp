#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_controller.hpp"

namespace ayanami {
namespace costmap {

// 前向声明: initialize 只接受 Ptr (shared_ptr), 无需完整类型。
class CostmapContext;

/**
 * @brief 局部控制插件基类 (costmap 族, spec §5.5)。
 *
 * 在 abstract::AbstractController 之上加地图族初始化 (CostmapContext 注入)。
 */
class CostmapController : public ayanami::abstract::AbstractController {
 public:
  using Ptr = std::shared_ptr<CostmapController>;

  virtual void initialize(const std::string& name, CostmapContext::Ptr) = 0;
};

}  // namespace costmap
}  // namespace ayanami
