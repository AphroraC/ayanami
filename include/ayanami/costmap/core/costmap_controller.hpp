#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 局部控制插件基类 (costmap族, MBF 式分层)。
 *
 * 在 abstract::AbstractController (零地图纯虚接口) 之上只新增地图族注入点:
 * - configure(context, map): 注入插件逻辑名 + 自身配置 + 地图模块 const 指针。
 * - computeVelocityCommand(...): 即 abstract 纯虚签名; 实现内部每周期经
 *   map->lockLocal() 加锁读 local master grid (零拷贝, 周期结束释放)。
 */
class CostmapController : public ayanami::abstract::AbstractController {
 public:
  using Ptr = std::shared_ptr<CostmapController>;

  virtual void configure(const PluginContext& context,
                         CostmapMapContext::ConstPtr map) = 0;
};

}  // namespace costmap
}  // namespace ayanami
