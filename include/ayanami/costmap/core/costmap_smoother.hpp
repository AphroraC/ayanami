#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_smoother.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 路径平滑插件基类 (costmap族, MBF 式分层)。
 *
 * 在 abstract::AbstractSmoother (零地图纯虚接口) 之上只新增地图族注入点:
 * - configure(context, map): 注入插件逻辑名 + 自身配置 + 地图模块 const 指针。
 * - smoothPath(...): 即 abstract 纯虚 in-place 签名; 需要碰撞检查时实现
 *   内部经 map->lockGlobal() 加锁读图 (零拷贝, 不拷贝不快照)。
 */
class CostmapSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  using Ptr = std::shared_ptr<CostmapSmoother>;

  virtual void configure(const PluginContext& context,
                         CostmapMapContext::ConstPtr map) = 0;
};

}  // namespace costmap
}  // namespace ayanami
