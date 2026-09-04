#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_planner.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 全局路径规划插件基类 (costmap族, MBF 式分层)。
 *
 * 在 abstract::AbstractPlanner (零地图纯虚接口) 之上只新增地图族注入点:
 * - configure(context, map): 插件逻辑名 + 自身配置 + costmap 地图模块
 *   (宿主持有所有权, 注入 const 智能指针; 插件绝不改图)。
 * - makePlan(...): 即 abstract 纯虚签名; 实现经持有的 map lockGlobal()
 *   (RAII shared 锁 + 零拷贝) 读图, 单次规划 (<10ms) 持锁, 地图写者等锁写入。
 */
class CostmapPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  using Ptr = std::shared_ptr<CostmapPlanner>;

  virtual void configure(const PluginContext& context,
                         CostmapContext::ConstPtr map) = 0;
};

}  // namespace costmap
}  // namespace ayanami
