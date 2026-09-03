#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_smoother.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 路径平滑插件基类 (costmap族, spec §5.5, MBF 式分层)。
 *
 * 在 abstract::AbstractSmoother (零地图纯虚接口) 之上只新增地图注入点:
 * - initialize(name, map): 注入地图模块 const 智能指针。
 * - smoothPath(...): 即 abstract 纯虚签名; 需要碰撞检查时实现内部经
 *   map->lockGlobal() 加锁读图 (零拷贝, 不拷贝不快照)。
 */
class CostmapSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  using Ptr = std::shared_ptr<CostmapSmoother>;

  virtual void initialize(const std::string& name,
                          CostmapContext::ConstPtr map) = 0;

  FailureCode smoothPath(const ayanami::nav_msgs::Path& path,
                         ayanami::nav_msgs::Path& smoothed_path,
                         double max_duration, bool check_for_collision,
                         std::string& message) override = 0;
};

}  // namespace costmap
}  // namespace ayanami
