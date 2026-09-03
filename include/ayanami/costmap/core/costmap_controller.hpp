#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/abstract/core/abstract_goal_checker.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 局部控制插件基类 (costmap族, spec §5.5, MBF 式分层)。
 *
 * 在 abstract::AbstractController (零地图纯虚接口) 之上只新增地图注入点:
 * - initialize(name, map): 注入地图模块 const 智能指针。
 * - computeVelocityCommands(...): 即 abstract 纯虚签名; 实现内部每周期经
 *   map->lockLocal() 加锁读 local master grid (零拷贝, 周期结束释放)。
 */
class CostmapController : public ayanami::abstract::AbstractController {
 public:
  using Ptr = std::shared_ptr<CostmapController>;

  virtual void initialize(const std::string& name,
                          CostmapContext::ConstPtr map) = 0;

  FailureCode computeVelocityCommands(
      const ayanami::geometry_msgs::PoseStamped& pose,
      const ayanami::geometry_msgs::TwistStamped& velocity,
      ayanami::geometry_msgs::TwistStamped& cmd_vel,
      std::string& message) override = 0;
};

}  // namespace costmap
}  // namespace ayanami
