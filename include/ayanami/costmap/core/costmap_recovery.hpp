#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_recovery.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
#include "ayanami/costmap/components/costmap_context.hpp"

namespace ayanami {
namespace costmap {

// 周期驱动接口回车类型单一来源 = abstract::RecoveryCycleResult。
using ayanami::abstract::RecoveryCycleResult;

/**
 * @brief 恢复行为插件基类 (costmap族, spec §5.5, MBF 式分层)。
 *
 * 在 abstract::AbstractRecovery (零地图纯虚接口) 之上只新增:
 * - initialize(name, map): 注入地图模块 const 智能指针 (碰撞类恢复读图用)。
 * - setCommandPort(port): 需要修改地图的恢复 (如 clear) 经此提交命令, 由
 *   唯一写者 (CostmapNavigationServer) 执行; 恢复插件绝不持锁直写地图。
 * - runRecovery / onCycleUpdate: 即 abstract 签名, 实现内部按需经持有的
 *   map 加锁读图。
 */
class CostmapRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  using Ptr = std::shared_ptr<CostmapRecovery>;

  virtual void initialize(const std::string& name,
                          CostmapContext::ConstPtr map) = 0;

  /// @brief 注入地图变更命令端口 (恢复插件可选)。默认无操作。
  virtual void setCommandPort(CostmapCommandPort::Ptr port) { (void)port; }
};

}  // namespace costmap
}  // namespace ayanami
