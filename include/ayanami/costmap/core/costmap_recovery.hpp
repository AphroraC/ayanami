#pragma once

#include <memory>
#include <string>

#include "ayanami/abstract/core/abstract_recovery.hpp"
#include "ayanami/costmap/core/costmap_command.hpp"
#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/types.hpp"

namespace ayanami {
namespace costmap {

using ayanami::ErrorCode;
using ayanami::RecoveryResultT;

/**
 * @brief 恢复行为插件基类 (costmap族, MBF 式分层)。
 *
 * 在 abstract::AbstractRecovery (start/tick/cancel/finish 统一协议) 之上
 * 只新增:
 * - configure(context, map): 注入插件逻辑名 + 自身配置 + 地图模块 const
 *   指针 (碰撞类恢复读图用)。
 * 需要修改地图的恢复 (如 clear) 在 configure 内经 map->commandPort()
 * 取 CostmapCommandPort 提交命令, 由唯一写者 (CostmapNavigationServer)
 * 执行; 恢复插件绝不持锁直写地图。
 */
class CostmapRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  using Ptr = std::shared_ptr<CostmapRecovery>;

  virtual void configure(const PluginContext& context,
                         CostmapMapContext::ConstPtr map) = 0;
};

}  // namespace costmap
}  // namespace ayanami
