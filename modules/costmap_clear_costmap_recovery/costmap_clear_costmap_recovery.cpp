// Task 24: modules/costmap_clear_costmap_recovery — 清代价地图恢复模块。
//
// 纯 C++ (零 ROS): 统一恢复协议 (start/tick/finish)。首周期构造类型化
// ClearCostmapCommand 并经 CostmapCommandPort 提交给唯一写者
// (CostmapNavigationServer) 执行清除, 随后 SUCCESS (无运动, 零速度)。
// 插件不自行 sleep、不直接触碰可变 wrapper / layer。

#include "costmap_clear_costmap_recovery.hpp"

namespace ayanami {
namespace modules {

using ayanami::costmap::ClearCostmapCommand;
using ayanami::costmap::CostmapCommandPort;

void CostmapClearCostmapRecovery::configure(
    const ayanami::PluginContext& context,
    ayanami::costmap::CostmapContext::ConstPtr map) {
  (void)map;  // 改图仅经命令端口, 不读图。
  // 参数单一来源: 本模块 json (config_recovery 指向文件) 的 recovery 段。
  param.clear_radius =
      context.config.param<double>("recovery", "clear_radius", param.clear_radius);
  param.affected_maps = context.config.param<std::string>(
      "recovery", "affected_maps", param.affected_maps);
  name = context.name;
}

void CostmapClearCostmapRecovery::setCommandPort(CostmapCommandPort::Ptr port) {
  command_port = std::move(port);
}

void CostmapClearCostmapRecovery::start() {
  canceled.store(false);
  submitted = false;
}

RecoveryResultT CostmapClearCostmapRecovery::tick() {
  if (canceled.load()) {
    return RecoveryResultT::failure(ErrorCode::CANCELED,
                                    "clear costmap recovery canceled");
  }
  if (!command_port) {
    return RecoveryResultT::failure(
        ErrorCode::INTERNAL_ERROR, "clear costmap recovery has no command port");
  }

  if (!submitted) {
    // 构造类型化清图命令 (地图变更只能经写者命令端口提交)。
    ClearCostmapCommand command;
    command.affected_maps = param.affected_maps;
    command.clear_radius = param.clear_radius;
    if (!command_port->submit(command)) {
      return RecoveryResultT::failure(
          ErrorCode::INTERNAL_ERROR,
          "clear costmap recovery command rejected by writer");
    }
    submitted = true;
  }

  RecoveryCycleData data;
  data.status = RecoveryStatus::SUCCESS;
  return RecoveryResultT::success(std::move(data));
}

bool CostmapClearCostmapRecovery::cancel() {
  canceled.store(true);
  return true;
}

}  // namespace modules
}  // namespace ayanami
