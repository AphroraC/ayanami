// Task 24: modules/costmap_clear_costmap_recovery — 清代价地图恢复模块。
//
// 纯 C++ (零 ROS): 周期驱动接口 (start/onCycleUpdate)。首周期构造类型化
// ClearCostmapCommand 并经 CostmapCommandPort 提交给唯一写者
// (CostmapNavigationServer) 执行清除, 随后 SUCCESS (无运动, 零速度)。
// 插件不自行 sleep、不直接触碰可变 wrapper / layer。

#include "costmap_clear_costmap_recovery.hpp"

#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/config.hpp>

#include <string>

namespace ayanami {
namespace modules {

using ayanami::costmap::ClearCostmapCommand;
using ayanami::costmap::CostmapCommandPort;

CostmapClearCostmapRecovery::CostmapClearCostmapRecovery() {
  // 参数在构造时经 glim 式显式 Config 读取并拷贝进成员 (无 thread_local 注入,
  // 配置不可读时保持默认值): 主 config.json 的 global 单层索引
  // config_clear_costmap_recovery → 本插件 json 文件。
  try {
    const Config file(
        GlobalConfig::getConfigPath("config_clear_costmap_recovery"));
    param.clear_radius =
        file.param<double>("recovery", "clear_radius", param.clear_radius);
    param.affected_maps = file.param<std::string>(
        "recovery", "affected_maps", param.affected_maps);
  } catch (...) {
    // 配置不可读时保持默认值。
  }
}

void CostmapClearCostmapRecovery::initialize(
    const std::string& name, ayanami::costmap::CostmapContext::ConstPtr map) {
  (void)map;  // 改图仅经命令端口, 不读图。
  this->name = name;
}

void CostmapClearCostmapRecovery::setCommandPort(CostmapCommandPort::Ptr port) {
  command_port = std::move(port);
}

void CostmapClearCostmapRecovery::start() {
  canceled.store(false);
  submitted = false;
}

ayanami::abstract::RecoveryCycleResult CostmapClearCostmapRecovery::tick(
    std::string& message) {
  message.clear();
  ayanami::abstract::RecoveryCycleResult r;  // 默认 RUNNING + 零速度。
  if (canceled.load()) {
    r.status = ayanami::abstract::RecoveryCycleResult::Status::CANCELED;
    r.failure = FailureCode::Canceled;
    message = "clear costmap recovery canceled";
    return r;
  }
  if (!command_port) {
    r.status = ayanami::abstract::RecoveryCycleResult::Status::FAILED;
    r.failure = FailureCode::InternalError;
    message = "clear costmap recovery has no command port";
    return r;
  }

  if (!submitted) {
    // 构造类型化清图命令 (地图变更只能经写者命令端口提交)。
    ClearCostmapCommand command;
    command.affected_maps = param.affected_maps;
    command.clear_radius = param.clear_radius;
    if (!command_port->submit(command)) {
      r.status = ayanami::abstract::RecoveryCycleResult::Status::FAILED;
      r.failure = FailureCode::InternalError;
      message = "clear costmap recovery command rejected by writer";
      return r;
    }
    submitted = true;
  }

  r.status = ayanami::abstract::RecoveryCycleResult::Status::SUCCESS;
  return r;
}

void CostmapClearCostmapRecovery::end() {}

FailureCode CostmapClearCostmapRecovery::runRecovery(std::string& message) {
  start();
  const auto r = tick(message);
  switch (r.status) {
    case ayanami::abstract::RecoveryCycleResult::Status::RUNNING:
    case ayanami::abstract::RecoveryCycleResult::Status::SUCCESS:
      return FailureCode::None;
    case ayanami::abstract::RecoveryCycleResult::Status::CANCELED:
      return FailureCode::Canceled;
    case ayanami::abstract::RecoveryCycleResult::Status::FAILED:
    default:
      return r.failure;
  }
}

bool CostmapClearCostmapRecovery::cancel() {
  canceled.store(true);
  return true;
}

}  // namespace modules
}  // namespace ayanami
