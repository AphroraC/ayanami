// Task 24: modules/costmap_wait_recovery — 等待恢复模块。
//
// 纯 C++ (零 ROS): 继承 costmap::CostmapRecovery, 循环 sleep 直到
// wait_duration 或 cancel; 不发速度。M1 headless 用, 无真底盘依赖。

#include "costmap_wait_recovery.hpp"

#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <chrono>
#include <thread>

namespace ayanami {
namespace modules {

CostmapWaitRecovery::CostmapWaitRecovery() { loadConfig(); }

void CostmapWaitRecovery::loadConfig() {
  try {
    const ayanami::ComponentConfig holder;
    const auto& file = holder.get();
    config_.wait_duration =
        file.param<double>("recovery", "wait_duration", config_.wait_duration);
    config_.sim_cycle_dt =
        file.param<double>("recovery", "sim_cycle_dt", config_.sim_cycle_dt);
  } catch (...) {
    // 配置不可读时保持默认值。
  }
}

void CostmapWaitRecovery::initialize(
    const std::string& name, ayanami::costmap::CostmapContext::Ptr ctx) {
  (void)name;
  costmap_context_ = std::move(ctx);
  canceled_.store(false);
}

FailureCode CostmapWaitRecovery::runRecovery(std::string& message) {
  message.clear();
  if (canceled_.load()) {
    message = "wait recovery canceled";
    return FailureCode::Canceled;
  }
  if (!costmap_context_) {
    message = "wait recovery not initialized";
    return FailureCode::InternalError;
  }

  const double dt = std::max(config_.sim_cycle_dt, 1e-6);
  const double target = std::max(config_.wait_duration, 0.0);
  const double total_frames = target / dt;

  for (int i = 0; i < static_cast<int>(std::ceil(total_frames)); ++i) {
    if (canceled_.load()) {
      message = "wait recovery canceled";
      return FailureCode::Canceled;
    }
    std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(dt)));
  }

  return FailureCode::None;
}

bool CostmapWaitRecovery::cancel() {
  canceled_.store(true);
  return true;
}

}  // namespace modules
}  // namespace ayanami
