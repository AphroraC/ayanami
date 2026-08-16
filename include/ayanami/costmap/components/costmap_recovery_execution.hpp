#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/components/recovery_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 恢复行为的 costmap 版执行 (spec §5.4, 对标 MBF RecoveryCostmapExecution)。
 *
 * 恢复行为同时操作 global + local 两个 costmap:
 * - preRun: 两者 checkActivate(); postRun: 两者 checkDeactivate();
 * - safetyCheck: 两个 wrapper 都 current 才返回 true;
 * - run: 覆写加 global wrapper 的 recursive_mutex scoped_lock 再调父。
 */
class CostmapRecoveryExecution : public ayanami::components::RecoveryExecution {
 public:
  using Ptr = std::shared_ptr<CostmapRecoveryExecution>;

  CostmapRecoveryExecution(const std::string& name,
                           ayanami::abstract::AbstractRecovery::Ptr recovery,
                           costmap_2d::CostmapWrapper::Ptr global,
                           costmap_2d::CostmapWrapper::Ptr local)
      : RecoveryExecution(std::move(recovery)),
        name_(name),
        global_costmap_(std::move(global)),
        local_costmap_(std::move(local)) {}

  ~CostmapRecoveryExecution() override = default;

  void preRun() override {
    if (global_costmap_) {
      global_costmap_->checkActivate();
    }
    if (local_costmap_) {
      local_costmap_->checkActivate();
    }
  }

  void postRun() override {
    if (global_costmap_) {
      global_costmap_->checkDeactivate();
    }
    if (local_costmap_) {
      local_costmap_->checkDeactivate();
    }
  }

  bool safetyCheck() override {
    return (!global_costmap_ || global_costmap_->isCurrent()) &&
           (!local_costmap_ || local_costmap_->isCurrent());
  }

 protected:
  void run() override {
    // ★ 关键: 同时锁 global + local 防与 obstacle feed (handleObstaclePointCloud
    // 走 local wrapper mutex) 死锁/竞态。锁顺序固定 global→local 避免死锁;
    // std::scoped_lock 内部按声明顺序加锁 (C++17)。
    if (global_costmap_ && local_costmap_) {
      std::scoped_lock lk(global_costmap_->mutex(), local_costmap_->mutex());
      RecoveryExecution::run();
      return;
    }
    if (global_costmap_) {
      std::lock_guard<std::recursive_mutex> lk(global_costmap_->mutex());
      RecoveryExecution::run();
      return;
    }
    if (local_costmap_) {
      std::lock_guard<std::recursive_mutex> lk(local_costmap_->mutex());
      RecoveryExecution::run();
      return;
    }
    RecoveryExecution::run();
  }

 private:
  std::string name_;
  costmap_2d::CostmapWrapper::Ptr global_costmap_;
  costmap_2d::CostmapWrapper::Ptr local_costmap_;
};

}  // namespace costmap
}  // namespace ayanami
