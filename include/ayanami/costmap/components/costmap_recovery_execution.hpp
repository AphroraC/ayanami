#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/abstract/components/recovery_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 恢复行为的 costmap 版执行 (spec §5.4, 对标 MBF RecoveryCostmapExecution)。
 *
 * 恢复行为同时操作 global + local 两个 costmap:
 * - preRun: 两者 checkActivate(); postRun: 两者 checkDeactivate();
 * - safetyCheck: 两个 wrapper 都 current 才返回 true;
 * - run: 覆写为 scoped_lock(global, local) 临界段包住父 run (clear 直写
 *   local master grid 与 obstacle feed 的 local 锁写串行化)。
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
    // ★ 关键: 整个恢复 (含 clear 直写 local master grid) 在持 global+local
    // 两把锁的临界段内执行, 防与 obstacle feed (markObstaclesInFrame 走 local
    // wrapper mutex 写回同一 cell) 并发竞态。锁序 global→local 无 AB/BA:
    // obstacle feed 严格"local 段写完释放才请求 global" (两段式锁作用域,
    // 见 markObstaclesInFrame), 从不持 local 等 global; scoped_lock 内部
    // std::lock 亦自带死锁回避, 双保险。
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
