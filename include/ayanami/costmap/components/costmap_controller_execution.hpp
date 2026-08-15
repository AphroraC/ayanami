#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <costmap_server/costmap_wrapper.hpp>

#include "ayanami/components/controller_execution.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief 局部控制的 costmap 版执行 (spec §5.4, 对标 MBF ControllerCostmapExecution)。
 *
 * 在 ControllerExecution 之上注入 CostmapWrapper 生命周期钩子:
 * - preRun: checkActivate() 幂等激活 costmap;
 * - postRun: checkDeactivate() 幂等停用 costmap;
 * - safetyCheck: costmap_->isCurrent(); ControllerExecution::run 每周期调
 *   safetyCheck, 非 current 时发布清零停止命令 (不发速度);
 * - run: 覆写加 recursive_mutex scoped_lock 再调父 (整循环持锁)。
 */
class CostmapControllerExecution : public ayanami::components::ControllerExecution {
 public:
  using Ptr = std::shared_ptr<CostmapControllerExecution>;

  CostmapControllerExecution(
      const std::string& name,
      ayanami::abstract::AbstractController::Ptr controller,
      costmap_2d::CostmapWrapper::Ptr costmap_wrapper)
      : ControllerExecution(std::move(controller)),
        name_(name),
        costmap_wrapper_(std::move(costmap_wrapper)) {}

  ~CostmapControllerExecution() override = default;

  void preRun() override {
    if (costmap_wrapper_) {
      costmap_wrapper_->checkActivate();
    }
  }

  void postRun() override {
    if (costmap_wrapper_) {
      costmap_wrapper_->checkDeactivate();
    }
  }

  bool safetyCheck() override {
    return !costmap_wrapper_ || costmap_wrapper_->isCurrent();
  }

 protected:
  void run() override {
    if (costmap_wrapper_) {
      std::lock_guard<std::recursive_mutex> lock(costmap_wrapper_->mutex());
      ControllerExecution::run();
    } else {
      ControllerExecution::run();
    }
  }

 private:
  std::string name_;
  costmap_2d::CostmapWrapper::Ptr costmap_wrapper_;
};

}  // namespace costmap
}  // namespace ayanami
