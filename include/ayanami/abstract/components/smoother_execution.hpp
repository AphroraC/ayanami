#pragma once

#include "ayanami/abstract/components/execution_base.hpp"
#include "ayanami/abstract/core/abstract_smoother.hpp"
#include "ayanami/types.hpp"

#include <memory>
#include <utility>

namespace ayanami {
namespace components {

/**
 * @brief 路径平滑执行 (单次 in-place 执行)。
 *
 * 职责表: 插件只做平滑; 失败立即映射为 ErrorCode 终态, 重试由 BT 决定。
 * in-place 语义: 路径以值传入, 插件就地修改; 成功后 getSmoothedPath()
 * 返回已平滑路径。
 *
 * 状态机: INITIALIZED→STARTED→SMOOTHING→SMOOTHED_PATH (或
 * NO_SMOOTHED_PATH/CANCELED/STOPPED/INTERNAL_ERROR)。
 */
class SmootherExecution : public ExecutionBase {
public:
  using Ptr = std::shared_ptr<SmootherExecution>;

  enum SmootherState {
    INITIALIZED,
    STARTED,
    SMOOTHING,
    SMOOTHED_PATH,
    NO_SMOOTHED_PATH,
    CANCELED,
    STOPPED,
    INTERNAL_ERROR,
  };

  explicit SmootherExecution(ayanami::abstract::AbstractSmoother::Ptr smoother)
      : smoother(std::move(smoother)) {}

  /// @brief 记录入参 (路径按值接管) 并启动平滑线程。
  using ExecutionBase::start;
  void smoothPath(nav_msgs::Path path, double max_duration,
                  bool check_for_collision) {
    this->path = std::move(path);
    this->max_duration = max_duration;
    this->check_for_collision = check_for_collision;
    ExecutionBase::start();
  }

  /// @brief 取消: 置 cancel_requested 并透传到 smoother 插件 (协作式)。
  void cancel() override {
    cancel_requested = true;
    if (smoother) smoother->cancel();
    condition.notify_all();
  }

  SmootherState getState() const {
    return static_cast<SmootherState>(state.load());
  }

  /// @brief 平滑结果 (in-place: 成功后即输入路径的平滑版本)。
  const nav_msgs::Path& getSmoothedPath() const { return path; }

protected:
  /// @brief 插件调用钩子: 默认调 abstract 签名; 地图族 execution 覆写为
  /// typed 调用 (执行层完成地图访问)。
  virtual SmootherResultT callSmoother(nav_msgs::Path& path,
                                       double max_duration,
                                       bool check_for_collision) {
    return smoother->smoothPath(path, max_duration, check_for_collision);
  }

  void run() override {
    setState(STARTED, false);
    try {
      if (cancel_requested) {
        error = ErrorCode::CANCELED;
        setState(CANCELED, true);
        return;
      }
      if (should_exit) {
        error = ErrorCode::CANCELED;
        setState(STOPPED, true);
        return;
      }
      setState(SMOOTHING, false);

      const SmootherResultT result =
          callSmoother(path, max_duration, check_for_collision);

      if (cancel_requested) {
        error = ErrorCode::CANCELED;
        setState(CANCELED, true);
        return;
      }
      if (result.ok) {
        error = ErrorCode::NONE;
        message = result.message;
        setState(SMOOTHED_PATH, true);
        return;
      }
      // 失败立即返回错误码 (重试由 BT 决定)。
      error = result.error;
      message = result.message;
      setState(NO_SMOOTHED_PATH, true);
      return;
    } catch (...) {
      error = ErrorCode::INTERNAL_ERROR;
      message = "Unknown error occurred";
      setState(INTERNAL_ERROR, true);
    }
  }

protected:
  ayanami::abstract::AbstractSmoother::Ptr smoother;
  nav_msgs::Path path;
  double max_duration{0.0};
  bool check_for_collision{false};
};

} // namespace components
} // namespace ayanami
