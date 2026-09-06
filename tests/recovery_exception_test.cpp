// recovery_exception_test: 异常路径闭合回归 (高优缺陷 #1)。
//
// 锁死三类异常场景的行为契约:
// 1. recovery 插件 start 抛异常 → 任务有界时间内以 ABORTED/INTERNAL_ERROR
// 结算 (不再悬挂驱动任务), recovery_exclusive 释放 (后续 planner 请求
// 不再 RESOURCE_BUSY);
// 2. ExecutionBase::run 逃逸异常 → runTask 兜底置 k_aborted 终态;
// 3. finish 抛异常不破坏已完成的终态结算。

#include <gtest/gtest.h>

#include <ayanami/abstract/components/action_base.hpp>
#include <ayanami/abstract/components/execution_base.hpp>
#include <ayanami/abstract/components/plugin_manager.hpp>
#include <ayanami/abstract/components/recovery_action.hpp>
#include <ayanami/abstract/components/recovery_execution.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/types.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#ifndef AYANAMI_TEST_THROWING_RECOVERY_SO
#error "AYANAMI_TEST_THROWING_RECOVERY_SO required"
#endif
#ifndef AYANAMI_TEST_PLANNER_SO
#error "AYANAMI_TEST_PLANNER_SO required"
#endif

namespace {

using ayanami::RecoveryResultT;
using ayanami::abstract::AbstractRecovery;
using ayanami::actions::GoalStatus;
using ayanami::actions::RecoveryGoal;
using ayanami::actions::RecoveryResult;

template <typename Task>
bool readyWithin(Task& task, int timeout_ms) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (task.ready()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return task.ready();
}

// start 抛异常的 recovery (测试二进制内直连 RecoveryAction, 不经 dlopen)。
class ThrowingStartRecovery : public AbstractRecovery {
 public:
  void start() override { throw std::runtime_error("recovery start exploded"); }
  RecoveryResultT tick() override {
    return RecoveryResultT::success(ayanami::RecoveryCycleData{});
  }
};

// finish 抛异常的 recovery: start/tick 正常, 收尾炸 —— 已完成的终态结算
// 不应被破坏。
class ThrowingFinishRecovery : public AbstractRecovery {
 public:
  void finish() override { throw std::runtime_error("recovery finish exploded"); }
  RecoveryResultT tick() override {
    return RecoveryResultT::success(
        ayanami::RecoveryCycleData{ayanami::RecoveryStatus::SUCCESS, {}});
  }
};

}  // namespace

TEST(RecoveryExceptionTest, throwingStartSettlesTaskAndReleasesSlot) {
  auto recovery = std::make_shared<ThrowingStartRecovery>();
  ayanami::components::RecoveryAction action(recovery);

  RecoveryGoal goal;
  goal.recovery_id = "throwing";
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(task.valid());

  // 修复前: start 逃逸 run → 状态停在 STARTED → 任务永不完成 (悬挂)。
  ASSERT_TRUE(readyWithin(task, 2000)) << "task never settled after start() threw";
  const auto completion_status = task.status();
  EXPECT_EQ(completion_status, GoalStatus::ABORTED);
  EXPECT_EQ(task.result().error, ayanami::ErrorCode::INTERNAL_ERROR);
  EXPECT_FALSE(action.isRunning(0));
}

TEST(RecoveryExceptionTest, throwingFinishKeepsCompletedOutcome) {
  auto recovery = std::make_shared<ThrowingFinishRecovery>();
  ayanami::components::RecoveryAction action(recovery);

  RecoveryGoal goal;
  goal.recovery_id = "throwing_finish";
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(task.valid());

  ASSERT_TRUE(readyWithin(task, 2000));
  // tick 已 SUCCESS 结算; finish 的逃逸异常不允许改写结果或悬挂。
  EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
  EXPECT_EQ(task.result().error, ayanami::ErrorCode::NONE);
}

// runTask 兜底: run 逃逸异常 → k_aborted 终态 (基类契约)。
TEST(RecoveryExceptionTest, escapedRunExceptionHitsAbortedSentinel) {
  class ThrowingExecution : public ayanami::components::ExecutionBase {
   public:
    void run() override { throw std::runtime_error("run exploded"); }
  };

  ThrowingExecution exec;
  exec.start();
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
  while (std::chrono::steady_clock::now() < deadline && !exec.aborted()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(exec.aborted());
  EXPECT_EQ(exec.getState(), ayanami::components::ExecutionBase::k_aborted);
  exec.join();
}

// 资源协议回归: 经完整 server 走 requestRecovery, start 抛异常后互斥必须
// 释放 —— 后续 planner 请求不得 RESOURCE_BUSY (修复前为永久死锁)。
namespace {

class RecoveryExceptionServer : public ayanami::components::NavigationServerBase {
 public:
  void loadPlugins(const std::string& recovery_so,
                   const std::string& planner_so) {
    default_planner = "success_planner";
    planner_plugins.loadPlugin({default_planner, planner_so}, {});
    recovery_plugins.loadPlugin({"throwing_recovery", recovery_so}, {});
    createActions();
  }
};

}  // namespace

TEST(RecoveryExceptionTest, failedRecoveryReleasesExclusiveResourceProtocol) {
  RecoveryExceptionServer server;
  server.loadPlugins(AYANAMI_TEST_THROWING_RECOVERY_SO, AYANAMI_TEST_PLANNER_SO);
  server.start();

  RecoveryGoal goal;
  goal.recovery_id = "throwing_recovery";
  auto recovery_task = server.requestRecovery(goal);
  ASSERT_TRUE(recovery_task.valid());
  ASSERT_TRUE(readyWithin(recovery_task, 2000))
      << "recovery task never settled; recovery_exclusive would deadlock";
  EXPECT_EQ(recovery_task.status(), GoalStatus::ABORTED);

  // 互斥已释放: planner 请求被正常接受并成功结算 (而非 RESOURCE_BUSY 拒绝)。
  ayanami::actions::GetPathGoal path_goal;
  auto path_task = server.requestPath(path_goal);
  ASSERT_TRUE(path_task.valid());
  ASSERT_TRUE(readyWithin(path_task, 2000));
  EXPECT_EQ(path_task.status(), GoalStatus::SUCCEEDED);
  EXPECT_NE(path_task.result().error, ayanami::ErrorCode::RESOURCE_BUSY);

  server.stop();
}
