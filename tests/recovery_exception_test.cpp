// recovery_exception_test: exception-path closure regression (high-priority defect #1).
//
// Locks the behavioral contracts for three exception scenarios:
// 1. a recovery module whose start throws -> the task settles as ABORTED/INTERNAL_ERROR within bounded time
// (the driving task no longer hangs) and recovery_exclusive is released (later planner requests
// no longer get RESOURCE_BUSY);
// 2. an exception escaping ExecutionBase::run -> the runTask backstop sets the k_aborted terminal state;
// 3. an exception thrown from finish must not corrupt an already-completed terminal settlement.

#include <gtest/gtest.h>

#include <ayanami/abstract/components/action_base.hpp>
#include <ayanami/abstract/components/execution_base.hpp>
#include <ayanami/abstract/components/module_manager.hpp>
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

using ayanami::RecoveryCycleData;
using ayanami::ResultT;
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

/// A recovery whose start throws (wired directly to RecoveryAction inside the test binary, no dlopen).
class ThrowingStartRecovery : public AbstractRecovery {
 public:
  void start() override { throw std::runtime_error("recovery start exploded"); }
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    cycle = ayanami::RecoveryCycleData{};
    return ResultT<void>::success();
  }
};

/// A recovery whose finish throws: start/tick are normal, the epilogue explodes -- an already-completed terminal
/// settlement must not be corrupted.
class ThrowingFinishRecovery : public AbstractRecovery {
 public:
  void finish() override { throw std::runtime_error("recovery finish exploded"); }
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    cycle = ayanami::RecoveryCycleData{ayanami::RecoveryStatus::SUCCESS, {}};
    return ResultT<void>::success();
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

  // Before the fix: an exception escaping start left the state stuck at STARTED -> the task never completed (hang).
  ASSERT_TRUE(readyWithin(task, 2000)) << "task never settled after start() threw";
  const auto completion_status = task.status();
  EXPECT_EQ(completion_status, GoalStatus::ABORTED);
  EXPECT_EQ(task.result().error, ayanami::ErrorCode::INTERNAL_ERROR);
  EXPECT_FALSE(action.isRunning());
}

TEST(RecoveryExceptionTest, throwingFinishKeepsCompletedOutcome) {
  auto recovery = std::make_shared<ThrowingFinishRecovery>();
  ayanami::components::RecoveryAction action(recovery);

  RecoveryGoal goal;
  goal.recovery_id = "throwing_finish";
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(task.valid());

  ASSERT_TRUE(readyWithin(task, 2000));
  // The tick already settled SUCCESS; an escaping exception from finish must not rewrite the result or hang.
  EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
  EXPECT_EQ(task.result().error, ayanami::ErrorCode::NONE);
}

// runTask backstop: an exception escaping run -> the k_aborted terminal state (base-class contract).
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

// Resource protocol regression: requestRecovery via the full server; after a start exception the mutex must be
// released -- later planner requests must not get RESOURCE_BUSY (a permanent deadlock before the fix).
namespace {

class RecoveryExceptionServer : public ayanami::components::NavigationServerBase {
 public:
  void loadModules(const std::string& recovery_so,
                   const std::string& planner_so) {
    default_planner = "success_planner";
    // No config file: these test modules read no parameters.
    const ayanami::ModuleContext no_config{"test", ayanami::Config("")};
    planner_modules.loadModule({default_planner, planner_so}, no_config, {});
    recovery_modules.loadModule({"throwing_recovery", recovery_so}, no_config,
                                {});
    createActions();
  }
};

}  // namespace

TEST(RecoveryExceptionTest, failedRecoveryReleasesExclusiveResourceProtocol) {
  RecoveryExceptionServer server;
  server.loadModules(AYANAMI_TEST_THROWING_RECOVERY_SO, AYANAMI_TEST_PLANNER_SO);
  server.start();

  RecoveryGoal goal;
  goal.recovery_id = "throwing_recovery";
  auto recovery_task = server.requestRecovery(goal);
  ASSERT_TRUE(recovery_task.valid());
  ASSERT_TRUE(readyWithin(recovery_task, 2000))
      << "recovery task never settled; recovery_exclusive would deadlock";
  EXPECT_EQ(recovery_task.status(), GoalStatus::ABORTED);

  // The mutex was released: the planner request is accepted normally and settles successfully (not rejected with RESOURCE_BUSY).
  ayanami::actions::GetPathGoal path_goal;
  auto path_task = server.requestPath(path_goal);
  ASSERT_TRUE(path_task.valid());
  ASSERT_TRUE(readyWithin(path_task, 2000));
  EXPECT_EQ(path_task.status(), GoalStatus::SUCCEEDED);
  EXPECT_NE(path_task.result().error, ayanami::ErrorCode::RESOURCE_BUSY);

  server.stop();
}
