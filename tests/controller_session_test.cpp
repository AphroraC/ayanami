// 复刻 legacy controller_session_test.cpp 契约 (新接口等价改写)。
//
// 契约: ControllerAction 一次 acceptGoal (携带 10-pose 路径) → 执行层持续
// 多帧 computeVelocityCommands → goal_checker 触发 ARRIVED_GOAL → result
// SUCCEEDED + reached=true; 中间帧经 on_velocity_command 旁路发布 cmd_vel
// (count>=1), setPlan 收到完整的 10-pose 路径。

#include <gtest/gtest.h>

#include <ayanami/abstract/components/controller_action.hpp>
#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

using ayanami::ErrorCode;
using ayanami::ControllerCommand;
using ayanami::ControllerResultT;
using ayanami::actions::ExePathGoal;
using ayanami::actions::GoalStatus;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

// 记录型控制器: 到达判定交给 goal_checker (N 帧 compute 后触发);
// setPlan 记录收到的路径尺寸 (验证 10-pose 路径被完整应用)。
class RecordingController : public ayanami::abstract::AbstractController {
public:
  ControllerResultT computeVelocityCommand(const PoseStamped&,
                                           const TwistStamped&) override {
    ++calls;
    ControllerCommand cmd;
    cmd.velocity = TwistStamped{};
    cmd.velocity.twist.linear.x() = 0.5;
    return ControllerResultT::success(std::move(cmd));
  }

  bool setPlan(const Path& plan) override {
    std::lock_guard<std::mutex> lock(mutex);
    applied_sizes.push_back(plan.poses.size());
    return true;
  }

  bool cancel() override { return true; }

  std::atomic<int> calls{0};
  std::mutex mutex;
  std::vector<std::size_t> applied_sizes;
};

// ---- checker 测试桩 ----

class LambdaGoalChecker : public ayanami::abstract::AbstractGoalChecker {
 public:
  explicit LambdaGoalChecker(std::function<bool()> reached)
      : reached(std::move(reached)) {}
  void reset() override {}
  bool isGoalReached(const PoseStamped&, const PoseStamped&,
                     const TwistStamped&) override {
    return reached();
  }
  ayanami::abstract::GoalCheckerTolerances getTolerances() const override {
    return {};
  }

 private:
  std::function<bool()> reached;
};

class AlwaysTrueProgressChecker
    : public ayanami::abstract::AbstractProgressChecker {
 public:
  void reset() override {}
  bool check(const PoseStamped&) override { return true; }
};

// ControllerAction 派生: createExecution 内注入 checker (NavigationServerBase
// 生产路径注入, 直连 action 测试用此子类)。
class TestControllerAction : public ayanami::components::ControllerAction {
 public:
  using ControllerAction::ControllerAction;
  void initCheckers(ayanami::abstract::AbstractGoalChecker::Ptr goal,
                    ayanami::abstract::AbstractProgressChecker::Ptr progress) {
    this->goal = std::move(goal);
    this->progress = std::move(progress);
  }

 protected:
  ayanami::components::ControllerExecution::Ptr createExecution() override {
    auto exec = ControllerAction::createExecution();
    exec->setGoalChecker(goal);
    exec->setProgressChecker(progress);
    return exec;
  }

 private:
  ayanami::abstract::AbstractGoalChecker::Ptr goal;
  ayanami::abstract::AbstractProgressChecker::Ptr progress;
};

Path makePath(std::size_t size) {
  Path path;
  path.header.frame_id = "map";
  for (std::size_t i = 0; i < size; ++i) {
    PoseStamped p =
        PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
    p.pose.position = Eigen::Vector3d(static_cast<double>(i), 0.0, 0.0);
    path.poses.push_back(p);
  }
  return path;
}

template <typename Task>
bool waitReady(Task& t, int max_ms = 5000) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(max_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (t.ready()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return t.ready();
}

}  // namespace

TEST(ControllerSession, tenPosePlanRunsFramesAndArrivesAtGoal) {
  auto& robot_state = ayanami::components::RobotStateController::instance();
  robot_state.initialize("map", "odom", "base_link", 10.0);
  robot_state.setTransform("map", "base_link", ayanami::Timestamp::latest(),
                           Eigen::Isometry3d::Identity());
  auto controller = std::make_shared<RecordingController>();
  TestControllerAction action(controller);
  action.initCheckers(
      std::make_shared<LambdaGoalChecker>(
          [&controller] { return controller->calls >= 3; }),
      std::make_shared<AlwaysTrueProgressChecker>());

  // 捕获 on_velocity_command 旁路 (controller execution 每帧发布 cmd_vel)。
  std::atomic<int> cmd_count{0};
  ayanami::CallbackSubscription cmd_cb =
      ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe(
          [&](const TwistStamped&) { ++cmd_count; });

  ExePathGoal goal;
  goal.path = makePath(10);
  auto task = action.acceptGoal(goal);
  ASSERT_TRUE(task.valid());
  ASSERT_TRUE(waitReady(task));

  EXPECT_EQ(task.status(), GoalStatus::SUCCEEDED);
  EXPECT_TRUE(task.result().reached);
  EXPECT_EQ(task.result().error, ErrorCode::NONE);
  // 持续多帧 computeVelocityCommand (至少 3 帧)
  EXPECT_GE(controller->calls.load(), 3);
  // 中间帧 cmd_vel 已经 on_velocity_command 旁路发布
  EXPECT_GE(cmd_count.load(), 1);

  // setPlan 收到完整 10-pose 路径
  {
    std::lock_guard<std::mutex> lock(controller->mutex);
    ASSERT_FALSE(controller->applied_sizes.empty());
    EXPECT_EQ(controller->applied_sizes.back(), 10u);
  }

}  // cmd_cb 析构自动退订
