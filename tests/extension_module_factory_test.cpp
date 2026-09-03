// Requirement 2/3 focused tests:
//  A. 所有生产模块 .so 都经公共工厂 create_extension_module 装载, 且类型化
//     dynamic_pointer_cast 到正确插件族类型 (不再有任意 create* 符号)。
//  B. 默认树扩展模块 (navigation_behavior_tree) 可独立装载: load_module →
//     BehaviorTreeExtension cast → registerNodes + registerTrees → createTree →
//     tick (执行树)。
//
// argv (一律为生产模块 .so 的绝对路径):
//   1=host(costmap_navigation) 2=dwb 3=smac 4=savitzky 5=pid
//   6=spin 7=backup 8=wait 9=clear 10=goal_checker 11=progress_checker
//   12=navigation_behavior_tree

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <Eigen/Geometry>

#include <ayanami/abstract/core/abstract_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/abstract/core/abstract_progress_checker.hpp>
#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/behavior_tree/action_nodes.hpp>
#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/behavior_tree/extension.hpp>
#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/utility/extension_module.hpp>

using namespace ayanami::behavior_tree;

namespace {

std::string g_host_so, g_dwb_so, g_smac_so, g_savitzky_so, g_pid_so;
std::string g_spin_so, g_backup_so, g_wait_so, g_clear_so;
std::string g_goal_checker_so, g_progress_checker_so, g_bt_ext_so;

// 类型化装载断言: ExtensionModule::load_module → dynamic_pointer_cast<T> 非空。
template <typename T>
void expectLoadsAs(const std::string& so, const char* what = nullptr) {
  auto ext = ayanami::ExtensionModule::load_module(so);
  SCOPED_TRACE(what ? what : so);
  EXPECT_NE(ext, nullptr) << "load_module failed: " << so;
  if (!ext) {
    return;
  }
  EXPECT_NE(std::dynamic_pointer_cast<T>(ext), nullptr)
      << so << " is not a " << typeid(T).name();
}

}  // namespace

// ========== A. 所有生产模块经公共工厂 + 类型化 cast 装载 ==========

TEST(ExtensionModuleFactoryTest, allProductionModulesLoadViaCommonFactory) {
  expectLoadsAs<ayanami::abstract::AbstractNavigation>(g_host_so, "host");
  expectLoadsAs<ayanami::abstract::AbstractController>(g_dwb_so, "dwb");
  expectLoadsAs<ayanami::costmap::CostmapPlanner>(g_smac_so, "smac");
  expectLoadsAs<ayanami::costmap::CostmapSmoother>(g_savitzky_so, "savitzky");
  expectLoadsAs<ayanami::abstract::AbstractController>(g_pid_so, "pid");
  expectLoadsAs<ayanami::costmap::CostmapRecovery>(g_spin_so, "spin");
  expectLoadsAs<ayanami::costmap::CostmapRecovery>(g_backup_so, "backup");
  expectLoadsAs<ayanami::costmap::CostmapRecovery>(g_wait_so, "wait");
  expectLoadsAs<ayanami::costmap::CostmapRecovery>(g_clear_so, "clear");
  expectLoadsAs<ayanami::abstract::AbstractGoalChecker>(g_goal_checker_so,
                                                        "goal_checker");
  expectLoadsAs<ayanami::abstract::AbstractProgressChecker>(
      g_progress_checker_so, "progress_checker");
  expectLoadsAs<ayanami::behavior_tree::BehaviorTreeExtension>(g_bt_ext_so,
                                                               "bt_ext");
}

TEST(ExtensionModuleFactoryTest, nonNavigationModuleIsNotNavigation) {
  // 反例: goal_checker .so 不得被 cast 成 AbstractNavigation。
  auto ext = ayanami::ExtensionModule::load_module(g_goal_checker_so);
  ASSERT_NE(ext, nullptr);
  EXPECT_EQ(std::dynamic_pointer_cast<ayanami::abstract::AbstractNavigation>(ext),
            nullptr);
}

// ========== B. 默认树扩展独立装载 + 注册 + 建树 + 执行 ==========

namespace {

// 最小同步 NavigationServer 桩: 每个 request 立即返回成功结果。
class StubNavServer : public ayanami::components::NavigationServerBase {
 public:
  using PathServer = ayanami::components::ActionServer<
      ayanami::actions::GetPathGoal, ayanami::actions::GetPathResult,
      ayanami::actions::GetPathFeedback>;
  using SmoothServer = ayanami::components::ActionServer<
      ayanami::actions::SmoothPathGoal, ayanami::actions::SmoothPathResult,
      ayanami::actions::SmoothPathFeedback>;
  using ExeServer = ayanami::components::ActionServer<
      ayanami::actions::ExePathGoal, ayanami::actions::ExePathResult,
      ayanami::actions::ExePathFeedback>;
  using RecoveryServer = ayanami::components::ActionServer<
      ayanami::actions::RecoveryGoal, ayanami::actions::RecoveryResult,
      ayanami::actions::RecoveryFeedback>;

  ayanami::components::TaskHandle<ayanami::actions::GetPathResult> requestPath(
      const ayanami::actions::GetPathGoal& goal) override {
    auto task = path_server.acceptGoal(goal);
    ayanami::actions::GetPathResult r;
    r.failure = ayanami::FailureCode::None;
    r.path.header.frame_id = "map";
    r.path.poses.push_back(pose(0.0, 0.0));
    r.path.poses.push_back(pose(1.0, 0.0));
    r.session_id = goal.session_id;
    r.plan_version = goal.plan_version;
    path_server.publishResult(task.goal_id(),
                               ayanami::actions::GoalStatus::SUCCEEDED, r);
    return task;
  }

  ayanami::components::TaskHandle<ayanami::actions::SmoothPathResult>
  requestSmoothing(const ayanami::actions::SmoothPathGoal& goal) override {
    auto task = smooth_server.acceptGoal(goal);
    ayanami::actions::SmoothPathResult r;
    r.failure = ayanami::FailureCode::None;
    r.smoothed_path = goal.path;
    r.session_id = goal.session_id;
    r.plan_version = goal.plan_version;
    smooth_server.publishResult(task.goal_id(),
                                 ayanami::actions::GoalStatus::SUCCEEDED, r);
    return task;
  }

  ayanami::components::TaskHandle<ayanami::actions::ExePathResult>
  requestExecution(const ayanami::actions::ExePathGoal& goal) override {
    auto task = exe_server.acceptGoal(goal);
    ayanami::actions::ExePathResult r;
    r.failure = ayanami::FailureCode::None;
    r.reached = true;
    r.session_id = goal.session_id;
    r.plan_version = goal.plan_version;
    exe_server.publishResult(task.goal_id(),
                              ayanami::actions::GoalStatus::SUCCEEDED, r);
    return task;
  }

  ayanami::components::TaskHandle<ayanami::actions::RecoveryResult>
  requestRecovery(const ayanami::actions::RecoveryGoal& goal) override {
    auto task = recovery_server.acceptGoal(goal);
    ayanami::actions::RecoveryResult r;
    r.failure = ayanami::FailureCode::None;
    r.session_id = goal.session_id;
    r.plan_version = goal.plan_version;
    recovery_server.publishResult(
        task.goal_id(), ayanami::actions::GoalStatus::SUCCEEDED, r);
    return task;
  }

  bool requestPlanUpdate(uint64_t, const ayanami::actions::ExePathGoal&) override {
    return true;
  }

  PathServer path_server;
  SmoothServer smooth_server;
  ExeServer exe_server;
  RecoveryServer recovery_server;

 private:
  static ayanami::geometry_msgs::PoseStamped pose(double x, double y) {
    ayanami::geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ayanami::Timestamp{static_cast<std::int64_t>(x * 1000)};
    p.pose.position = Eigen::Vector3d(x, y, 0.0);
    return p;
  }
};

}  // namespace

TEST(ExtensionModuleFactoryTest, defaultTreeExtensionLoadsRegistersAndExecutes) {
  auto ext = std::dynamic_pointer_cast<BehaviorTreeExtension>(
      ayanami::ExtensionModule::load_module(g_bt_ext_so));
  ASSERT_NE(ext, nullptr) << "load default BT extension failed";
  EXPECT_EQ(ext->name(), "navigation");
  EXPECT_EQ(ext->defaultTreeId(), "NavigateWithReplanningAndSmoothing");

  // 独立注册 (不依赖 pilot): 节点 + 默认树 XML 全来自扩展。
  BT::BehaviorTreeFactory factory;
  EXPECT_NO_THROW(ext->registerNodes(factory));
  std::vector<std::string> tree_ids;
  EXPECT_NO_THROW(tree_ids = ext->registerTrees(factory));
  ASSERT_FALSE(tree_ids.empty());

  auto server = std::make_shared<StubNavServer>();
  auto bb = BT::Blackboard::create();
  bb->set<ayanami::components::NavigationRuntime::Ptr>(k_navigation_runtime,
                                                       server);
  bb->set<ayanami::geometry_msgs::PoseStamped>(k_goal,
      ayanami::geometry_msgs::PoseStamped{});
  bb->set<ayanami::nav_msgs::Path>(k_raw_path, ayanami::nav_msgs::Path{});
  bb->set<ayanami::nav_msgs::Path>(k_smoothed_path, ayanami::nav_msgs::Path{});
  bb->set<double>(k_planner_frequency, 100.0);
  bb->set<double>(k_smoother_frequency, -1.0);
  bb->set<double>(k_controller_frequency, 20.0);
  bb->set<std::string>(k_planner_id, "x");
  bb->set<std::string>(k_smoother_id, "x");
  bb->set<std::string>(k_controller_id, "x");
  bb->set<std::string>(k_recovery_id, "x");
  bb->set<std::string>(k_clear_costmap_id, "x");
  bb->set<std::string>(k_spin_id, "x");
  bb->set<std::string>(k_wait_id, "x");
  bb->set<std::string>(k_backup_id, "x");
  bb->set<uint64_t>(k_session_id, 0u);
  bb->set<uint64_t>(k_plan_version, 0u);
  bb->set<uint64_t>(k_reset_stuck_token, 1u);
  bb->set<uint64_t>(k_goal_revision, 0u);
  bb->set<ayanami::FailureContext>(k_failure_context, ayanami::FailureContext{});
  EXPECT_NO_THROW(validateRequiredBlackboardEntries(bb));

  auto tree = factory.createTree(ext->defaultTreeId(), bb);
  // 执行树: mock path/smooth/exe 同步成功 → 数次 tick 后整树 SUCCESS。
  const int deadline_ms = 2000;
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(deadline_ms);
  while (std::chrono::steady_clock::now() < deadline &&
         status == BT::NodeStatus::RUNNING) {
    status = tree.tickExactlyOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_host_so = argv[1];
  if (argc > 2) g_dwb_so = argv[2];
  if (argc > 3) g_smac_so = argv[3];
  if (argc > 4) g_savitzky_so = argv[4];
  if (argc > 5) g_pid_so = argv[5];
  if (argc > 6) g_spin_so = argv[6];
  if (argc > 7) g_backup_so = argv[7];
  if (argc > 8) g_wait_so = argv[8];
  if (argc > 9) g_clear_so = argv[9];
  if (argc > 10) g_goal_checker_so = argv[10];
  if (argc > 11) g_progress_checker_so = argv[11];
  if (argc > 12) g_bt_ext_so = argv[12];
  return RUN_ALL_TESTS();
}
