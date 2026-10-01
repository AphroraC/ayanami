// modules/costmap_navigation (AbstractNavigation host module).
//
// M1 key milestone: assembling all core library components (CostmapNavigationServer + the 4 module families +
// BehaviorTreePilot) into the dlopen entry module. This test:
// - ExtensionModule loads libayanami_costmap_navigation.so
// - initialize: configureCoreMaps + loadModules (dlopen astar/minsnap/spin
// real algorithm modules + a test controller) + createActions + pilot wiring (
// configure -> loadFromFile -> setNavigationRuntime)
// - start publishes the first visualization (the per-layer costmap slots are captured)
// - setGoal -> navigation_thread runs the BT -> raw_path/smoothed_path land in the blackboard +
// on_velocity_command receives a non-zero velocity (the first cmd_vel.linear frame is initialized, non-NaN)
// - Preemption: the second setGoal -> zero velocity first + session_id increments + the BT tree is not rebuilt
// - destruction order does not crash

#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <ayanami/behavior_tree/blackboard_protocol.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <costmap_navigation/costmap_navigation.hpp>
#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/utility/extension_module.hpp>

using ayanami::testing::testTransforms;
namespace {

/// @brief Declare the four recovery roles the production tree references.
///
/// The tree names recoveries by ROLE (spin/wait/backup/clear_costmap), and the BT
/// extension maps each role to a module through config_recovery. A role nobody
/// declares resolves to an empty id, which the pilot now refuses to build a tree
/// with: a recovery that cannot resolve is a configuration error, not something
/// to discover while the robot is stuck.
/// @return the JSON array literal for the navigation section's config_recovery.
std::string writeRecoveryRoleFiles(
  const std::filesystem::path& dir,
  const std::string& module_name,
  const std::string& so_name = "") {
  const char* roles[] = {"clear_costmap", "spin", "wait", "backup"};
  std::string list;
  for (const char* role : roles) {
    const std::string file = std::string("role_") + role + ".json";
    {
      std::ofstream ofs(dir / file);
      ofs << "{ \"recovery\": { \"name\": \"" << module_name << "\", \"role\": \"" << role << "\"";
      if (!so_name.empty()) {
        ofs << ", \"so_name\": \"" << so_name << "\"";
      }
      // The module's own (empty) parameter section: the loader requires a
      // section named after the module, so "reads no parameters" is declared
      // rather than looking like a name/section mismatch.
      ofs << " }, \"" << module_name << "\": {} }";
    }
    if (!list.empty()) {
      list += ", ";
    }
    list += "\"" + file + "\"";
  }
  return "[" + list + "]";
}

/// @brief Wait until the pilot's tick thread has consumed the goal and produced
/// this attempt's blackboard. The assertion is on the blackboard rather than on
/// attemptActive(): with synchronous module mocks the whole tree can reach a
/// terminal status within a tick, and a finished attempt keeps its blackboard as
/// the record while the live instance is already gone.
bool waitUntilPilotAttempt(const ayanami::behavior_tree::BehaviorTreePilot* pilot, int timeout_ms = 2000) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (pilot->blackboard() != nullptr) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

using ayanami::behavior_tree::k_goal_revision;
using ayanami::behavior_tree::k_raw_path;
using ayanami::behavior_tree::k_session_id;
using ayanami::behavior_tree::k_smoothed_path;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::modules::CostmapNavigation;
using ayanami::nav_msgs::Path;

// argv: module .so + 7 module .so files (astar/minsnap/pid/test_controller/spin/
// goal_checker/progress_checker).
std::string g_module_so;
std::string g_astar_so;
std::string g_minsnap_so;
std::string g_pid_so;
std::string g_test_controller_so;
std::string g_spin_so;
std::string g_goal_checker_so;
std::string g_progress_checker_so;

// Captures on_velocity_command events.
std::vector<TwistStamped> g_cmds;
std::mutex g_cmd_mutex;
// Preemption marker: after the setGoal pre-emption the observer records the zero-velocity count.
std::atomic<bool> g_preempt_armed{false};
std::atomic<int> g_zero_after_preempt{0};
// Visualization capture.
std::vector<std::string> g_viz_names;
std::mutex g_viz_mutex;

// ---------------- Test utilities ----------------

bool isZeroTwist(const TwistStamped& t) {
  return t.twist.linear.norm() < 1e-9 && t.twist.angular.norm() < 1e-9;
}

bool allFinite(const TwistStamped& t) {
  return std::isfinite(t.twist.linear.x()) && std::isfinite(t.twist.linear.y()) && std::isfinite(t.twist.linear.z()) &&
         std::isfinite(t.twist.angular.x()) && std::isfinite(t.twist.angular.y()) && std::isfinite(t.twist.angular.z());
}

PoseStamped makePose(double x, double y = 0.0) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

std::filesystem::path makeTempDir() {
  std::filesystem::path dir = std::filesystem::temp_directory_path() / "ayanami_costmap_navigation_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void write_file(const std::filesystem::path& path, const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

std::string moduleJson(const std::string& module, const std::string& name, const std::string& so_name) {
  // The module's own (empty) parameter section: the loader requires a section
  // named after the module, so that "this module reads no parameters" is
  // declared rather than indistinguishable from a name/section mismatch.
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name + "\",\n    \"so_name\": \"" + so_name +
         "\"\n  },\n  \"" + name + "\": {}\n}\n";
}

/// Unclosed version (to be concatenated with the module parameter section into a single file).
std::string moduleJsonOpen(const std::string& module, const std::string& name, const std::string& so_name) {
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name + "\",\n    \"so_name\": \"" + so_name +
         "\"\n  },\n  \"" + name + "\": {},\n";
}

/// Generates a temp config dir. so_name uses the real module absolute path injected via argv.
std::filesystem::path makeConfigDir() {
  const auto dir = makeTempDir() / "cfg";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  std::filesystem::create_directories(dir / "BT");

  write_file(
    dir / "config.json",
    "{\n  \"global\": {\n"
    "    \"config_navigation\": \"costmap_navigation.json\"\n"
    "  }\n}\n");

  write_file(
    dir / "planner.json",
    moduleJsonOpen("planner", "test_planner", g_astar_so) +
      "  \"test_planner\": {\n"
      "    \"use_diagonal\": true,\n    \"tolerance\": 0.0,\n"
      "    \"cost_factor\": 0.5,\n    \"lethal_cost\": 254,\n"
      "    \"neutral_cost\": 1.0,\n    \"use_final_approach\": false\n"
      "  }\n}\n");
  write_file(
    dir / "smoother.json",
    moduleJsonOpen("smoother", "test_smoother", g_minsnap_so) +
      "  \"test_smoother\": {\n"
      "    \"do_refinement\": true,\n    \"refinement_num\": 2\n"
      "  }\n}\n");
  write_file(dir / "controller.json", moduleJson("controller", "test_controller", g_test_controller_so));
  write_file(
    dir / "recovery.json",
    "{\n  \"recovery\": {\n    \"name\": \"test_spin\",\n"
    "    \"role\": \"spin\",\n    \"so_name\": \"" +
      g_spin_so +
      "\",\n"
      "    \"sim_cycle_dt\": 0.01\n"
      "  },\n"
      "  \"test_spin\": {\n"
      "    \"spin_dist\": 1.57,\n    \"max_w\": 0.5,\n"
      "    \"collision_radius\": 0.3\n"
      "  }\n}\n");
  // goal/progress checker modules (the execution needs injected checkers; missing ones are judged InternalError).
  write_file(
    dir / "goal_checker.json",
    moduleJsonOpen("goal_checker", "simple_goal_checker", g_goal_checker_so) +
      "  \"simple_goal_checker\": {\n"
      "    \"position_tolerance\": 0.10,\n"
      "    \"angle_tolerance\": 0.20,\n"
      "    \"linear_tolerance\": 0.10,\n"
      "    \"angular_tolerance\": 0.10\n"
      "  }\n}\n");
  write_file(
    dir / "progress_checker.json",
    moduleJsonOpen("progress_checker", "simple_progress_checker", g_progress_checker_so) +
      "  \"simple_progress_checker\": {\n"
      "    \"required_movement\": 0.1,\n"
      "    \"movement_angle_required\": 0.2,\n"
      "    \"time_period\": 5.0\n"
      "  }\n}\n");

  // 20x20 @ 0.5 = 10m x 10m, origin (0,0); all-FREE static map (A* can plan from (0,0)
  // to (2,0)).
  const std::string pgm_path = (dir / "map.pgm").string();
  {
    std::ofstream out(pgm_path, std::ios::binary);
    out << "P5\n20 20\n255\n";
    for (int i = 0; i < 400; ++i) {
      out.put(static_cast<char>(255));
    }
  }
  write_file(dir / "map.json", R"({"map":{"image":"map.pgm","resolution":0.5,"origin":[0.0,0.0,0.0]}})");

  // Costmap parameters are merged into costmap_navigation.json (the old costmap.json was removed).
  {
    std::ofstream nav(dir / "costmap_navigation.json");
    nav << "{\n  \"navigation\": {\n"
        << "    \"behavior_tree_extension\": \"" AYANAMI_TEST_BT_EXT_SO "\",\n"
        << "    \"behavior_tree_xml\": \"BT/navigation.xml\",\n"
        << "    \"behavior_tree_id\": \"NavigateWithReplanningAndSmoothing\",\n"
        << "    \"planner_id\": \"test_planner\",\n"
        << "    \"smoother_id\": \"test_smoother\",\n"
        << "    \"controller_id\": \"test_controller\",\n"
        << "    \"config_planner\": \"planner.json\",\n"
        << "    \"config_smoother\": \"smoother.json\",\n"
        << "    \"config_controller\": \"controller.json\",\n"
        << "    \"config_goal_checker\": \"goal_checker.json\",\n"
        << "    \"config_progress_checker\": \"progress_checker.json\",\n"
        << "    \"config_recovery\": " << writeRecoveryRoleFiles(dir, "test_spin", g_spin_so) << ",\n"
        << "    \"clear_costmap_id\": \"test_spin\",\n"
        << "    \"spin_id\": \"test_spin\",\n"
        << "    \"wait_id\": \"test_spin\",\n"
        << "    \"backup_id\": \"test_spin\",\n"
        << "    \"goal_checker_id\": \"simple_goal_checker\",\n"
        << "    \"progress_checker_id\": \"simple_progress_checker\",\n"
        << "    \"planner_frequency\": 100.0,\n"
        << "    \"smoother_frequency\": -1.0,\n"
        << "    \"controller_frequency\": 20.0,\n"
        << "    \"tick_frequency\": 100.0\n"
        << "  },\n"
        << "  \"costmap\": {\n"
        << "    \"map_frame\": \"map\",\n"
        << "    \"robot_frame\": \"base_link\",\n"
        << "    \"resolution\": 0.5\n"
        << "  },\n"
        << "  \"costmap_global\": {\n"
        << "    \"rolling_window\": false,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"layers\": [\"layer_static.json\", \"layer_inflation.json\"]\n"
        << "  },\n"
        << "  \"costmap_local\": {\n"
        << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
        << "    \"rolling_window\": true,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"update_frequency\": 10.0,\n"
        << "    \"observation_keep_time\": 0.5,\n"
        << "    \"layers\": [\"layer_obstacle.json\", \"layer_inflation.json\"]\n"
        << "  }\n"
        << "}\n";
  }
  write_file(
    dir / "layer_static.json",
    "{\"layer\": {\"name\": \"static_layer\", \"so_name\": "
    "\"libayanami_costmap_static_layer.so\"},\n \"static_layer\": "
    "{\"static_map\": \"" +
      pgm_path + "\"}}\n");
  write_file(
    dir / "layer_inflation.json",
    "{\"layer\": {\"name\": \"inflation_layer\", \"so_name\": "
    "\"libayanami_costmap_inflation_layer.so\"},\n \"inflation_layer\": "
    "{\"inflation_radius\": 1.0, \"inscribed_radius\": 0.8, "
    "\"cost_scaling_factor\": 2.0}}\n");
  write_file(
    dir / "layer_obstacle.json",
    "{\"layer\": {\"name\": \"obstacle_layer\", \"so_name\": "
    "\"libayanami_costmap_obstacle_layer.so\"},\n \"obstacle_layer\": {"
    "\"combination\": \"overwrite\", \"footprint_clearing\": true, "
    "\"observation_sources\": [\"cloud\", \"scan\"], "
    "\"cloud\": {\"type\": \"pointcloud\"}, "
    "\"scan\": {\"type\": \"laserscan\"}}}\n");

  write_file(
    dir / "BT" / "navigation.xml",
    R"(<root BTCPP_format="4" main_tree_to_execute="NavigateWithReplanningAndSmoothing">
  <BehaviorTree ID="NavigateWithReplanningAndSmoothing">
    <RecoveryNode name="NavigateRecovery" number_of_retries="6">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="{planner_frequency}">
          <RecoveryNode name="ComputePath" number_of_retries="1">
            <Sequence>
              <ComputePathToPose goal="{goal}" path="{raw_path}"/>
              <IsPathValid path="{raw_path}"/>
              <SmoothPath  input_path="{raw_path}" path="{smoothed_path}"/>
              <IsPathValid path="{smoothed_path}"/>
            </Sequence>
            <Sequence>
              <Recovery recovery="{clear_costmap_id}"/>
              <AlwaysFailure/>
            </Sequence>
          </RecoveryNode>
        </RateController>
        <RecoveryNode name="FollowPath" number_of_retries="1">
          <FollowPath path="{smoothed_path}" frequency="{controller_frequency}"/>
          <Sequence>
            <Recovery recovery="{clear_costmap_id}"/>
            <AlwaysFailure/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryActions">
        <GoalUpdated goal="{goal}"/>
        <RecoverySequence name="RecoveryActions">
          <Sequence><Recovery recovery="{spin_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{wait_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{backup_id}"/>  <AlwaysFailure/></Sequence>
        </RecoverySequence>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>)");

  return dir;
}

/// Gives the robot a static map-frame pose at the origin: the controller
/// loop requires a valid robot state (pose via the robot-state transform
/// lookup) to compute and emit velocity commands.
void giveRobotStaticPose() {
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", "map", "base_link", 0.25);
  testTransforms().set("map", "base_link", ayanami::Timestamp::latest(), Eigen::Isometry3d::Identity());
}

// RAII subscription handle: replaced on attach, unsubscribed on destruction at detach (no dangling id).
ayanami::CallbackSubscription g_cmd_cb;
ayanami::CallbackSubscription g_viz_cb;
ayanami::CallbackSubscription g_viz_inflation_cb;
ayanami::CallbackSubscription g_viz_local_cb;

void attachObservers() {
  g_cmds.clear();
  g_zero_after_preempt.store(0);
  g_preempt_armed.store(false);
  g_cmd_cb = ayanami::interfaces::NavigationCallbacks::on_velocity_command.subscribe([](const TwistStamped& t) {
    std::lock_guard<std::mutex> lock(g_cmd_mutex);
    g_cmds.push_back(t);
    if (g_preempt_armed.load() && isZeroTwist(t)) {
      g_zero_after_preempt.fetch_add(1);
    }
  });
  g_viz_names.clear();
  // Per-layer artifact slots: each emission is labeled "<map>:<artifact>".
  g_viz_cb = ayanami::interfaces::NavigationCallbacks::on_global_costmap_static.subscribe(
    [](const ayanami::nav_msgs::OccupancyGrid&) {
      std::lock_guard<std::mutex> lock(g_viz_mutex);
      g_viz_names.push_back("global_costmap:static");
    });
  g_viz_inflation_cb = ayanami::interfaces::NavigationCallbacks::on_global_costmap_inflation.subscribe(
    [](const ayanami::nav_msgs::OccupancyGrid&) {
      std::lock_guard<std::mutex> lock(g_viz_mutex);
      g_viz_names.push_back("global_costmap:inflation");
    });
  g_viz_local_cb = ayanami::interfaces::NavigationCallbacks::on_local_costmap_inflation.subscribe(
    [](const ayanami::nav_msgs::OccupancyGrid&) {
      std::lock_guard<std::mutex> lock(g_viz_mutex);
      g_viz_names.push_back("local_costmap:inflation");
    });
}

void detachObservers() {
  g_cmd_cb.reset();
  g_viz_cb.reset();
  g_viz_inflation_cb.reset();
  g_viz_local_cb.reset();
}

int nonZeroCmdCount() {
  std::lock_guard<std::mutex> lock(g_cmd_mutex);
  int count = 0;
  for (const auto& t : g_cmds) {
    if (!isZeroTwist(t)) {
      ++count;
    }
  }
  return count;
}

bool anyCmdHasNaNLinear() {
  std::lock_guard<std::mutex> lock(g_cmd_mutex);
  for (const auto& t : g_cmds) {
    if (!allFinite(t)) {
      return true;
    }
  }
  return false;
}

template <typename Fn>
bool waitUntil(const Fn& fn, int timeout_ms = 3000) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (fn()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return fn();
}

/// Loads the host module via the common factory -> AbstractNavigation::Ptr -> dynamic_cast<CostmapNavigation*>.
/// Owning reference: component lifetime = process lifetime (no dlclose); the host shared_ptr is intentionally leaked to
/// stay alive, and the test is still responsible for deleting the host.
CostmapNavigation* createHost(const std::filesystem::path& config_dir) {
  auto nav =
    std::dynamic_pointer_cast<ayanami::abstract::AbstractNavigation>(ayanami::ExtensionModule::loadModule(g_module_so));
  EXPECT_NE(nav, nullptr);
  if (!nav) {
    return nullptr;
  }
  auto* host = dynamic_cast<CostmapNavigation*>(nav.get());
  EXPECT_NE(host, nullptr);
  if (host == nullptr) {
    return nullptr;
  }
  host->initialize(config_dir.string());
  (void)new std::shared_ptr<ayanami::abstract::AbstractNavigation>(std::move(nav));  // leak-persist
  return host;
}

}  // namespace

// ---------------- Host assembly (configureCoreMaps + loadModules +) ----------------

TEST(CostmapNavigationTest, dlopenAllAlgorithmModulesLoad) {
  // All 4 real algorithm module .so files go through the common create_extension_module factory and obtain the correct
  // module family type via typed dynamic_pointer_cast.
  EXPECT_NE(
    std::dynamic_pointer_cast<ayanami::costmap::CostmapPlanner>(ayanami::ExtensionModule::loadModule(g_astar_so)),
    nullptr);
  EXPECT_NE(
    std::dynamic_pointer_cast<ayanami::costmap::CostmapSmoother>(ayanami::ExtensionModule::loadModule(g_minsnap_so)),
    nullptr);
  EXPECT_NE(
    std::dynamic_pointer_cast<ayanami::abstract::AbstractController>(ayanami::ExtensionModule::loadModule(g_pid_so)),
    nullptr);
  EXPECT_NE(
    std::dynamic_pointer_cast<ayanami::costmap::CostmapRecovery>(ayanami::ExtensionModule::loadModule(g_spin_so)),
    nullptr);
}

TEST(CostmapNavigationTest, initializeBuildsFullStack) {
  const auto dir = makeConfigDir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);

  // configureCoreMaps: global/local costmaps built
  ASSERT_NE(host->navigationServer().costmapContext(), nullptr);
  ASSERT_NE(host->navigationServer().globalCostmap(), nullptr);
  ASSERT_NE(host->navigationServer().localCostmap(), nullptr);

  // loadModules: planner/smoother/controller/recovery actions built
  ASSERT_NE(host->navigationServer().plannerAction(), nullptr);
  ASSERT_NE(host->navigationServer().smootherAction(), nullptr);
  ASSERT_NE(host->navigationServer().controllerAction(), nullptr);
  ASSERT_NE(host->navigationServer().recoveryAction("test_spin"), nullptr);

  // pilot is configured + loadFromFile + setNavigationRuntime
  // (loadFromFile validated the 17 blackboard entries -> tree built)
  // The stack is wired once the pilot's factory is built; the per-attempt
  // blackboard appears when the first goal is consumed (see setGoalRunsBt...).
  EXPECT_TRUE(host->pilot().ready());

  host->stop();
  delete host;
}

// ---------------- start / spinOnce visualization bypass ----------------

TEST(CostmapNavigationTest, startPublishesVisualizationAndSpinOnceRepeats) {
  attachObservers();
  const auto dir = makeConfigDir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);

  host->start();
  EXPECT_TRUE(waitUntil([&] {
    std::lock_guard<std::mutex> lock(g_viz_mutex);
    return !g_viz_names.empty();
  }));

  bool has_global = false;
  bool has_local = false;
  {
    std::lock_guard<std::mutex> lock(g_viz_mutex);
    for (const auto& n : g_viz_names) {
      if (n.rfind("global_costmap:", 0) == 0) has_global = true;
      if (n.rfind("local_costmap:", 0) == 0) has_local = true;
    }
  }
  EXPECT_TRUE(has_global);
  EXPECT_TRUE(has_local);

  // spinOnce: incremental publishing (nav2 Costmap2DPublisher semantics) --
  // an UNCHANGED map emits nothing, so repeated rate-limited calls no longer
  // grow the full-grid emission log.
  const size_t before = g_viz_names.size();
  for (int i = 0; i < 5; ++i) {
    host->spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
  }
  EXPECT_EQ(g_viz_names.size(), before);

  host->stop();
  delete host;
  detachObservers();
}

// ---------------- per-layer costmap emission ----------------
// Generation-gated slot semantics locked end to end: configure emits each
// artifact once; a content update re-emits exactly that map's artifact; an
// unchanged map stays silent.

TEST(CostmapNavigationTest, layerEmissionOnUpdateAndSilence) {
  attachObservers();
  const auto dir = makeConfigDir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);
  {
    // Consistent frame tree for the local (odom-frame) map: the robot state
    // resolves map <- odom <- base_link; the lidar hangs off base_link.
    auto& rsc = ayanami::components::RobotStateController::instance();
    rsc.initialize("map", "odom", "base_link", 0.25);
    const auto identity = Eigen::Isometry3d::Identity();
    testTransforms().set("map", "odom", ayanami::Timestamp::latest(), identity);
    testTransforms().set("odom", "base_link", ayanami::Timestamp::latest(), identity);
    testTransforms().set("odom", "lidar", ayanami::Timestamp::latest(), identity);
  }
  host->start();

  // Configure-time initial emission: every artifact fired at least once.
  ASSERT_TRUE(waitUntil([&] {
    std::lock_guard<std::mutex> lock(g_viz_mutex);
    return g_viz_names.size() >= 3;
  }));

  auto countLabel = [&](const std::string& label) {
    std::lock_guard<std::mutex> lock(g_viz_mutex);
    return static_cast<int>(std::count(g_viz_names.begin(), g_viz_names.end(), label));
  };
  const int global_static_before = countLabel("global_costmap:static");
  const int global_inflation_before = countLabel("global_costmap:inflation");
  const int local_before = countLabel("local_costmap:inflation");

  // Feed an observation: obstacle at base_link (0.3, 0) -- the LOCAL map
  // content changes -> exactly the local artifact re-emits (writer inline
  // path emits on the map cycle; no spinOnce polling involved).
  auto& rsc = ayanami::components::RobotStateController::instance();
  ayanami::sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  pcl::PointXYZI hit;
  hit.x = 0.3f;
  hit.y = 0.0f;
  hit.z = 0.0f;
  cloud.cloud.points.push_back(hit);
  host->setPointcloud(cloud);
  ASSERT_TRUE(host->navigationServer().waitForMapIdle(2.0));
  ASSERT_TRUE(waitUntil([&] { return countLabel("local_costmap:inflation") > local_before; }));
  EXPECT_EQ(countLabel("global_costmap:static"), global_static_before);
  EXPECT_EQ(countLabel("global_costmap:inflation"), global_inflation_before);

  host->stop();
  delete host;
  detachObservers();
}

// ---------------- static-zone mark filter ----------------
// Cloud hits landing in a cell the GLOBAL map already blocks (>= 253) must
// NOT be marked into the local map: the static map owns that geometry, and
// re-marking it from live range noise made the local halo boundary flicker
// across paths that legally pass ~0.3 m from walls (sustained IsPathValid
// FAIL churn, 2026-09-29). Probes are chosen hermetically from the global
// grid so the test does not depend on specific map coordinates.
TEST(CostmapNavigationTest, staticZoneMarksAreFilteredFromLocalMap) {
  attachObservers();
  const auto dir = makeConfigDir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);
  {
    auto& rsc = ayanami::components::RobotStateController::instance();
    rsc.initialize("map", "odom", "base_link", 0.25);
    const auto identity = Eigen::Isometry3d::Identity();
    testTransforms().set("map", "odom", ayanami::Timestamp::latest(), identity);
    testTransforms().set("odom", "base_link", ayanami::Timestamp::latest(), identity);
    testTransforms().set("odom", "lidar", ayanami::Timestamp::latest(), identity);
  }
  host->start();
  ASSERT_TRUE(waitUntil([&] {
    std::lock_guard<std::mutex> lock(g_viz_mutex);
    return g_viz_names.size() >= 3;
  }));

  auto& server = host->navigationServer();
  auto ctx = server.costmapContext();
  ASSERT_NE(ctx, nullptr);

  // Hermetic probes: a globally-blocked (253) cell and a globally-free cell,
  // both inside the local window + lidar range, and 0.6 m apart so one
  // point's inflation cannot reach the other's cell.
  bool have_halo = false, have_free = false;
  Eigen::Vector3d halo_pt = Eigen::Vector3d::Zero();
  Eigen::Vector3d free_pt = Eigen::Vector3d::Zero();
  ctx->readGlobal([&](const ayanami::costmap::CostmapReader& g) {
    if (!g.valid()) {
      return;
    }
    const auto* grid = g.grid();
    for (unsigned my = 0; my < grid->getSizeInCellsY(); ++my) {
      for (unsigned mx = 0; mx < grid->getSizeInCellsX(); ++mx) {
        const double wx = grid->getOriginX() + (mx + 0.5) * grid->getResolution();
        const double wy = grid->getOriginY() + (my + 0.5) * grid->getResolution();
        const double d = std::hypot(wx, wy);
        if (d > 2.5) {
          continue;  // inside the local window + lidar range
        }
        const unsigned cost = grid->getCost(mx, my);
        if (!have_halo && cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          halo_pt = Eigen::Vector3d(wx, wy, 0.0);
          have_halo = true;
        }
        if (!have_free && cost == costmap_2d::FREE_SPACE && d > 0.4) {
          free_pt = Eigen::Vector3d(wx, wy, 0.0);
          have_free = true;
        }
      }
      if (have_halo && have_free &&
          (halo_pt.head<2>() - free_pt.head<2>()).norm() > 0.6) {
        return;
      }
      have_halo = have_halo && !(have_halo && have_free &&
                                 (halo_pt.head<2>() - free_pt.head<2>()).norm() <= 0.6);
    }
  });
  if (!have_halo || !have_free) {
    GTEST_SKIP() << "map geometry lacks a usable 253/free probe pair";
  }

  ayanami::sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  pcl::PointXYZI hit;
  hit.x = static_cast<float>(halo_pt.x());
  hit.y = static_cast<float>(halo_pt.y());
  hit.z = 0.0f;
  cloud.cloud.points.push_back(hit);
  hit.x = static_cast<float>(free_pt.x());
  hit.y = static_cast<float>(free_pt.y());
  cloud.cloud.points.push_back(hit);
  host->setPointcloud(cloud);
  ASSERT_TRUE(host->navigationServer().waitForMapIdle(2.0));

  int free_marked = -1;
  int halo_marked = -1;
  ctx->readLocal([&](const ayanami::costmap::CostmapReader& l) {
    ASSERT_TRUE(l.valid());
    const auto* grid = l.grid();
    unsigned mx = 0, my = 0;
    if (grid->worldToMap(free_pt.x(), free_pt.y(), mx, my)) {
      free_marked = grid->getCost(mx, my);
    }
    if (grid->worldToMap(halo_pt.x(), halo_pt.y(), mx, my)) {
      halo_marked = grid->getCost(mx, my);
    }
  });
  ASSERT_GE(free_marked, 0) << "free probe outside the local map";
  ASSERT_GE(halo_marked, 0) << "halo probe outside the local map";
  EXPECT_GE(free_marked, costmap_2d::LETHAL_OBSTACLE)
      << "a mark outside the static halo must be kept";
  EXPECT_LT(halo_marked, costmap_2d::LETHAL_OBSTACLE)
      << "a mark inside the static halo must be filtered "
         "(free_marked=" << free_marked << ")";

  host->stop();
  delete host;
  detachObservers();
}

// ---------------- setGoal: BT truly runs path -> cmd_vel ----------------

TEST(CostmapNavigationTest, setGoalRunsBtAndEmitsVelocity) {
  attachObservers();
  const auto dir = makeConfigDir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);
  host->start();
  giveRobotStaticPose();

  host->setGoal(makePose(2.0, 0.0));

  ASSERT_TRUE(waitUntilPilotAttempt(&host->pilot()));
  const auto bb = host->pilot().blackboard();
  ASSERT_NE(bb, nullptr);

  // raw_path / smoothed_path written to the pilot blackboard within 100ms
  EXPECT_TRUE(waitUntil(
    [&] {
      Path raw;
      if (!bb->get<Path>(k_raw_path, raw) || raw.poses.empty()) {
        return false;
      }
      Path smoothed;
      return bb->get<Path>(k_smoothed_path, smoothed) && !smoothed.poses.empty();
    },
    2000));

  // The BT main flow advances -> the controller emits a non-zero velocity via the on_velocity_command bypass
  EXPECT_TRUE(waitUntil([&] { return nonZeroCmdCount() >= 1; }, 2000));

  // The first cmd_vel.twist.linear frame is initialized (non-NaN)
  EXPECT_FALSE(anyCmdHasNaNLinear());

  host->stop();
  delete host;
  detachObservers();
}

// ---------------- Goal revision: controller keeps running + same session/tree ----------------

TEST(CostmapNavigationTest, newGoalKeepsControllerAndSameSessionTree) {
  attachObservers();
  const auto dir = makeConfigDir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);
  host->start();
  giveRobotStaticPose();

  host->setGoal(makePose(2.0, 0.0));
  ASSERT_TRUE(waitUntilPilotAttempt(&host->pilot()));
  const auto bb = host->pilot().blackboard();
  ASSERT_NE(bb, nullptr);

  ASSERT_TRUE(waitUntil([&] { return nonZeroCmdCount() >= 1; }, 2000));

  uint64_t sid_before = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_session_id, sid_before));
  const void* tree_addr = &host->pilot().tree();

  uint64_t revision_before = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_goal_revision, revision_before));
  g_preempt_armed.store(true);
  host->setGoal(makePose(0.5, 0.5));
  EXPECT_TRUE(waitUntil(
    [&] {
      uint64_t revision = 0;
      return bb->get<uint64_t>(k_goal_revision, revision) && revision > revision_before;
    },
    1000));
  g_preempt_armed.store(false);
  EXPECT_TRUE(waitUntil([&] { return nonZeroCmdCount() >= 1; }, 1000));

  uint64_t sid_after = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_session_id, sid_after));
  EXPECT_EQ(sid_after, sid_before);

  // The BT tree is not rebuilt: the same tree instance
  EXPECT_EQ(&host->pilot().tree(), tree_addr);

  host->stop();
  delete host;
  detachObservers();
}

// ---------------- Terminal attempt -> second attempt on the resident stack ----------------

TEST(CostmapNavigationTest, secondTerminalAttemptReusesResidentStack) {
  attachObservers();
  const auto dir = makeConfigDir();
  CostmapNavigation* host = createHost(dir);
  ASSERT_NE(host, nullptr);
  host->start();
  giveRobotStaticPose();  // robot at the map origin

  std::mutex result_mutex;
  std::vector<ayanami::actions::GoalStatus> statuses;
  ayanami::CallbackSubscription result_cb = ayanami::interfaces::NavigationCallbacks::on_navigation_result.subscribe(
    [&](const ayanami::actions::NavigationResult& r) {
      std::lock_guard<std::mutex> lock(result_mutex);
      statuses.push_back(r.status);
    });

  // Attempt 1: goal AT the robot's position -> the goal checker accepts on
  // the first control cycle -> the attempt reaches a terminal SUCCEEDED.
  host->setGoal(makePose(0.0, 0.0));
  ASSERT_TRUE(waitUntilPilotAttempt(&host->pilot()));
  ASSERT_TRUE(waitUntil([&] { return !host->pilot().attemptActive(); }, 10000))
    << "attempt 1 did not reach a terminal status";
  {
    std::lock_guard<std::mutex> lock(result_mutex);
    ASSERT_FALSE(statuses.empty());
    EXPECT_EQ(statuses.back(), ayanami::actions::GoalStatus::SUCCEEDED);
  }
  const void* tree_addr = &host->pilot().tree();
  const auto bb = host->pilot().blackboard();
  ASSERT_NE(bb, nullptr);
  uint64_t session_first = 0;
  ASSERT_TRUE(bb->get<uint64_t>(k_session_id, session_first));

  // Attempt 2 on the RESIDENT stack: same tree object, same blackboard
  // instance, advanced session, terminal SUCCEEDED again.
  host->setGoal(makePose(0.0, 0.0));
  ASSERT_TRUE(waitUntil(
    [&] {
      uint64_t sid = 0;
      return bb->get<uint64_t>(k_session_id, sid) && sid > session_first;
    },
    2000));
  EXPECT_EQ(&host->pilot().tree(), tree_addr) << "the second attempt must reuse the resident tree";
  EXPECT_EQ(host->pilot().blackboard(), bb) << "the second attempt must reuse the resident blackboard";
  ASSERT_TRUE(waitUntil([&] { return !host->pilot().attemptActive(); }, 10000))
    << "attempt 2 did not reach a terminal status";
  {
    std::lock_guard<std::mutex> lock(result_mutex);
    ASSERT_GE(statuses.size(), 2u);
    EXPECT_EQ(statuses[1], ayanami::actions::GoalStatus::SUCCEEDED);
  }

  host->stop();
  delete host;
  result_cb.reset();
  detachObservers();
}

// ---------------- Destruction order ----------------

TEST(CostmapNavigationTest, destructorDoesNotCrash) {
  const auto dir = makeConfigDir();
  {
    CostmapNavigation* host = createHost(dir);
    ASSERT_NE(host, nullptr);
    host->start();
    host->setGoal(makePose(2.0, 0.0));
    // End of scope -> delete: stop joins the nav thread -> pilot stop -> server stop
    // (clearActions runs before clearModules, so BT nodes no longer hold stale module pointers)
    delete host;
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_module_so = argv[1];
  if (argc > 2) g_astar_so = argv[2];
  if (argc > 3) g_minsnap_so = argv[3];
  if (argc > 4) g_pid_so = argv[4];
  if (argc > 5) g_test_controller_so = argv[5];
  if (argc > 6) g_spin_so = argv[6];
  if (argc > 7) g_goal_checker_so = argv[7];
  if (argc > 8) g_progress_checker_so = argv[8];
  return RUN_ALL_TESTS();
}
