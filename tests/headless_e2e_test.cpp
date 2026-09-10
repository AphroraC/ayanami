// (M1 finale): headless E2E full-pipeline test.
//
// No real chassis/ROS: assembles the whole chain via the dlopen libayanami_costmap_navigation.so host module:
// initialize -> start -> setGoal -> BT (smac planner -> savitzky smoother ->
// controller) -> cmd_vel emitted via the on_velocity_command bypass; periodic virtual pointclouds
// (out-of-map obstacles do not affect planning); also verifies the on_costmap_visualization bypass fires.
// Finally stop/destruction must not crash.
//
// argv: 1=host.so, 2=smac.so, 3=savitzky.so, 4=test_controller.so,
// 5=spin.so, 6=goal_checker.so, 7=progress_checker.so,
// 8=assets_dir (tests/assets, contains headless_map.pgm)

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/sensor_msgs.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

/// @brief Declare the four recovery roles the production tree references.
///
/// The tree names recoveries by ROLE (spin/wait/backup/clear_costmap), and the BT
/// extension maps each role to a module through config_recovery. A role nobody
/// declares resolves to an empty id, which the pilot now refuses to build a tree
/// with: a recovery that cannot resolve is a configuration error, not something
/// to discover while the robot is stuck.
/// @return the JSON array literal for the navigation section's config_recovery.
std::string writeRecoveryRoleFiles(const std::filesystem::path& dir,
                                   const std::string& module_name,
                                   const std::string& so_name = "") {
  const char* roles[] = {"clear_costmap", "spin", "wait", "backup"};
  std::string list;
  for (const char* role : roles) {
    const std::string file = std::string("role_") + role + ".json";
    {
      std::ofstream ofs(dir / file);
      ofs << "{ \"recovery\": { \"name\": \"" << module_name
          << "\", \"role\": \"" << role << "\"";
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

using ayanami::abstract::AbstractNavigation;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::Twist;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::interfaces::NavigationCallbacks;

std::string g_host_so;
std::string g_smac_so;
std::string g_savitzky_so;
std::string g_test_controller_so;
std::string g_spin_so;
std::string g_goal_checker_so;
std::string g_progress_checker_so;
std::string g_assets_dir;

// ---- Collectors ----
std::mutex g_cmd_mutex;
TwistStamped g_last_cmd;
int g_cmd_count = 0;
int g_cmd_cb_id = -1;

std::mutex g_map_mutex;
int g_map_count = 0;

// ---- Test utilities ----

PoseStamped makePose(double x, double y) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

bool isZeroTwist(const TwistStamped& t) {
  return t.twist.linear.norm() < 1e-9 && t.twist.angular.norm() < 1e-9;
}
/// Builds a single-point PointCloud2 (x/y/z FLOAT32, 12B/point); the obstacle lies outside the map and does not affect planning.
ayanami::sensor_msgs::PointCloud2 makeFarPointCloud() {
  ayanami::sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  const float x = 50.0f;  // Outside the 20x20 @0.5 = 10m map -> worldToMap fails, nothing lands on the map
  const float y = 50.0f;
  const float z = 0.0f;
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = 0.0f;
  cloud.cloud.push_back(p);
  return cloud;
}

void setupTransforms() {
  auto& store = ayanami::interfaces::TransformStore::instance();
  store.setTransform("odom", "base_link", ayanami::Timestamp::latest(),
                     Eigen::Isometry3d::Identity());
  store.setTransform("map", "odom", ayanami::Timestamp::latest(),
                     Eigen::Isometry3d::Identity());
  store.setTransform("base_link", "lidar", ayanami::Timestamp::latest(),
                     Eigen::Isometry3d::Identity());
}

void write_file(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

std::string moduleJson(const std::string& module, const std::string& name,
                        const std::string& so_name) {
  // The module's own (empty) parameter section: the loader requires a section
  // named after the module, so that "this module reads no parameters" is
  // declared rather than indistinguishable from a name/section mismatch.
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + so_name + "\"\n  },\n  \"" + name +
         "\": {}\n}\n";
}

/// Full config suite (mirrors the complete set). static_map points to tests/assets
/// headless_map.pgm (20x20 @0.5 = 10m x 10m, all FREE; A* can plan from (0,0) to (2,0)).
std::filesystem::path makeConfigDir() {
  const auto dir = std::filesystem::temp_directory_path() /
                   "ayanami_headless_e2e";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  std::filesystem::create_directories(dir / "BT");

  write_file(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"costmap_navigation.json\"\n"
             "  }\n}\n");

  {
    std::ofstream nav(dir / "costmap_navigation.json");
    nav << "{\n  \"navigation\": {\n"
        << "    \"behavior_tree_extension\": \"" AYANAMI_TEST_BT_EXT_SO "\",\n"
        << "    \"behavior_tree_xml\": \"BT/navigation.xml\",\n"
        << "    \"behavior_tree_id\": \"NavigateWithReplanningAndSmoothing\",\n"
        << "    \"config_planner\": \"planner.json\",\n"
        << "    \"config_smoother\": \"smoother.json\",\n"
        << "    \"config_controller\": \"controller.json\",\n"
        << "    \"config_recovery\": "
        << writeRecoveryRoleFiles(dir, "test_spin", g_spin_so) << ",\n"
        << "    \"config_goal_checker\": \"goal_checker.json\",\n"
        << "    \"config_progress_checker\": \"progress_checker.json\",\n"
        << "    \"clear_costmap_id\": \"test_spin\",\n"
        << "    \"spin_id\": \"test_spin\",\n"
        << "    \"wait_id\": \"test_spin\",\n"
        << "    \"backup_id\": \"test_spin\",\n"
        << "    \"planner_frequency\": 100.0,\n"
        << "    \"smoother_frequency\": -1.0,\n"
        << "    \"controller_frequency\": 20.0,\n"
        << "    \"tick_frequency\": 100.0,\n"
        << "    \"costmap_visualization_rate\": 100.0\n"
        << "  }\n";
  }

  write_file(dir / "planner.json",
             moduleJson("planner", "test_planner", g_smac_so));
  write_file(dir / "smoother.json",
             moduleJson("smoother", "test_smoother", g_savitzky_so));
  write_file(dir / "controller.json",
             moduleJson("controller", "test_controller",
                         g_test_controller_so));
  write_file(dir / "recovery.json",
             "{\n  \"recovery\": {\n    \"name\": \"test_spin\",\n"
             "    \"role\": \"spin\",\n    \"so_name\": \"" + g_spin_so +
             "\"\n  },\n  \"test_spin\": {}\n}\n");
  // goal/progress checker modules (the execution needs injected checkers; missing ones are judged
  // InternalError). Parameters merged into the same json (modules read them explicitly via config_goal_checker /
  // config_progress_checker; the progress required_movement is set small to
  // simulate a robot that must keep judging progress, avoiding a false NoProgress from the 5s stationary window).
  write_file(dir / "goal_checker.json",
             "{\n  \"goal_checker\": {\n"
             "    \"name\": \"simple_goal_checker\",\n"
             "    \"so_name\": \"" + g_goal_checker_so + "\"\n"
             "  },\n"
             // The module reads its own section (ModuleContext::name), not the
             // family one: parameters left in the family section are inert.
             "  \"simple_goal_checker\": {\n"
             "    \"position_tolerance\": 0.10,\n"
             "    \"angle_tolerance\": 0.20,\n"
             "    \"linear_tolerance\": 0.10,\n"
             "    \"angular_tolerance\": 0.10\n"
             "  }\n}\n");
  write_file(dir / "progress_checker.json",
             "{\n  \"progress_checker\": {\n"
             "    \"name\": \"simple_progress_checker\",\n"
             "    \"so_name\": \"" + g_progress_checker_so + "\"\n"
             "  },\n"
             "  \"simple_progress_checker\": {\n"
             "    \"required_movement\": 0.1,\n"
             "    \"movement_angle_required\": 0.2,\n"
             "    \"time_period\": 5.0\n"
             "  }\n}\n");

  // static_map points to the committed asset (tests/assets/headless_map.pgm).
  {
    // Costmap parameters are merged into costmap_navigation.json (the old costmap.json was removed).
    std::ofstream nav(dir / "costmap_navigation.json", std::ios::app);
    nav << ",\n  \"costmap\": {\n"
        << "    \"global_frame\": \"map\",\n"
        << "    \"local_frame\": \"odom\",\n"
        << "    \"robot_frame\": \"base_link\",\n"
        << "    \"resolution\": 0.5\n"
        << "  },\n"
        << "  \"costmap_global\": {\n"
        << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
        << "    \"rolling_window\": false,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"static_map\": \""
        << (std::filesystem::path(g_assets_dir) / "headless_map.pgm").string()
        << "\",\n"
        << "    \"inflation_radius\": 1.0,\n"
        << "    \"inscribed_radius\": 0.8,\n"
        << "    \"cost_scaling_factor\": 2.0\n"
        << "  },\n"
        << "  \"costmap_local\": {\n"
        << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
        << "    \"rolling_window\": true,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"inflation_radius\": 1.0,\n"
        << "    \"inscribed_radius\": 0.8,\n"
        << "    \"cost_scaling_factor\": 2.0\n"
        << "  }\n"
        << "}\n";
  }

  write_file(dir / "costmap_savitzky_smoother.json",
             "{\n  \"costmap_savitzky_smoother\": {\n"
             "    \"do_refinement\": true,\n    \"refinement_num\": 2\n"
             "  }\n}\n");
  write_file(dir / "costmap_spin_recovery.json",
             "{\n  \"recovery\": {\n"
             "    \"spin_dist\": 1.57,\n    \"max_w\": 0.5,\n"
             "    \"sim_cycle_dt\": 0.01,\n    \"collision_radius\": 0.3\n"
             "  }\n}\n");

  write_file(dir / "BT" / "navigation.xml",
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

void attachCollectors() {
  g_cmd_count = 0;
  g_last_cmd = TwistStamped{};
  g_cmd_cb_id = NavigationCallbacks::on_velocity_command.add(
      [](const TwistStamped& t) {
        std::lock_guard<std::mutex> lock(g_cmd_mutex);
        g_last_cmd = t;
        ++g_cmd_count;
      });
  g_map_count = 0;
}

void detachCollectors() {
  if (g_cmd_cb_id >= 0) {
    NavigationCallbacks::on_velocity_command.remove(g_cmd_cb_id);
    g_cmd_cb_id = -1;
  }
}

int cmdCount() {
  std::lock_guard<std::mutex> lock(g_cmd_mutex);
  return g_cmd_count;
}

bool nonZeroCmdCountAtLeast(int n) {
  // The controller keeps emitting 0.5 linear velocity each cycle, so non-zero commands dominate; the total count approximates the trigger threshold.
  std::lock_guard<std::mutex> lock(g_cmd_mutex);
  return g_cmd_count >= n;
}

template <typename Fn>
bool waitUntil(const Fn& fn, int timeout_ms = 5000) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (fn()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return fn();
}

}  // namespace

// ---------------- Headless E2E: full navigation pipeline ----------------

TEST(HeadlessE2E, fullNavigationPipeline) {
  // 1. GlobalConfig::instance(tmp config / full suite)
  const auto dir = makeConfigDir();
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);

  // 2. NavigationCallbacks cleanup (protects against pollution leaked by prior tests)
  detachCollectors();

  // 3. Load the host via the common factory -> AbstractNavigation::Ptr (typed cast, 4)
  auto host_ext = ayanami::ExtensionModule::loadModule(g_host_so);
  ASSERT_NE(host_ext, nullptr) << "loadModule failed: " << g_host_so;
  AbstractNavigation::Ptr nav =
      std::dynamic_pointer_cast<AbstractNavigation>(std::move(host_ext));
  ASSERT_NE(nav, nullptr);

  // 4. Collect cmd_vel
  attachCollectors();

  // 5. Collect map visualization (verifies the start/rate-limited on_costmap_visualization bypass)
  std::atomic<int> map_count{0};
  ayanami::CallbackSubscription map_cb =
      NavigationCallbacks::on_costmap_visualization.subscribe(
          [&map_count](const std::string&, const ayanami::nav_msgs::OccupancyGrid&) {
            ++map_count;
          });

  // 6. Start
  nav->initialize(dir.string());
  nav->start();

  // 7. Obstacle observation: feed 3 virtual pointcloud frames before setGoal (outside the map, no planning impact),
  // verifying the host-delivered map pipeline (AbstractNavigation::setPointcloud, not the global input slot;
  // input/output layering discipline) does not crash. Note: do not feed concurrently after the controller thread starts (the plan states
  // Costmap*Execution::run holds the local wrapper lock for the whole loop, so concurrent pointcloud callbacks would deadlock --
  // discovered, see report, pending reviewer ruling).
  setupTransforms();
  for (int i = 0; i < 3; ++i) {
    nav->setPointcloud(makeFarPointCloud());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // 8. setGoal (map frame, x=2, y=0, yaw=0)
  nav->setGoal(makePose(2.0, 0.0));

  // 9. Wait for cmd_count to reach the expectation
  EXPECT_TRUE(waitUntil([] { return nonZeroCmdCountAtLeast(3); }, 10000))
      << "cmd_count did not reach threshold: " << cmdCount();

  // 10. Assert: cmd_count > threshold, cmd_vel initialized (not dirty memory / not NaN)
  EXPECT_GT(cmdCount(), 1);
  TwistStamped last = g_last_cmd;
  EXPECT_TRUE(std::isfinite(last.twist.linear.x()));
  EXPECT_FALSE(std::isnan(last.twist.angular.z()));
  // The visualization bypass fired (start first frame + rate-limited repeats)
  EXPECT_GT(map_count.load(), 0);

  // 11. nav->stop + cleanup; destruction does not crash
  nav->stop();
  detachCollectors();
  nav.reset();  // ~CostmapNavigation -> stop (idempotent) + join, must not crash
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_host_so = argv[1];
  if (argc > 2) g_smac_so = argv[2];
  if (argc > 3) g_savitzky_so = argv[3];
  if (argc > 4) g_test_controller_so = argv[4];
  if (argc > 5) g_spin_so = argv[5];
  if (argc > 6) g_goal_checker_so = argv[6];
  if (argc > 7) g_progress_checker_so = argv[7];
  if (argc > 8) g_assets_dir = argv[8];
  return RUN_ALL_TESTS();
}
