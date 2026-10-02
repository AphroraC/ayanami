// FSM-host headless E2E: assembles the whole chain via dlopen of
// libayanami_rose_navigation_fsm.so exactly as production loads it --
// costmap machinery + derived ESDF writer + FsmDriver (astar planner ->
// esdfmap minco smoother -> abstract pid controller -> nothing recovery).
//
// argv: 1=fsm_host.so, 2=astar.so, 3=esdfmap_minco.so, 4=abstract_pid.so,
//       5=nothing_recovery.so, 6=goal_checker.so, 7=progress_checker.so,
//       8=assets_dir (tests/assets, contains headless_map.pgm)

#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/sensor_msgs.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

using ayanami::testing::testTransforms;

namespace {

using ayanami::abstract::AbstractNavigation;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::interfaces::NavigationCallbacks;

std::string g_host_so;
std::string g_planner_so;
std::string g_smoother_so;
std::string g_controller_so;
std::string g_recovery_so;
std::string g_goal_checker_so;
std::string g_progress_checker_so;
std::string g_assets_dir;

void write_file(const std::filesystem::path& path, const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

std::string moduleJson(const std::string& family, const std::string& name,
                       const std::string& so_name) {
  // The loader requires a section named after the module: "reads no
  // parameters" is declared rather than looking like a name/section
  // mismatch (parameters come from the module's own section defaults).
  return "{\n  \"" + family + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + so_name +
         "\"\n  },\n  \"" + name + "\": {}\n}\n";
}

/// Full config suite: an FSM-era route (no BT keys) + inline-layer costmap
/// blocks + the esdfmap section. The headless map is 20x20 @0.5 (all FREE).
std::filesystem::path makeConfigDir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_fsm_e2e";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  write_file(
      dir / "config.json",
      "{\n  \"global\": {\n"
      "    \"config_navigation\": \"fsm_navigation.json\"\n"
      "  }\n}\n");

  const std::string static_map =
      (std::filesystem::path(g_assets_dir) / "headless_map.pgm").string();
  std::ofstream nav(dir / "fsm_navigation.json");
  nav << "{\n  \"navigation\": {\n"
      << "    \"config_planner\": \"planner.json\",\n"
      << "    \"config_smoother\": \"smoother.json\",\n"
      << "    \"config_controller\": \"controller.json\",\n"
      << "    \"config_recovery\": [\"recovery.json\"],\n"
      << "    \"config_goal_checker\": \"goal_checker.json\",\n"
      << "    \"config_progress_checker\": \"progress_checker.json\",\n"
      << "    \"fsm_frequency\": 50.0,\n"
      << "    \"planner_frequency\": -1.0,\n"
      << "    \"smoother_frequency\": -1.0,\n"
      << "    \"smoother_timeout\": 2.0,\n"
      << "    \"controller_frequency\": 20.0,\n"
      << "    \"max_nav_retries\": 6,\n"
      // A stationary test robot must not trip the gates mid-scenario.
      << "    \"oscillation_distance\": 0.1,\n"
      << "    \"oscillation_timeout\": 3600.0,\n"
      << "    \"controller_patience\": -1.0,\n"
      << "    \"recovery_patience\": 0.0,\n"
      << "    \"max_vel_x\": 2.5,\n    \"max_vel_y\": 2.5,\n"
      << "    \"max_vel_theta\": 3.0,\n"
      << "    \"robot_frame\": \"base_link\",\n"
      << "    \"velocity_timeout\": 0.25\n"
      << "  },\n"
      << "  \"esdfmap\": {\n"
      << "    \"max_distance\": 3.0,\n"
      << "    \"update_rate\": 50.0,\n"
      << "    \"stale_timeout\": 0.0\n"
      << "  },\n"
      << "  \"costmap\": {\n"
      << "    \"global_frame\": \"map\",\n"
      << "    \"local_frame\": \"odom\",\n"
      << "    \"robot_frame\": \"base_link\",\n"
      << "    \"resolution\": 0.5\n"
      << "  },\n"
      << "  \"costmap_global\": {\n"
      << "    \"rolling_window\": false,\n    \"track_unknown\": false,\n"
      << "    \"static_layer\": {\n"
      << "      \"static_map\": \"" << static_map << "\",\n"
      << "      \"size_x\": 20, \"size_y\": 20, \"resolution\": 0.5\n"
      << "    },\n"
      << "    \"inflation_layer\": {\n"
      << "      \"inflation_radius\": 1.0, \"inscribed_radius\": 0.8, "
         "\"cost_scaling_factor\": 2.0\n"
      << "    }\n"
      << "  },\n"
      << "  \"costmap_local\": {\n"
      << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
      << "    \"rolling_window\": true,\n    \"track_unknown\": false,\n"
      << "    \"obstacle_layer\": {\n"
      << "      \"combination\": \"overwrite\", "
         "\"footprint_clearing\": true,\n"
      << "      \"observation_sources\": [\"cloud\"],\n"
      << "      \"cloud\": {\"type\": \"pointcloud\"}\n"
      << "    },\n"
      << "    \"inflation_layer\": {\n"
      << "      \"inflation_radius\": 1.0, \"inscribed_radius\": 0.8, "
         "\"cost_scaling_factor\": 2.0\n"
      << "    }\n"
      << "  }\n}\n";

  write_file(dir / "planner.json",
             moduleJson("planner", "costmap_astar_planner", g_planner_so));
  write_file(dir / "smoother.json",
             moduleJson("smoother", "esdfmap_minco_smoother", g_smoother_so));
  write_file(dir / "controller.json",
             moduleJson("controller", "abstract_pid_controller",
                        g_controller_so));
  write_file(dir / "recovery.json",
             moduleJson("recovery", "abstract_nothing_recovery",
                        g_recovery_so));
  write_file(
      dir / "goal_checker.json",
      "{\n  \"goal_checker\": {\n"
      "    \"name\": \"simple_goal_checker\",\n    \"so_name\": \"" +
          g_goal_checker_so +
          "\"\n  },\n  \"simple_goal_checker\": {\n"
          "    \"position_tolerance\": 0.10,\n    \"angle_tolerance\": 0.20\n"
          "  }\n}\n");
  write_file(
      dir / "progress_checker.json",
      "{\n  \"progress_checker\": {\n"
      "    \"name\": \"simple_progress_checker\",\n    \"so_name\": \"" +
          g_progress_checker_so +
          "\"\n  },\n  \"simple_progress_checker\": {\n"
          "    \"required_movement\": 0.1,\n    \"time_period\": 3600.0\n"
          "  }\n}\n");
  return dir;
}

PoseStamped makePose(double x, double y) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

/// A single far-away point (outside the map): exercises the observation
/// pipeline without affecting planning.
ayanami::sensor_msgs::PointCloud2 makeFarPointCloud() {
  ayanami::sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.header.stamp = ayanami::Timestamp::latest();
  pcl::PointXYZI p;
  p.x = 50.0f;  // outside the 20x20 @0.5 = 10m map
  p.y = 50.0f;
  p.z = 0.0f;
  p.intensity = 0.0f;
  cloud.cloud.push_back(p);
  return cloud;
}

void setupTransforms() {
  testTransforms().set("odom", "base_link", ayanami::Timestamp::latest(),
                       Eigen::Isometry3d::Identity());
  testTransforms().set("map", "odom", ayanami::Timestamp::latest(),
                       Eigen::Isometry3d::Identity());
}

template <typename Fn>
bool waitUntil(const Fn& fn, int timeout_ms = 10000) {
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

// ---- scenario 1: the follow pipeline emits velocity commands ----

TEST(FsmHostE2E, followPipelineEmitsCommands) {
  const auto dir = makeConfigDir();
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);

  auto host_ext = ayanami::ExtensionModule::loadModule(g_host_so);
  ASSERT_NE(host_ext, nullptr) << "loadModule failed: " << g_host_so;
  AbstractNavigation::Ptr nav =
      std::dynamic_pointer_cast<AbstractNavigation>(std::move(host_ext));
  ASSERT_NE(nav, nullptr);

  std::mutex cmd_mutex;
  TwistStamped last_cmd;
  int cmd_count = 0;
  ayanami::CallbackSubscription cmd_cb =
      NavigationCallbacks::on_velocity_command.subscribe(
          [&](const TwistStamped& t) {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            last_cmd = t;
            ++cmd_count;
          });

  setupTransforms();
  nav->initialize(dir.string());
  nav->start();

  for (int i = 0; i < 3; ++i) {
    nav->setPointcloud(makeFarPointCloud());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  nav->setGoal(makePose(2.0, 0.0));
  EXPECT_TRUE(waitUntil(
      [&] {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        return cmd_count >= 3;
      }))
      << "cmd_count did not reach threshold";
  {
    std::lock_guard<std::mutex> lock(cmd_mutex);
    EXPECT_TRUE(std::isfinite(last_cmd.twist.linear.x()));
    EXPECT_FALSE(std::isnan(last_cmd.twist.angular.z()));
  }

  nav->stop();
  cmd_cb.reset();
  nav.reset();  // idempotent stop + join; must not crash
}

// ---- scenario 2: goal at the robot position reaches SUCCEEDED twice ----

TEST(FsmHostE2E, consecutiveAttemptsBothSucceed) {
  const auto dir = makeConfigDir();
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);

  auto host_ext = ayanami::ExtensionModule::loadModule(g_host_so);
  ASSERT_NE(host_ext, nullptr) << "loadModule failed: " << g_host_so;
  AbstractNavigation::Ptr nav =
      std::dynamic_pointer_cast<AbstractNavigation>(std::move(host_ext));
  ASSERT_NE(nav, nullptr);

  std::mutex result_mutex;
  std::vector<ayanami::actions::GoalStatus> statuses;
  ayanami::CallbackSubscription result_cb =
      NavigationCallbacks::on_navigation_result.subscribe(
          [&](const ayanami::actions::NavigationResult& r) {
            std::lock_guard<std::mutex> lock(result_mutex);
            statuses.push_back(r.status);
          });

  setupTransforms();
  nav->initialize(dir.string());
  nav->start();

  for (int i = 0; i < 3; ++i) {
    nav->setPointcloud(makeFarPointCloud());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // Attempt 1: goal at the robot's position -> the goal checker accepts on
  // the first control cycle -> SUCCEEDED.
  nav->setGoal(makePose(0.0, 0.0));
  ASSERT_TRUE(waitUntil(
      [&] {
        std::lock_guard<std::mutex> lock(result_mutex);
        return !statuses.empty();
      }))
      << "attempt 1 never reached a terminal status";
  {
    std::lock_guard<std::mutex> lock(result_mutex);
    ASSERT_FALSE(statuses.empty());
    EXPECT_EQ(statuses.back(), ayanami::actions::GoalStatus::SUCCEEDED);
  }

  // Attempt 2: the same flow again on the resident FSM stack.
  nav->setGoal(makePose(0.0, 0.0));
  ASSERT_TRUE(waitUntil(
      [&] {
        std::lock_guard<std::mutex> lock(result_mutex);
        return statuses.size() >= 2;
      }))
      << "attempt 2 never reached a terminal status";
  {
    std::lock_guard<std::mutex> lock(result_mutex);
    ASSERT_GE(statuses.size(), 2u);
    EXPECT_EQ(statuses[1], ayanami::actions::GoalStatus::SUCCEEDED);
  }

  nav->stop();
  result_cb.reset();
  nav.reset();
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_host_so = argv[1];
  if (argc > 2) g_planner_so = argv[2];
  if (argc > 3) g_smoother_so = argv[3];
  if (argc > 4) g_controller_so = argv[4];
  if (argc > 5) g_recovery_so = argv[5];
  if (argc > 6) g_goal_checker_so = argv[6];
  if (argc > 7) g_progress_checker_so = argv[7];
  if (argc > 8) g_assets_dir = argv[8];
  return RUN_ALL_TESTS();
}
