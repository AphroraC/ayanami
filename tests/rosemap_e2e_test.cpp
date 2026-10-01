// rosemap-family headless E2E: dlopen's the FULL production rosemap stack
// (libayanami_rosemap_navigation.so host + the real astar planner / minco
// smoother / MPC controller / escape+wait recoveries / the shared BT
// extension) against the shipped rosemap route layout, then drives a
// CLOSED-LOOP virtual chassis: the on_velocity_command bypass integrates
// the robot pose (thread-safe TF source), so the navigation must actually
// REACH the goal for the test to pass (terminal SUCCEEDED through
// on_navigation_result).
//
// argv: 1=host.so, 2=planner.so, 3=smoother.so, 4=controller.so,
//       5=escape.so, 6=wait.so, 7=goal_checker.so, 8=progress_checker.so

#include <gtest/gtest.h>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/timestamp.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

namespace {

using ayanami::abstract::AbstractNavigation;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::interfaces::NavigationCallbacks;

std::string g_host_so;
std::string g_planner_so;
std::string g_smoother_so;
std::string g_controller_so;
std::string g_escape_so;
std::string g_wait_so;
std::string g_goal_checker_so;
std::string g_progress_checker_so;
std::string g_bt_so_path;  // the shared BT extension .so

// ---- thread-safe transform source (the virtual chassis mutates it while
// the navigation threads look up) ----
class LockedTransformSource {
 public:
  void set(const std::string& parent, const std::string& child,
           const Eigen::Isometry3d& transform) {
    std::lock_guard<std::mutex> lock(mutex_);
    edges_[{parent, child}] = transform;
    edges_[{child, parent}] = transform.inverse();
  }

  bool lookup(const std::string& target, const std::string& source,
              Eigen::Isometry3d& transform) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (target == source) {
      transform = Eigen::Isometry3d::Identity();
      return true;
    }
    std::map<std::string, Eigen::Isometry3d> reached{
        {target, Eigen::Isometry3d::Identity()}};
    std::vector<std::string> frontier{target};
    while (!frontier.empty()) {
      const std::string current = frontier.back();
      frontier.pop_back();
      for (const auto& [endpoints, edge] : edges_) {
        if (endpoints.first != current) {
          continue;
        }
        const std::string& next = endpoints.second;
        if (reached.count(next) != 0) {
          continue;
        }
        reached[next] = reached[current] * edge;
        if (next == source) {
          transform = reached[next];
          return true;
        }
        frontier.push_back(next);
      }
    }
    return false;
  }

 private:
  mutable std::mutex mutex_;
  std::map<std::pair<std::string, std::string>, Eigen::Isometry3d> edges_;
};

LockedTransformSource g_transforms;

// ---- virtual chassis state ----
std::mutex g_robot_mutex;
double g_robot_x{0.0};
double g_robot_y{0.0};
double g_robot_yaw{0.0};
std::atomic<bool> g_chassis_running{false};

TwistStamped g_last_cmd;
std::mutex g_cmd_mutex;
std::atomic<int> g_cmd_count{0};
std::atomic<int> g_field_count{0};
std::mutex g_result_mutex;
std::vector<ayanami::actions::NavigationResult> g_results;
std::vector<std::string> g_exe_results;  // controller task outcomes (diag)
int g_exe_cb_id{-1};
int g_cmd_cb_id{-1};
ayanami::CallbackSubscription g_field_cb;
int g_result_cb_id{-1};

/// @brief Push the robot pose into the TF source (g_robot_mutex HELD).
void publishRobotStateLocked() {
  Eigen::Isometry3d map_from_base = Eigen::Isometry3d::Identity();
  map_from_base.translation().x() = g_robot_x;
  map_from_base.translation().y() = g_robot_y;
  map_from_base.linear() =
      Eigen::Isometry3d(
          Eigen::AngleAxisd(g_robot_yaw, Eigen::Vector3d::UnitZ()))
          .linear();
  g_transforms.set("map", "base_link", map_from_base);
}

void chassisLoop() {
  const std::chrono::duration<double> dt{0.02};
  while (g_chassis_running.load()) {
    TwistStamped cmd;
    {
      std::lock_guard<std::mutex> lock(g_cmd_mutex);
      cmd = g_last_cmd;
    }
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    // Integrate the BODY-frame command in the world frame.
    const double cos_yaw = std::cos(g_robot_yaw);
    const double sin_yaw = std::sin(g_robot_yaw);
    g_robot_x += (cos_yaw * cmd.twist.linear.x() -
                  sin_yaw * cmd.twist.linear.y()) * dt.count();
    g_robot_y += (sin_yaw * cmd.twist.linear.x() +
                  cos_yaw * cmd.twist.linear.y()) * dt.count();
    g_robot_yaw += cmd.twist.angular.z() * dt.count();
    publishRobotStateLocked();
    std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::milliseconds>(dt));
  }
}

PoseStamped makePose(double x, double y) {
  auto pose = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  pose.pose.position = Eigen::Vector3d(x, y, 0.0);
  return pose;
}

template <typename Fn>
bool waitUntil(const Fn& fn, int timeout_ms) {
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

void write_file(const std::filesystem::path& path, const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

std::string moduleJson(const std::string& family, const std::string& name,
                       const std::string& so_name) {
  return "{\n  \"" + family + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + so_name +
         "\"\n  },\n  \"" + name + "\": {}\n}\n";
}

/// @brief The full rosemap route (the shipped config layout, paths pointing
/// at the dlopen'd test module .so files).
std::filesystem::path makeConfigDir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_rosemap_e2e";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  std::filesystem::create_directories(dir / "BT");

  write_file(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"rosemap_navigation.json\"\n"
             "  }\n}\n");

  write_file(
      dir / "rosemap_navigation.json",
      "{\n  \"navigation\": {\n"
      "    \"behavior_tree_extension\": \"" + g_bt_so_path + "\",\n"
      "    \"behavior_tree_id\": \"NavigateWithReplanningAndSmoothing\",\n"
      "    \"behavior_tree_xml\": \"BT/rosemap_navigation.xml\",\n"
      "    \"recovery_roles\": [\"escape\", \"wait\"],\n"
      "    \"config_planner\": \"planner.json\",\n"
      "    \"config_smoother\": \"smoother.json\",\n"
      "    \"config_controller\": \"controller.json\",\n"
      "    \"config_goal_checker\": \"goal_checker.json\",\n"
      "    \"config_progress_checker\": \"progress_checker.json\",\n"
      "    \"config_recovery\": [\"escape.json\", \"wait.json\"],\n"
      "    \"planner_frequency\": 5.0,\n"
      "    \"smoother_frequency\": 10.0,\n"
      "    \"controller_frequency\": 30.0,\n"
      "    \"tick_frequency\": 50.0,\n"
      "    \"max_nav_retries\": 4,\n"
      "    \"oscillation_distance\": 0.05,\n"
      "    \"oscillation_timeout\": 30.0,\n"
      "    \"controller_patience\": -1.0,\n"
      "    \"recovery_patience\": 0.0,\n"
      "    \"max_vel_x\": 2.5,\n"
      "    \"max_vel_y\": 2.5,\n"
      "    \"max_vel_theta\": 3.0,\n"
      "    \"robot_frame\": \"base_link\",\n"
      "    \"velocity_timeout\": 0.25\n"
      "  },\n"
      "  \"rosemap_field\": {\n"
      "    \"field_frame\": \"map\",\n"
      "    \"resolution\": 0.1,\n"
      "    \"size_x\": 16.0,\n"
      "    \"size_y\": 16.0,\n"
      "    \"update_frequency\": 10.0,\n"
      "    \"max_distance\": 3.0,\n"
      "    \"stale_timeout\": 0.0,\n"
      "    \"min_obstacle_z\": -1.0,\n"
      "    \"max_obstacle_z\": 2.0,\n"
      "    \"obstacle_min_range\": 0.1,\n"
      "    \"obstacle_max_range\": 30.0,\n"
      "    \"raytrace_max_range\": 30.0\n"
      "  }\n}\n");

  write_file(dir / "planner.json",
             moduleJson("planner", "rosemap_astar_planner", g_planner_so));
  write_file(dir / "smoother.json",
             moduleJson("smoother", "rosemap_minco_smoother", g_smoother_so));
  write_file(dir / "controller.json",
             moduleJson("controller", "rosemap_mpc_controller",
                        g_controller_so));
  write_file(dir / "escape.json",
             "{\n  \"recovery\": {\n    \"name\": \"rosemap_escape_recovery\",\n"
             "    \"role\": \"escape\",\n    \"so_name\": \"" + g_escape_so +
             "\",\n    \"sim_cycle_dt\": 0.02\n  },\n"
             "  \"rosemap_escape_recovery\": {\n"
             "    \"escape_speed\": 0.4,\n    \"exit_distance\": 0.4,\n"
             "    \"max_ticks\": 300\n  }\n}\n");
  write_file(dir / "wait.json",
             "{\n  \"recovery\": {\n    \"name\": \"rosemap_wait_recovery\",\n"
             "    \"role\": \"wait\",\n    \"so_name\": \"" + g_wait_so +
             "\"\n  },\n  \"rosemap_wait_recovery\": {\n"
             "    \"wait_duration\": 0.1\n  }\n}\n");
  write_file(dir / "goal_checker.json",
             "{\n  \"goal_checker\": {\n    \"name\": \"simple_goal_checker\",\n"
             "    \"so_name\": \"" + g_goal_checker_so +
             "\"\n  },\n  \"simple_goal_checker\": {\n"
             "    \"position_tolerance\": 0.2,\n"
             "    \"angle_tolerance\": 0.5\n  }\n}\n");
  write_file(dir / "progress_checker.json",
             "{\n  \"progress_checker\": {\n    \"name\": "
             "\"simple_progress_checker\",\n    \"so_name\": \"" +
                 g_progress_checker_so +
                 "\"\n  },\n  \"simple_progress_checker\": {\n"
                 "    \"required_movement\": 0.05,\n"
                 "    \"time_period\": 5.0\n  }\n}\n");

  // The shipped production tree (escape/wait ladder).
  write_file(dir / "BT" / "rosemap_navigation.xml", R"(<root BTCPP_format="4" main_tree_to_execute="NavigateWithReplanningAndSmoothing">
  <BehaviorTree ID="NavigateWithReplanningAndSmoothing">
    <RecoveryNode name="NavigateRecovery" number_of_retries="{nav_retries}" budget_reset_token="{goal_revision}" failure_context="{failure_context}">
      <ReactiveSequence name="ProgressGate">
        <IsRobotProgressing distance="{oscillation_distance}" window="{oscillation_timeout}"/>
        <PipelineSequence name="NavigateWithReplanning">
          <RateController hz="{planner_frequency}" reset_token="{goal_revision}">
            <RecoveryNode name="ComputePath" number_of_retries="1" failure_context="{failure_context}">
              <Sequence>
                <ComputePathToPose goal="{goal}" path="{raw_path}" failure_context="{failure_context}"/>
                <IsPathValid path="{raw_path}" failure_context="{failure_context}"/>
              </Sequence>
              <Sequence>
                <Recovery recovery="{escape_id}" failure_context="{failure_context}"/>
              </Sequence>
            </RecoveryNode>
          </RateController>
          <RateController hz="{smoother_frequency}" reset_token="{raw_path_revision}">
            <Sequence>
              <Fallback name="SmoothOrDegrade">
                <SmoothPath input_path="{smoothed_path}" raw_path="{raw_path}" path="{smoothed_path}" failure_context="{failure_context}"/>
                <DegradeToRawPath input_path="{raw_path}" path="{smoothed_path}" failure_context="{failure_context}"/>
              </Fallback>
              <IsPathValid path="{smoothed_path}" failure_context="{failure_context}"/>
            </Sequence>
          </RateController>
          <RecoveryNode name="FollowPath" number_of_retries="1" budget_reset_token="{goal_revision}" failure_context="{failure_context}">
            <FollowPath path="{smoothed_path}" frequency="{controller_frequency}" failure_context="{failure_context}"/>
            <Sequence>
              <Recovery recovery="{escape_id}" failure_context="{failure_context}"/>
            </Sequence>
          </RecoveryNode>
        </PipelineSequence>
      </ReactiveSequence>
      <ReactiveFallback name="RecoveryActions">
        <GoalUpdated goal="{goal}"/>
        <RecoverySequence name="RecoveryActions" reset_token="{goal_revision}">
          <Recovery recovery="{escape_id}" failure_context="{failure_context}"/>
          <Recovery recovery="{wait_id}" failure_context="{failure_context}"/>
        </RecoverySequence>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>)");

  return dir;
}

}  // namespace

TEST(RosemapE2E, ClosedLoopNavigationReachesTheGoal) {
  const auto dir = makeConfigDir();
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);

  // Collectors.
  g_cmd_count = 0;
  g_field_count = 0;
  g_results.clear();
  g_cmd_cb_id = NavigationCallbacks::on_velocity_command.add(
      [](const TwistStamped& t) {
        std::lock_guard<std::mutex> lock(g_cmd_mutex);
        g_last_cmd = t;
        ++g_cmd_count;
      });
  g_field_cb = NavigationCallbacks::on_rosemap_field.subscribe(
      [](const ayanami::nav_msgs::OccupancyGrid&) { ++g_field_count; });
  g_result_cb_id = NavigationCallbacks::on_navigation_result.add(
      [](const ayanami::actions::NavigationResult& r) {
        std::lock_guard<std::mutex> lock(g_result_mutex);
        g_results.push_back(r);
      });
  g_exe_cb_id = NavigationCallbacks::on_exe_path_result.add(
      [](const ayanami::actions::GoalResult<ayanami::actions::ExePathResult>& r) {
        std::lock_guard<std::mutex> lock(g_result_mutex);
        g_exe_results.push_back(
            "status=" + std::to_string(static_cast<int>(r.status)) +
            " error=" + std::to_string(static_cast<int>(r.result.error)) +
            " " + r.result.message);
      });

  // Install the thread-safe TF source (the chassis thread mutates edges
  // while the navigation threads look up).
  ayanami::components::RobotStateController::instance().setTransformSource(
      [](const std::string& target, const std::string& source,
         const ayanami::Timestamp&, Eigen::Isometry3d& transform, double) {
        return g_transforms.lookup(target, source, transform);
      });

  // Initial robot state at the origin, facing +x.
  {
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    g_robot_x = 0.0;
    g_robot_y = 0.0;
    g_robot_yaw = 0.0;
    publishRobotStateLocked();
  }
  g_transforms.set("odom", "base_link", Eigen::Isometry3d::Identity());
  g_transforms.set("map", "odom", Eigen::Isometry3d::Identity());

  // Load the production host module.
  auto host_ext = ayanami::ExtensionModule::loadModule(g_host_so);
  ASSERT_NE(host_ext, nullptr) << "loadModule failed: " << g_host_so;
  AbstractNavigation::Ptr nav =
      std::dynamic_pointer_cast<AbstractNavigation>(std::move(host_ext));
  ASSERT_NE(nav, nullptr);

  nav->initialize(dir.string());
  nav->start();

  // Virtual chassis drives the robot from the emitted commands.
  g_chassis_running = true;
  std::thread chassis(chassisLoop);

  // Navigate 3 m straight ahead.
  nav->setGoal(makePose(3.0, 0.0));

  const bool succeeded = waitUntil(
      [&] {
        std::lock_guard<std::mutex> lock(g_result_mutex);
        for (const auto& r : g_results) {
          if (r.status == ayanami::actions::GoalStatus::SUCCEEDED) {
            return true;
          }
        }
        return false;
      },
      30000);
  EXPECT_TRUE(succeeded)
      << "navigation never succeeded; results=" << g_results.size()
      << " cmds=" << g_cmd_count.load() << " exe_results="
      << [&] {
           std::lock_guard<std::mutex> lock(g_result_mutex);
           std::string out;
           for (const auto& r : g_exe_results) {
             out += " | " + r;
           }
           return out;
         }();

  // The closed loop really drove the robot to the goal.
  double final_x = 0.0;
  double final_y = 0.0;
  {
    std::lock_guard<std::mutex> lock(g_robot_mutex);
    final_x = g_robot_x;
    final_y = g_robot_y;
  }
  EXPECT_NEAR(final_x, 3.0, 0.35);
  EXPECT_NEAR(final_y, 0.0, 0.35);
  EXPECT_GT(g_cmd_count.load(), 5);
  EXPECT_GT(g_field_count.load(), 0);

  // Teardown must be clean.
  g_chassis_running = false;
  chassis.join();
  nav->stop();
  NavigationCallbacks::on_velocity_command.remove(g_cmd_cb_id);
  g_field_cb.reset();
  NavigationCallbacks::on_navigation_result.remove(g_result_cb_id);
  NavigationCallbacks::on_exe_path_result.remove(g_exe_cb_id);
  nav.reset();
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_host_so = argv[1];
  if (argc > 2) g_planner_so = argv[2];
  if (argc > 3) g_smoother_so = argv[3];
  if (argc > 4) g_controller_so = argv[4];
  if (argc > 5) g_escape_so = argv[5];
  if (argc > 6) g_wait_so = argv[6];
  if (argc > 7) g_goal_checker_so = argv[7];
  if (argc > 8) g_progress_checker_so = argv[8];
  if (argc > 9) g_bt_so_path = argv[9];
  return RUN_ALL_TESTS();
}
