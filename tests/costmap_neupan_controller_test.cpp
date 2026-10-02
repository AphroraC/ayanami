// Module-level tests for costmap_neupan_controller (stage-4 acceptance of
// logs/neupan-controller-migration-plan.md).
//
// Coverage:
// - factory symbol + family inheritance (dlopen full chain);
// - no-plan failure code; straight-run arrival (zeros + ok on arrive);
// - heading outer loop reacts to a diagonal reference;
// - the obstacle feed wiring: a real SimpleObstacleLayer presence mark,
//   read back through CostmapMapContext::snapshotLocalObstaclePresence as
//   map-frame points, decelerates the robot before the point;
// - the tolerance-driven planner rebuild keeps tracking.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <costmap/costmap_wrapper.hpp>
#include <costmap/layered_costmap.hpp>
#include <costmap/simple_obstacle_layer.hpp>

#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

#include "costmap_neupan_controller.hpp"

namespace {

namespace fs = std::filesystem;
using ayanami::ErrorCode;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::nav_msgs::Path;

fs::path g_config_dir;
std::string g_module_so;

constexpr double kDt = 0.05;  // control period used for the integration

PoseStamped makePose(double x, double y, double yaw = 0.0) {
  PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x() = x;
  pose.pose.position.y() = y;
  pose.pose.orientation =
      Eigen::Quaterniond(std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0));
  return pose;
}

Path makePath(double x0, double y0, double x1, double y1, int steps) {
  Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    // Unit quaternion on purpose: the module must derive the tangent yaw.
    path.poses.push_back(makePose(x0 + t * (x1 - x0), y0 + t * (y1 - y0)));
    path.poses.back().pose.orientation = Eigen::Quaterniond::Identity();
  }
  return path;
}

void writeModuleJson(const std::string& extra = "") {
  std::ofstream ofs(g_config_dir / "costmap_neupan_controller.json");
  ofs << R"({
  "controller": {"name": "costmap_neupan_controller", "so_name": "libayanami_costmap_neupan_controller.so"},
  "costmap_neupan_controller": {
    "kinematics": "omni", "receding": 10, "step_time": 0.1, "ref_speed": 1.0,
    "collision_threshold": 0.3, "length": 0.5, "width": 0.5,
    "max_speed_v": 1.5, "max_speed_phi": 1.57, "max_acce_v": 4.0, "max_acce_phi": 4.0,
    "dune_checkpoint": ")" << AYANAMI_NEUPAN_MODEL_DIR
      << R"(", "ipath_interval": 0.1, "arrive_threshold": 0.1, "ind_range": 10,
    "iter_num": 2, "dune_max_num": 200, "nrmp_max_num": 10, "iter_threshold": 0.01,
    "q_s": 0.1, "p_u": 2.0, "eta": 15.0, "d_min": 0.1, "d_max": 1.0,
    "ro_obs": 50.0, "bk": 0.0,
    "osqp_eps_abs": 0.0001, "osqp_eps_rel": 0.0001, "osqp_max_iter": 1000,
    "osqp_time_limit": 0.02, "osqp_polish": true, "osqp_verbose": false,
    "feed_max_points": 200, "heading_kp": 2.5, "heading_deadband": 0.05,
    "max_angular_z": 3.0, "heading_lookahead": 3)" << extra << "\n  }\n}";
}

ayanami::modules::CostmapNeupanController::Ptr makeController(
    const ayanami::costmap::CostmapMapContext::ConstPtr& map = nullptr) {
  auto controller = std::make_shared<ayanami::modules::CostmapNeupanController>();
  controller->configureParameters(ayanami::ModuleContext{
      "costmap_neupan_controller",
      ayanami::Config((g_config_dir / "costmap_neupan_controller.json").string())});
  if (map) {
    ayanami::costmap::CostmapController::Ptr family = controller;
    family->configure(
        ayanami::ModuleContext{"costmap_neupan_controller",
                               ayanami::Config((g_config_dir / "costmap_neupan_controller.json").string())},
        map);
  }
  return controller;
}

TwistStamped compute(const ayanami::modules::CostmapNeupanController::Ptr& ctl,
                     const PoseStamped& pose, ErrorCode& failure) {
  TwistStamped cmd;
  const ayanami::ResultT<void> result =
      ctl->computeVelocityCommand(pose, TwistStamped{}, cmd);
  failure = result.ok ? ErrorCode::NONE : result.error;
  return cmd;
}

/// @brief A local map with one obstacle mark, same frame as the poses.
ayanami::costmap::CostmapMapContext::ConstPtr makeMapWithObstacle(
    double obstacle_x, double obstacle_y) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/true, /*track_unknown=*/false);
  lm->resizeMap(80, 80, 0.05, -2.0, -2.0);
  auto layer = std::make_shared<costmap_2d::SimpleObstacleLayer>();
  lm->addLayer(layer);
  auto wrapper = std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
  layer->markObstacles(*lm->getCostmap(), obstacle_x, obstacle_y);

  auto ctx = std::make_shared<ayanami::costmap::CostmapMapContext>();
  ctx->setLocalCostmap(wrapper);
  return ctx->getSharedConstPtr();
}

}  // namespace

TEST(CostmapNeupanControllerTest, ModuleFactoryCreatesDerivedController) {
  ASSERT_FALSE(g_module_so.empty());
  auto controller = std::dynamic_pointer_cast<ayanami::modules::CostmapNeupanController>(
      ayanami::ExtensionModule::loadModule(g_module_so));
  ASSERT_NE(controller, nullptr);
  // Family chain: costmap controller from the abstract root.
  ayanami::abstract::AbstractController::Ptr base = controller;
  EXPECT_NE(std::dynamic_pointer_cast<ayanami::costmap::CostmapController>(base),
            nullptr);
}

TEST(CostmapNeupanControllerTest, FailsWithoutPlan) {
  auto ctl = makeController();
  ErrorCode failure = ErrorCode::NONE;
  const TwistStamped cmd = compute(ctl, makePose(0, 0), failure);
  EXPECT_EQ(failure, ErrorCode::INVALID_INPUT);
  EXPECT_EQ(cmd.twist.linear.x(), 0.0);
}

TEST(CostmapNeupanControllerTest, StraightRunAdvancesAndArrivesWithZeros) {
  auto ctl = makeController();  // no map: nominal tracking, no points
  ASSERT_TRUE(ctl->setPlan(makePath(0, 0, 3, 0, 30)));

  double x = 0.0;
  bool arrived_zeros = false;
  ErrorCode failure = ErrorCode::NONE;
  for (int frame = 0; frame < 600; ++frame) {
    const TwistStamped cmd = compute(ctl, makePose(x, 0), failure);
    ASSERT_EQ(failure, ErrorCode::NONE) << "frame " << frame;
    x += cmd.twist.linear.x() * kDt;
    if (x > 2.5 && std::abs(cmd.twist.linear.x()) < 1e-6 &&
        std::abs(cmd.twist.linear.y()) < 1e-6) {
      arrived_zeros = true;
      break;
    }
  }
  EXPECT_GT(x, 2.0) << "robot advanced along +x, at " << x;
  EXPECT_TRUE(arrived_zeros)
      << "the planner's internal arrival must zero the twist (the goal "
         "checker owns the verdict)";
}

TEST(CostmapNeupanControllerTest, HeadingOuterLoopSteersTowardDiagonal) {
  auto ctl = makeController();
  ASSERT_TRUE(ctl->setPlan(makePath(0, 0, 2, 2, 20)));

  // Body yaw 0 while the reference runs at atan2(1,1) ~ 0.785: the outer
  // loop must command a positive turn (kp * err, capped at max_angular_z).
  ErrorCode failure = ErrorCode::NONE;
  const TwistStamped cmd = compute(ctl, makePose(0, 0, 0.0), failure);
  ASSERT_EQ(failure, ErrorCode::NONE);
  EXPECT_GT(cmd.twist.angular.z(), 0.5);
  EXPECT_LT(cmd.twist.angular.z(), 3.0 + 1e-9);
  // A body already aligned commands (almost) nothing.
  const TwistStamped aligned = compute(ctl, makePose(0, 0, 0.785398), failure);
  EXPECT_NEAR(aligned.twist.angular.z(), 0.0, 0.35);
}

TEST(CostmapNeupanControllerTest, PresenceFeedDeceleratesBeforeObstacle) {
  const ayanami::costmap::CostmapMapContext::ConstPtr map =
      makeMapWithObstacle(1.5, 0.0);
  auto ctl = makeController(map);
  ASSERT_TRUE(ctl->setPlan(makePath(0, 0, 4, 0, 40)));

  double x = 0.0;
  double max_late_vx = 0.0;
  ErrorCode failure = ErrorCode::NONE;
  for (int frame = 0; frame < 400; ++frame) {
    const TwistStamped cmd = compute(ctl, makePose(x, 0), failure);
    ASSERT_EQ(failure, ErrorCode::NONE) << "frame " << frame;
    x += cmd.twist.linear.x() * kDt;
    if (x > 0.6) {
      max_late_vx = std::max(max_late_vx, cmd.twist.linear.x());
    }
    // footprint half length 0.25: the front edge must never pass the point
    ASSERT_LE(x, 1.5 - 0.25 + 0.06)
        << "front edge grazed the obstacle at frame " << frame;
    if (std::abs(cmd.twist.linear.x()) < 1e-6 && x > 0.5) break;
  }
  EXPECT_LT(max_late_vx, 0.75)
      << "the robot must slow near the point (ref_speed 1.0), late vx was "
      << max_late_vx;
}

TEST(CostmapNeupanControllerTest, ToleranceRebuildKeepsTracking) {
  auto ctl = makeController();
  // Tighter than param.arrive_threshold (0.1): triggers the one-shot
  // planner rebuild (the warn-only clamp contract).
  ayanami::PoseTolerance tight;
  tight.position = 0.05;
  ctl->setGoalTolerances(tight);
  ASSERT_TRUE(ctl->setPlan(makePath(0, 0, 2, 0, 20)));

  double x = 0.0;
  ErrorCode failure = ErrorCode::NONE;
  for (int frame = 0; frame < 400; ++frame) {
    const TwistStamped cmd = compute(ctl, makePose(x, 0), failure);
    ASSERT_EQ(failure, ErrorCode::NONE) << "frame " << frame;
    x += cmd.twist.linear.x() * kDt;
    if (x > 1.5) break;
  }
  EXPECT_GT(x, 1.2) << "tracking survived the rebuild, at " << x;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_module_so = argv[1];
  }
  g_config_dir = fs::temp_directory_path() / "ayanami_neupan_controller_test";
  fs::remove_all(g_config_dir);
  fs::create_directories(g_config_dir);
  {
    std::ofstream ofs(g_config_dir / "config.json");
    ofs << R"({"global": { "config_controller": "costmap_neupan_controller.json" }})";
  }
  writeModuleJson();
  ayanami::GlobalConfig::instance(g_config_dir.string(), true);
  return RUN_ALL_TESTS();
}
