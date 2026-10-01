// rosemap_astar_planner tests: direct-constructed module over synthetic
// snapshots -- corridor detour, clearance guarantee, relative time
// parameterization (turn limiting visible in the stamps), goal repair,
// cancellation, and failure modes.

#include <gtest/gtest.h>

#include <rosemap_astar_planner/rosemap_astar_planner.hpp>

#include <ayanami/rosemap/core/edt.hpp>

#include <ayanami/rosemap/core/rosemap_map_context.hpp>
#include <ayanami/rosemap/core/rosemap_query.hpp>
#include <ayanami/utility/config.hpp>

#include <cmath>
#include <chrono>
#include <filesystem>
#include <thread>
#include <fstream>
#include <memory>
#include <utility>

namespace {

using ayanami::ErrorCode;
using ayanami::nav_msgs::Path;
using ayanami::rosemap::RosemapMapContext;
using RosemapSnapshot = ayanami::rosemap::RosemapSnapshot;

constexpr int k_size = 80;  // cells
constexpr double k_res = 0.1;
constexpr double k_origin = -4.0;

/// @brief A synthetic field: an empty 8x8 m area with one vertical wall of
/// marked cells at world x == 0, spanning all y except a door gap.
struct FieldFixture {
  RosemapSnapshot snapshot;

  explicit FieldFixture(double door_min_y = -0.5, double door_max_y = 0.5) {
    snapshot.frame = "map";
    snapshot.resolution = k_res;
    snapshot.size_x = k_size;
    snapshot.size_y = k_size;
    snapshot.origin_x = k_origin;
    snapshot.origin_y = k_origin;
    snapshot.max_distance = 3.0;
    snapshot.occupied.assign(static_cast<std::size_t>(k_size) * k_size, 0);
    // Wall covering world x in [-0.1, 0.1] -> cells 39..40.
    const int wall_x0 = 39;
    const int wall_x1 = 40;
    const int door_y0 = static_cast<int>((door_min_y - k_origin) / k_res);
    const int door_y1 = static_cast<int>((door_max_y - k_origin) / k_res);
    for (int y = 0; y < k_size; ++y) {
      if (y >= door_y0 && y <= door_y1) {
        continue;
      }
      for (int x = wall_x0; x <= wall_x1; ++x) {
        snapshot.occupied[static_cast<std::size_t>(y) * k_size + x] = 1;
      }
    }
    const bool edt_ok = ayanami::rosemap::signedEdt2d(
        snapshot.occupied, k_size, k_size, k_res, 3.0, snapshot.distance);
    EXPECT_TRUE(edt_ok);
  }
};

class RosemapAstarPlannerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    planner = std::make_shared<ayanami::modules::RosemapAstarPlanner>();
    context = std::make_shared<RosemapMapContext>();
    const ayanami::Config empty_config;
    planner->configure(
        ayanami::ModuleContext{"rosemap_astar_planner", empty_config},
        context);
  }

  /// @brief Publish a fixture snapshot and return a reference for queries.
  const RosemapSnapshot& publish(const FieldFixture& fixture) {
    current = std::make_shared<RosemapSnapshot>(fixture.snapshot);
    context->publishSnapshot(current);
    return *current;
  }

  std::shared_ptr<ayanami::modules::RosemapAstarPlanner> planner;
  RosemapMapContext::Ptr context;
  std::shared_ptr<const RosemapSnapshot> current;
};

TEST_F(RosemapAstarPlannerTest, DetoursThroughTheDoorAndKeepsClearance) {
  const auto& snapshot = publish(FieldFixture(-0.5, 0.5));

  ayanami::geometry_msgs::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.position = Eigen::Vector3d(-2.0, -2.0, 0.0);
  start.pose.orientation = Eigen::Quaterniond::Identity();
  ayanami::geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position = Eigen::Vector3d(2.0, -2.0, 0.0);
  goal.pose.orientation = Eigen::Quaterniond::Identity();

  Path plan;
  auto result = planner->makePlan(start, goal, 0.0, plan);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_GE(plan.poses.size(), 2u);

  // The path must pass through the door gap (somewhere around y ~ 0).
  bool near_door = false;
  for (const auto& pose : plan.poses) {
    if (pose.pose.position.x() > -0.4 && pose.pose.position.x() < 0.4 &&
        std::abs(pose.pose.position.y()) < 1.2) {
      near_door = true;
    }
  }
  EXPECT_TRUE(near_door) << "path did not pass through the door gap";

  // Every sampled point keeps safe_radius clearance (default 0.3).
  for (const auto& pose : plan.poses) {
    const auto query = ayanami::rosemap::sampleDistance(
        snapshot, pose.pose.position.x(), pose.pose.position.y());
    ASSERT_EQ(query.status, ayanami::rosemap::RosemapQueryStatus::kOk);
    EXPECT_GE(query.distance, 0.29) << "path violates clearance";
  }

  // Stamps are relative, finite, and nondecreasing from zero.
  double previous = -1.0;
  for (const auto& pose : plan.poses) {
    const double t = pose.header.stamp.toSeconds();
    EXPECT_GE(t, previous);
    EXPECT_LT(t, previous + 60.0);
    previous = t;
  }
  EXPECT_DOUBLE_EQ(plan.poses.front().header.stamp.toSeconds(), 0.0);
}

TEST_F(RosemapAstarPlannerTest, SharpTurnSlowsDown) {
  publish(FieldFixture(-0.5, 0.5));

  // Start far north of the door, goal south of it: a sharp turn at the door.
  ayanami::geometry_msgs::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.position = Eigen::Vector3d(-2.0, 2.5, 0.0);
  ayanami::geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position = Eigen::Vector3d(2.0, -2.5, 0.0);

  Path plan;
  auto result = planner->makePlan(start, goal, 0.0, plan);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_GE(plan.poses.size(), 3u);
  // With turn limiting, the slowest segment must be slower than a
  // straight-shot would be (expected_speed 2.0 over a ~7.8 m route).
  double min_speed = 1e9;
  double total_time = 0.0;
  for (std::size_t i = 1; i < plan.poses.size(); ++i) {
    const double dt = plan.poses[i].header.stamp.toSeconds() -
                      plan.poses[i - 1].header.stamp.toSeconds();
    const double ds =
        (plan.poses[i].pose.position - plan.poses[i - 1].pose.position)
            .head<2>()
            .norm();
    if (dt > 1e-6) {
      min_speed = std::min(min_speed, ds / dt);
    }
    total_time += dt;
  }
  EXPECT_LT(min_speed, 1.5) << "turn limiting never engaged";
  EXPECT_GT(total_time, 0.5);
}

TEST_F(RosemapAstarPlannerTest, OutOfFieldGoalIsProjected) {
  publish(FieldFixture());

  ayanami::geometry_msgs::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.position = Eigen::Vector3d(-1.0, 0.0, 0.0);
  ayanami::geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position = Eigen::Vector3d(30.0, 0.0, 0.0);  // far outside

  Path plan;
  auto result = planner->makePlan(start, goal, 0.0, plan);
  ASSERT_TRUE(result.ok) << result.message;
  // Rose semantics: the path runs to the field boundary projection, then
  // the REAL goal is appended so the controller keeps pushing toward it
  // while the rolling window slides (the next replan closes the gap).
  EXPECT_DOUBLE_EQ(plan.poses.back().pose.position.x(), 30.0);
  // The in-field prefix reaches the boundary projection (~ +3.9).
  bool reaches_boundary = false;
  for (std::size_t i = 0; i + 1 < plan.poses.size(); ++i) {
    if (plan.poses[i].pose.position.x() > 3.0) {
      reaches_boundary = true;
    }
  }
  EXPECT_TRUE(reaches_boundary);
}

TEST_F(RosemapAstarPlannerTest, UnsafeStartFails) {
  publish(FieldFixture());

  ayanami::geometry_msgs::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.position = Eigen::Vector3d(0.0, -2.0, 0.0);  // inside the wall
  ayanami::geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position = Eigen::Vector3d(2.0, -2.0, 0.0);

  Path plan;
  auto result = planner->makePlan(start, goal, 0.0, plan);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::START_INVALID);
}

TEST_F(RosemapAstarPlannerTest, CancelAbortsPlanning) {
  // makePlan resets its cancel flag at entry (family convention: a stale
  // flag from a previous task must not poison the next run), so the cancel
  // has to land DURING a search: run one big enough to be still expanding
  // when the main thread cancels.
  constexpr int k_big = 500;
  RosemapSnapshot big;
  big.frame = "map";
  big.resolution = k_res;
  big.size_x = k_big;
  big.size_y = k_big;
  big.origin_x = -25.0;
  big.origin_y = -25.0;
  big.max_distance = 3.0;
  big.occupied.assign(static_cast<std::size_t>(k_big) * k_big, 0);
  const bool edt_ok =
      ayanami::rosemap::signedEdt2d(big.occupied, k_big, k_big, k_res, 3.0,
                                    big.distance);
  ASSERT_TRUE(edt_ok);
  current = std::make_shared<RosemapSnapshot>(std::move(big));
  context->publishSnapshot(current);

  ayanami::geometry_msgs::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.position = Eigen::Vector3d(-24.0, -24.0, 0.0);
  ayanami::geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position = Eigen::Vector3d(24.0, 24.0, 0.0);

  Path plan;
  ayanami::ResultT<void> result =
      ayanami::ResultT<void>::failure(ErrorCode::NONE, "not run");
  std::thread planner_thread([&] {
    result = planner->makePlan(start, goal, 0.0, plan);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  EXPECT_TRUE(planner->cancel());
  planner_thread.join();
  EXPECT_FALSE(result.ok) << "expected the cancel to land mid-search";
  EXPECT_EQ(result.error, ErrorCode::CANCELED);
}

TEST_F(RosemapAstarPlannerTest, MissingSnapshotFails) {
  ayanami::geometry_msgs::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.position = Eigen::Vector3d(-2.0, 0.0, 0.0);
  ayanami::geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position = Eigen::Vector3d(2.0, 0.0, 0.0);

  Path plan;
  auto result = planner->makePlan(start, goal, 0.0, plan);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::MAP_UNAVAILABLE);
}

TEST_F(RosemapAstarPlannerTest, ParamStructLoadsFromConfig) {
  // Config-owned values land in the module's param struct (loader seam);
  // behavior follows them (4-connected search still crosses the door).
  const std::string json = R"({
    "rosemap_astar_planner": {
      "safe_radius": 0.25,
      "expected_speed": 1.5,
      "use_diagonal": false
    }
  })";
  const auto dir = std::filesystem::temp_directory_path() /
                   "rosemap_astar_planner_test_cfg";
  std::filesystem::create_directories(dir);
  const auto path = dir / "module.json";
  {
    std::ofstream ofs(path);
    ofs << json;
  }
  ayanami::Config cfg(path.string());
  planner->configureParameters(
      ayanami::ModuleContext{"rosemap_astar_planner", cfg});

  publish(FieldFixture(-0.5, 0.5));
  ayanami::geometry_msgs::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.position = Eigen::Vector3d(-2.0, -2.0, 0.0);
  ayanami::geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position = Eigen::Vector3d(2.0, -2.0, 0.0);
  Path plan;
  auto result = planner->makePlan(start, goal, 0.0, plan);
  EXPECT_TRUE(result.ok) << result.message;
}

}  // namespace
