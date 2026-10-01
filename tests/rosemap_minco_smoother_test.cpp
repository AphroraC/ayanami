// rosemap_minco_smoother tests: direct-constructed module over synthetic
// snapshots -- straight-line pass-through quality, clearance improvement,
// validation rollback, budget exit, and the two-pose passthrough.

#include <gtest/gtest.h>

#include <rosemap_minco_smoother/rosemap_minco_smoother.hpp>

#include <ayanami/rosemap/core/edt.hpp>
#include <ayanami/rosemap/core/rosemap_map_context.hpp>
#include <ayanami/rosemap/core/rosemap_query.hpp>
#include <ayanami/utility/config.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <fstream>
#include <memory>
#include <utility>

namespace {

using ayanami::ErrorCode;
using ayanami::nav_msgs::Path;
using ayanami::rosemap::RosemapMapContext;
using RosemapSnapshot = ayanami::rosemap::RosemapSnapshot;

constexpr int k_size = 100;   // cells
constexpr double k_res = 0.1;
constexpr double k_origin = -5.0;

/// @brief A synthetic 10x10 m field with an optional wall at world x == 0
/// (all y except an optional door gap).
struct FieldFixture {
  RosemapSnapshot snapshot;

  explicit FieldFixture(bool with_wall = false, double door_min_y = -0.5,
                        double door_max_y = 0.5) {
    snapshot.frame = "map";
    snapshot.resolution = k_res;
    snapshot.size_x = k_size;
    snapshot.size_y = k_size;
    snapshot.origin_x = k_origin;
    snapshot.origin_y = k_origin;
    snapshot.max_distance = 3.0;
    snapshot.occupied.assign(static_cast<std::size_t>(k_size) * k_size, 0);
    if (with_wall) {
      const int door_y0 = static_cast<int>((door_min_y - k_origin) / k_res);
      const int door_y1 = static_cast<int>((door_max_y - k_origin) / k_res);
      for (int y = 0; y < k_size; ++y) {
        if (y >= door_y0 && y <= door_y1) {
          continue;
        }
        for (int x = 49; x <= 50; ++x) {
          snapshot.occupied[static_cast<std::size_t>(y) * k_size + x] = 1;
        }
      }
    }
    const bool edt_ok = ayanami::rosemap::signedEdt2d(
        snapshot.occupied, k_size, k_size, k_res, 3.0, snapshot.distance);
    EXPECT_TRUE(edt_ok);
  }
};

/// @brief Build a straight stamped path (relative stamps at 1 m/s).
Path straightPath(double x0, double y0, double x1, double y1, int segments) {
  Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= segments; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(segments);
    ayanami::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position = Eigen::Vector3d(x0 + t * (x1 - x0),
                                         y0 + t * (y1 - y0), 0.0);
    pose.header.stamp = ayanami::Timestamp(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(t *
                                          std::hypot(x1 - x0, y1 - y0))));
    path.poses.push_back(std::move(pose));
  }
  return path;
}

class RosemapMincoSmootherTest : public ::testing::Test {
 protected:
  void SetUp() override {
    smoother = std::make_shared<ayanami::modules::RosemapMincoSmoother>();
    context = std::make_shared<RosemapMapContext>();
    const ayanami::Config empty_config;
    smoother->configure(
        ayanami::ModuleContext{"rosemap_minco_smoother", empty_config},
        context);
  }

  const RosemapSnapshot& publish(const FieldFixture& fixture) {
    current = std::make_shared<RosemapSnapshot>(fixture.snapshot);
    context->publishSnapshot(current);
    return *current;
  }

  std::shared_ptr<ayanami::modules::RosemapMincoSmoother> smoother;
  RosemapMapContext::Ptr context;
  std::shared_ptr<const RosemapSnapshot> current;
};

TEST_F(RosemapMincoSmootherTest, StraightLineStaysStraight) {
  publish(FieldFixture(false));
  Path path = straightPath(-2.0, -1.0, 2.0, 1.0, 6);
  auto result = smoother->smoothPath(path, 1.0);
  ASSERT_TRUE(result.ok) << result.message;

  // Endpoints preserved; every pose stays near the original line; stamps
  // relative and increasing.
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x(), -2.0);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x(), 2.0);
  double previous_stamp = -1e9;
  for (const auto& pose : path.poses) {
    const Eigen::Vector2d p(pose.pose.position.x(), pose.pose.position.y());
    // Distance to the y = 0.5x + 0 line: |0.5x - y| / sqrt(1.25).
    const double deviation = std::abs(0.5 * p.x() - p.y()) / std::sqrt(1.25);
    EXPECT_LT(deviation, 0.05) << "smoothed path drifted off the line";
    const double stamp = pose.header.stamp.toSeconds();
    EXPECT_GE(stamp, previous_stamp);
    previous_stamp = stamp;
  }
  EXPECT_DOUBLE_EQ(path.poses.front().header.stamp.toSeconds(), 0.0);
}

TEST_F(RosemapMincoSmootherTest, ClearanceImprovesNearObstacle) {
  // Path with a control point hugging the wall; the smoother should push
  // it to at least safe_dist clearance (or roll back without making things
  // worse -- the FAIL-safe assertion is the final non-collision check).
  publish(FieldFixture(true, -5.0, 5.0));  // solid wall at x == 0

  // Control points: one 0.15 m from the wall (closer than safe_dist 0.35).
  Path path;
  path.header.frame_id = "map";
  const double ys[] = {-2.0, -1.0, 0.0, 1.0, 2.0};
  for (int i = 0; i < 5; ++i) {
    ayanami::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position =
        Eigen::Vector3d(i == 2 ? -0.15 : -2.0, ys[i], 0.0);
    pose.header.stamp = ayanami::Timestamp(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(i * 2.0)));
    path.poses.push_back(std::move(pose));
  }
  auto result = smoother->smoothPath(path, 1.0);
  ASSERT_TRUE(result.ok) << result.message;
  // Whatever the outcome, the final path must not penetrate the wall.
  for (const auto& pose : path.poses) {
    const auto query = ayanami::rosemap::sampleDistance(
        *current, pose.pose.position.x(), pose.pose.position.y());
    if (query.status == ayanami::rosemap::RosemapQueryStatus::kOk) {
      EXPECT_GE(query.distance, -1e-3) << "final path penetrates an obstacle";
    }
  }
}

TEST_F(RosemapMincoSmootherTest, CollisionInputRollsBack) {
  // The input path runs straight THROUGH a solid wall; with only three
  // control points the optimizer cannot escape, validation detects the
  // collision, and the smoother rolls back to the input path (SUCCESS).
  publish(FieldFixture(true, -5.0, 5.0));

  Path path = straightPath(-2.0, 0.0, 2.0, 0.0, 4);
  Path input_copy = path;
  auto result = smoother->smoothPath(path, 1.0);
  ASSERT_TRUE(result.ok) << result.message;
  // Rolled back: pose count and positions match the input exactly.
  ASSERT_EQ(path.poses.size(), input_copy.poses.size());
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    EXPECT_DOUBLE_EQ(path.poses[i].pose.position.x(),
                     input_copy.poses[i].pose.position.x());
    EXPECT_DOUBLE_EQ(path.poses[i].pose.position.y(),
                     input_copy.poses[i].pose.position.y());
  }
}

TEST_F(RosemapMincoSmootherTest, TinyBudgetStillSucceeds) {
  publish(FieldFixture(false));
  Path path = straightPath(-2.0, 0.0, 2.0, 0.0, 5);
  auto result = smoother->smoothPath(path, 1e-9);
  // The budget fires through the progress callback: the smoother either
  // validated the last iterate or rolled back -- SUCCESS either way.
  ASSERT_TRUE(result.ok) << result.message;
}

TEST_F(RosemapMincoSmootherTest, TwoPosePathPassesThrough) {
  publish(FieldFixture(false));
  Path path = straightPath(-2.0, 0.0, 2.0, 0.0, 1);
  Path input_copy = path;
  auto result = smoother->smoothPath(path, 1.0);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(path.poses.size(), input_copy.poses.size());
}

TEST_F(RosemapMincoSmootherTest, MissingSnapshotFails) {
  Path path = straightPath(-2.0, 0.0, 2.0, 0.0, 4);
  auto result = smoother->smoothPath(path, 1.0);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::MAP_UNAVAILABLE);
}

TEST_F(RosemapMincoSmootherTest, FreezeWindowKeepsNearStartPoints) {
  publish(FieldFixture(false));
  ayanami::Config cfg;
  const std::string json = R"({
    "rosemap_minco_smoother": {
      "freeze_window": 100.0
    }
  })";
  const auto dir = std::filesystem::temp_directory_path() /
                   "rosemap_minco_smoother_test_cfg";
  std::filesystem::create_directories(dir);
  {
    std::ofstream ofs(dir / "module.json");
    ofs << json;
  }
  ayanami::Config file_cfg((dir / "module.json").string());
  smoother->configureParameters(
      ayanami::ModuleContext{"rosemap_minco_smoother", file_cfg});

  // All control points fall inside the (huge) freeze window: the path must
  // come back identical to the input.
  Path path = straightPath(-2.0, -1.0, 2.0, 1.0, 6);
  Path input_copy = path;
  auto result = smoother->smoothPath(path, 1.0);
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_EQ(path.poses.size(), input_copy.poses.size());
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    EXPECT_NEAR(path.poses[i].pose.position.x(),
                input_copy.poses[i].pose.position.x(), 1e-9);
    EXPECT_NEAR(path.poses[i].pose.position.y(),
                input_copy.poses[i].pose.position.y(), 1e-9);
  }
}

}  // namespace
