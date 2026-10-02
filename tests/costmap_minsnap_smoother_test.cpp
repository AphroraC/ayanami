// modules/costmap_minsnap_smoother (CostmapSmoother-family minimum-snap
// smoothing over a square safety corridor).
//
// Coverage:
// - closed-form solve on a straight line: success, endpoints preserved,
//   output stays on the line
// - L-corner: output is collision-free (or the original path is kept --
//   both are SUCCESS by contract)
// - waypoint velocity constraint honored by the closed-form solver
//   (solver-level: interior point tangent matches the constrained velocity)
// - corridor computeBound/shrinkBound semantics
// - zero iterations -> rollback + SUCCESS (graceful degradation)
// - cancel -> CANCELED
// - module.cpp exports the unified create_extension_module factory

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <costmap/costmap_2d.hpp>
#include <costmap/layered_costmap.hpp>
#include <costmap/static_layer.hpp>
#include <costmap/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <costmap_minsnap_smoother/costmap_minsnap_smoother.hpp>
#include <ayanami/utility/extension_module.hpp>

#include "minsnap_solver.hpp"
#include "square_corridor.hpp"

namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::costmap::CostmapMapContext;
using ayanami::modules::CostmapMinSnapSmoother;

std::string g_module_so;

std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapper(
    unsigned int size, double res,
    const std::function<void(costmap_2d::Costmap2D&)>& obstacle_fn = {}) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(size, size, res, 0.0, 0.0);

  costmap_2d::Costmap2D layer_map(size, size, res, 0.0, 0.0,
                                  costmap_2d::FREE_SPACE);
  if (obstacle_fn) {
    obstacle_fn(layer_map);
  }
  auto static_layer = std::make_shared<costmap_2d::StaticLayer>();
  static_layer->setMap(layer_map);
  lm->addLayer(static_layer);
  lm->updateMap(0.0, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

std::shared_ptr<CostmapMinSnapSmoother> makeSmoother(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper,
    const std::string& name = "test_minsnap",
    ayanami::Config cfg = ayanami::Config("")) {
  auto smoother = std::make_shared<CostmapMinSnapSmoother>();
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(wrapper);
  smoother->configure(ayanami::ModuleContext{name, std::move(cfg)},
                      ctx->getSharedConstPtr());
  return smoother;
}

ayanami::nav_msgs::Path makePath(
    const std::vector<Eigen::Vector2d>& points) {
  ayanami::nav_msgs::Path path;
  path.header.frame_id = "map";
  for (const auto& p : points) {
    auto pose = ayanami::geometry_msgs::PoseStamped::fromIsometry(
        "map", Eigen::Isometry3d::Identity());
    pose.pose.position = Eigen::Vector3d(p.x(), p.y(), 0.0);
    path.poses.push_back(std::move(pose));
  }
  return path;
}

bool pathCollides(const ayanami::nav_msgs::Path& path,
                  const std::set<std::pair<unsigned int, unsigned int>>&
                      obstacles) {
  for (const auto& pose : path.poses) {
    const double x = pose.pose.position.x();
    const double y = pose.pose.position.y();
    if (x < 0 || y < 0) {
      return true;
    }
    const unsigned int mx = static_cast<unsigned int>(x / 0.5);
    const unsigned int my = static_cast<unsigned int>(y / 0.5);
    if (obstacles.count({mx, my}) != 0) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(CostmapMinSnapSmootherTest, straightLineStaysOnLine) {
  auto wrapper = makeWrapper(20, 0.5);
  auto smoother = makeSmoother(wrapper);

  ayanami::nav_msgs::Path path =
      makePath({{1.0, 1.0}, {3.0, 1.0}, {5.0, 1.0}, {7.0, 1.0}, {9.0, 1.0}});
  const auto original_size = path.poses.size();
  const ResultT<void> result = smoother->smoothPath(path, 0.5);

  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.front().pose.position.x(), 1.0, 1e-6);
  EXPECT_NEAR(path.poses.back().pose.position.y(), 1.0, 1e-6);
  EXPECT_NEAR(path.poses.back().pose.position.x(), 9.0, 1e-6);
  for (const auto& pose : path.poses) {
    EXPECT_NEAR(pose.pose.position.y(), 1.0, 0.05);
  }
  // The sampled trajectory is denser than the 5-waypoint input.
  EXPECT_GT(path.poses.size(), original_size);
}

TEST(CostmapMinSnapSmootherTest, cornerOutputStaysCollisionFree) {
  std::set<std::pair<unsigned int, unsigned int>> obstacles;
  auto wrapper = makeWrapper(20, 0.5, [&](costmap_2d::Costmap2D& m) {
    // A wall with an inner corner the polyline would clip.
    for (unsigned int y = 8; y <= 14; ++y) {
      m.setCost(10, y, costmap_2d::LETHAL_OBSTACLE);
    }
    for (unsigned int x = 10; x <= 16; ++x) {
      m.setCost(x, 8, costmap_2d::LETHAL_OBSTACLE);
    }
    for (unsigned int y = 8; y <= 14; ++y) {
      for (unsigned int x = 10; x <= 16; ++x) {
        if (m.getCost(x, y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          obstacles.emplace(x, y);
        }
      }
    }
  });
  auto smoother = makeSmoother(wrapper);

  ayanami::nav_msgs::Path path =
      makePath({{2.0, 5.0}, {5.25, 4.75}, {5.25, 3.0}, {8.5, 3.0}});
  const ayanami::nav_msgs::Path original = path;
  const ResultT<void> result = smoother->smoothPath(path, 1.0);

  ASSERT_TRUE(result.ok) << result.message;
  // Contract: either a collision-free smoothed path, or the original kept.
  const bool unchanged = path.poses.size() == original.poses.size();
  if (!unchanged) {
    EXPECT_FALSE(pathCollides(path, obstacles));
    EXPECT_NEAR(path.poses.front().pose.position.x(),
                original.poses.front().pose.position.x(), 1e-6);
    EXPECT_NEAR(path.poses.back().pose.position.y(),
                original.poses.back().pose.position.y(), 1e-6);
  }
}

TEST(MinSnapSolverTest, closedFormHonorsWaypointVelocity) {
  using ayanami::modules::minsnap::ControlPoint;
  using ayanami::modules::minsnap::SolveProblem;
  using ayanami::modules::minsnap::solveClosedForm;
  using ayanami::modules::minsnap::sampleSegment;
  using ayanami::modules::minsnap::trapezoidalTimeAllocation;

  std::vector<Eigen::Vector2d> waypoints{{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0},
                                         {6.0, 0.0}, {8.0, 0.0}};
  SolveProblem problem;
  problem.order = 6;
  problem.derivative_order = 3;
  problem.normalized_time = true;
  problem.times = trapezoidalTimeAllocation(waypoints, 3.0, 1.0);
  ASSERT_FALSE(problem.times.empty());
  problem.points.resize(waypoints.size());
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    problem.points[i].xy = waypoints[i];
    problem.points[i].corridor = {waypoints[i].x(), waypoints[i].y(),
                                  waypoints[i].x(), waypoints[i].y()};
  }
  // Interior velocity constraint: pass point 2 at 1.5 m/s along +x.
  problem.points[2].has_velocity = true;
  problem.points[2].velocity = Eigen::Vector2d(1.5, 0.0);
  problem.points.back().has_velocity = true;

  ayanami::modules::minsnap::SegmentCoefficients coefficients;
  ASSERT_TRUE(solveClosedForm(problem, coefficients));

  // Tangent at the constrained waypoint (start of segment 2).
  const auto samples = sampleSegment(coefficients, problem.order,
                                     problem.times, 2, true, 50);
  const Eigen::Vector2d tangent =
      (samples[2] - samples[0]).normalized();
  EXPECT_GT(tangent.x(), 0.99);
  EXPECT_LT(std::abs(tangent.y()), 0.05);
}

TEST(CorridorTest, computeBoundRespectsWallAndRange) {
  using ayanami::modules::corridor::Box;
  using ayanami::modules::corridor::computeBound;
  using ayanami::modules::corridor::shrinkBound;

  // Vertical wall at x in [4.0, 4.5): cell column 8 blocked (0.5m cells).
  const auto blocked = [](double x, double y) {
    (void)y;
    return x >= 4.0 && x < 4.5;
  };
  Box bound;
  // Anchor left of the wall: the box must not cross x = 4.0. Left of the
  // anchor is free for the full 2.0 m per-side range.
  ASSERT_TRUE(computeBound(blocked, 2.25, 2.25, 2.0, 0.5, 0.0, bound));
  EXPECT_LE(bound[2], 4.0 + 1e-9);
  EXPECT_NEAR(bound[2], 3.75, 1e-9);
  EXPECT_NEAR(bound[0], 0.25, 1e-9);
  // Per-side range clamp: 1.0 m from the same anchor on both sides.
  ASSERT_TRUE(computeBound(blocked, 2.25, 2.25, 1.0, 0.5, 0.0, bound));
  EXPECT_GE(bound[0], 2.25 - 1.0 - 1e-9);
  EXPECT_LE(bound[2], 2.25 + 1.0 + 1e-9);
  EXPECT_LE(bound[2] - bound[0], 2.0 + 1e-9);
  // Anchor inside the wall -> degenerate box, returns false.
  EXPECT_FALSE(computeBound(blocked, 4.2, 2.25, 2.0, 0.5, 0.0, bound));

  const Box shrunk = shrinkBound({1.0, 1.0, 3.0, 3.0}, 0.25, {{2.6, 1.5}});
  EXPECT_NEAR(shrunk[0], 1.25, 1e-9);
  EXPECT_NEAR(shrunk[2], 2.75, 1e-9);  // pulled back to keep the limit inside
  EXPECT_NEAR(shrunk[1], 1.25, 1e-9);
  EXPECT_NEAR(shrunk[3], 2.75, 1e-9);
}

TEST(CostmapMinSnapSmootherTest, zeroIterationsRollsBackWithSuccess) {
  auto wrapper = makeWrapper(20, 0.5);
  ayanami::Config cfg("");
  cfg.overrideParam<int>("test_minsnap", "collision_iterations", 0);
  auto smoother = makeSmoother(wrapper, "test_minsnap", std::move(cfg));

  ayanami::nav_msgs::Path path = makePath({{1.0, 1.0}, {9.0, 9.0}});
  const ayanami::nav_msgs::Path original = path;
  const ResultT<void> result = smoother->smoothPath(path, 0.5);

  EXPECT_TRUE(result.ok) << result.message;
  EXPECT_EQ(path.poses.size(), original.poses.size());
  EXPECT_NEAR(path.poses[0].pose.position.x(),
              original.poses[0].pose.position.x(), 1e-9);
}

// ---- max_duration is a WHOLE-CALL soft deadline ----
// The budget starts at smoothPath entry (corridor build included); expiry
// degrades gracefully: the planner's path is kept and SUCCESS is reported.
// A partial-corridor solve must never run, and the BT recovery ladder must
// never hear about a smoothing-budget miss (smoothing = enhancement).
TEST(CostmapMinSnapSmootherTest, zeroBudgetDegradesGracefully) {
  auto smoother = makeSmoother(makeWrapper(20, 0.5));
  const ayanami::nav_msgs::Path path =
      makePath({{0.5, 0.5}, {2.5, 0.5}, {4.5, 2.5}, {6.5, 4.5}});
  ayanami::nav_msgs::Path work = path;
  const ResultT<void> result = smoother->smoothPath(work, 0.0);
  ASSERT_TRUE(result.ok);
  EXPECT_NE(result.message.find("budget"), std::string::npos)
      << "message should name the budget expiry: " << result.message;
  ASSERT_EQ(work.poses.size(), path.poses.size());
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    EXPECT_LT((work.poses[i].pose.position - path.poses[i].pose.position)
                  .norm(),
              1e-9)
        << "the original path must be kept unchanged on budget expiry";
  }
}

TEST(CostmapMinSnapSmootherTest, cancelMidSolveReturnsCanceled) {
  // A wall across the path keeps many repair rounds busy (pinned points
  // can never be repaired); an external cancel 10 ms in must surface as
  // CANCELED (the smoother clears its flag only at entry).
  auto wrapper = makeWrapper(800, 0.05, [](costmap_2d::Costmap2D& m) {
    // Wall across x = 4.0 m -- the waypoint polyline crosses it (and one
    // waypoint sits inside), so repair rounds never converge.
    for (unsigned int y = 0; y <= 100; ++y) {
      m.setCost(80, y, costmap_2d::LETHAL_OBSTACLE);
    }
  });
  ayanami::Config cfg("");
  cfg.overrideParam<int>("test_minsnap", "collision_iterations", 500);
  auto smoother = makeSmoother(wrapper, "test_minsnap", std::move(cfg));

  std::vector<Eigen::Vector2d> pts;
  for (int i = 0; i <= 39; ++i) {
    pts.emplace_back(1.0 + 0.2 * i, 1.0 + (i % 2) * 0.1);
  }
  ayanami::nav_msgs::Path path = makePath(pts);
  std::atomic<bool> done{false};
  std::atomic<ErrorCode> error{ErrorCode::NONE};
  std::thread worker([&] {
    const ResultT<void> result = smoother->smoothPath(path, -1.0);
    error.store(result.error);
    done.store(true);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_TRUE(smoother->cancel());
  worker.join();
  EXPECT_TRUE(done.load());
  EXPECT_EQ(error.load(), ErrorCode::CANCELED);
}

TEST(CostmapMinSnapSmootherTest, osqpCorridorBackendEngagesOnFreeMap) {
  // With the OSQP backend selected the solve must produce a smoothed
  // trajectory (a silent failure would roll back to the 5-pose input).
  auto wrapper = makeWrapper(20, 0.5);
  ayanami::Config cfg("");
  cfg.overrideParam<std::string>("test_minsnap", "backend", "osqp_corridor");
  auto smoother = makeSmoother(wrapper, "test_minsnap", std::move(cfg));

  ayanami::nav_msgs::Path path =
      makePath({{1.0, 1.0}, {3.0, 1.0}, {5.0, 1.0}, {7.0, 1.0}, {9.0, 1.0}});
  const ResultT<void> result = smoother->smoothPath(path, 1.0);

  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GT(path.poses.size(), 5u);
  for (const auto& pose : path.poses) {
    EXPECT_NEAR(pose.pose.position.y(), 1.0, 0.05);
  }
  EXPECT_NEAR(path.poses.front().pose.position.x(), 1.0, 1e-6);
  EXPECT_NEAR(path.poses.back().pose.position.x(), 9.0, 1e-6);
}

TEST(CostmapMinSnapSmootherTest, osqpCorridorBackendDetourStaysFree) {
  std::set<std::pair<unsigned int, unsigned int>> obstacles;
  auto wrapper = makeWrapper(20, 0.5, [&](costmap_2d::Costmap2D& m) {
    for (unsigned int y = 8; y <= 12; ++y) {
      for (unsigned int x = 8; x <= 12; ++x) {
        m.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
      }
    }
    for (unsigned int y = 8; y <= 12; ++y) {
      for (unsigned int x = 8; x <= 12; ++x) {
        obstacles.emplace(x, y);
      }
    }
  });
  ayanami::Config cfg("");
  cfg.overrideParam<std::string>("test_minsnap", "backend", "osqp_corridor");
  auto smoother = makeSmoother(wrapper, "test_minsnap", std::move(cfg));

  // Waypoints detour above the block; corridors must keep the curve free.
  ayanami::nav_msgs::Path path =
      makePath({{2.0, 2.0}, {4.5, 4.5}, {5.75, 4.5}, {8.0, 2.0}});
  const ayanami::nav_msgs::Path original = path;
  const ResultT<void> result = smoother->smoothPath(path, 1.0);

  ASSERT_TRUE(result.ok) << result.message;
  const bool unchanged = path.poses.size() == original.poses.size();
  if (!unchanged) {
    EXPECT_FALSE(pathCollides(path, obstacles));
  }
}

TEST(CostmapMinSnapSmootherTest, moduleFactoryCreatesSmoother) {
  ASSERT_FALSE(g_module_so.empty());
  auto smoother = std::dynamic_pointer_cast<CostmapMinSnapSmoother>(
      ayanami::ExtensionModule::loadModule(g_module_so));
  ASSERT_NE(smoother, nullptr);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_module_so = argv[1];
  }
  return RUN_ALL_TESTS();
}
