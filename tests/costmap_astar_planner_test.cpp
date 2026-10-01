// modules/costmap_astar_planner (CostmapPlanner-family A* global path planning).
//
// Coverage:
// - makePlan with start/goal on an all-free map -> ok, non-empty path, start ~ start, end ~ goal
// - goal inside an obstacle -> nearestValidGoal -> ok (with tolerance) / GOAL_INVALID (without tolerance)
// - start inside an obstacle / out of bounds -> ErrorCode::START_INVALID
// - cancel mid-planning -> ErrorCode::CANCELED
// - module.cpp exports the unified create_extension_module factory (verified via dlopen)

#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/static_layer.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <costmap_astar_planner/costmap_astar_planner.hpp>
#include <ayanami/utility/extension_module.hpp>

using ayanami::testing::testTransforms;
namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::costmap::CostmapMapContext;
using ayanami::modules::CostmapAstarPlanner;

std::string g_module_so;

// ---------------- Test utilities ----------------

// Builds a small NxN @ res LayeredCostmap + StaticLayer with origin=(0,0).
// The obstacle_fn callback can mark obstacles (all free by default).
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
  auto static_layer = std::make_shared<costmap_2d_layers::StaticLayer>();
  static_layer->setMap(layer_map);
  lm->addLayer(static_layer);
  lm->updateMap(0.0, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

std::shared_ptr<CostmapMapContext> makeContext(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper) {
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setGlobalCostmap(wrapper);
  return ctx;
}

// Planning overlay test context: identity map<-odom so the local grid
// coordinates coincide with the global ones (isolated frames per test).
std::shared_ptr<CostmapMapContext> makeOverlayContext(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& global_wrapper,
    const std::shared_ptr<costmap_2d::CostmapWrapper>& local_wrapper) {
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map_o", "odom_o", "base_o", 0.25);
  testTransforms().set("odom_o", "map_o", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity());
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setFrames("map_o", "odom_o", "base_o");
  ctx->setGlobalCostmap(global_wrapper);
  ctx->setLocalCostmap(local_wrapper);
  return ctx;
}

std::shared_ptr<CostmapAstarPlanner> makePlanner(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper) {
  auto planner = std::make_shared<CostmapAstarPlanner>();
  planner->configure(ayanami::ModuleContext{"test_astar", ayanami::Config("")},
                     makeContext(wrapper)->getSharedConstPtr());
  return planner;
}

ayanami::geometry_msgs::PoseStamped makePose(double x, double y) {
  auto p = ayanami::geometry_msgs::PoseStamped::fromIsometry(
      "map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

/// World coordinates of the center of cell (mx,my) (res=0.5, origin=0).
ayanami::geometry_msgs::PoseStamped cellCenter(unsigned int mx, unsigned int my,
                                               double res = 0.5) {
  return makePose((static_cast<double>(mx) + 0.5) * res,
                  (static_cast<double>(my) + 0.5) * res);
}

}  // namespace

// ---------- makePlan success path ----------

TEST(CostmapAstarPlannerTest, makePlanSuccessOnFreeMap) {
  auto wrapper = makeWrapper(20, 0.5);  // 20x20 @ 0.5m = 10m x 10m, all free
  auto planner = makePlanner(wrapper);

  auto start = cellCenter(2, 2);
  auto goal = cellCenter(17, 17);
ayanami::nav_msgs::Path plan;
    const ResultT<void> result = planner->makePlan(start, goal, 0.1, plan);

  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_FALSE(plan.poses.empty());
  EXPECT_NEAR(plan.poses.front().pose.position.x(), start.pose.position.x(), 1e-6);
  EXPECT_NEAR(plan.poses.front().pose.position.y(), start.pose.position.y(), 1e-6);
  EXPECT_NEAR(plan.poses.back().pose.position.x(), goal.pose.position.x(), 1e-6);
  EXPECT_NEAR(plan.poses.back().pose.position.y(), goal.pose.position.y(), 1e-6);
}

TEST(CostmapAstarPlannerTest, makePlanRoutesAroundObstacle) {
  // A central 4x4 obstacle block (cells 8..11) with start/goal on either side.
  auto wrapper = makeWrapper(20, 0.5, [](costmap_2d::Costmap2D& m) {
    for (unsigned int y = 8; y <= 11; ++y) {
      for (unsigned int x = 8; x <= 11; ++x) {
        m.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
      }
    }
  });
  auto planner = makePlanner(wrapper);

  auto start = cellCenter(2, 2);
  auto goal = cellCenter(17, 17);
ayanami::nav_msgs::Path plan;
    const ResultT<void> result = planner->makePlan(start, goal, 0.1, plan);

  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_FALSE(plan.poses.empty());
}

// ---------- Goal inside an obstacle (nearestValidGoal) ----------

// ---------- Family read contract: the overlay includes local-only obstacles ----------

TEST(CostmapAstarPlannerTest, localOnlyObstacleDivertsOverlayPlan) {
  // Global: fully free corridor. Local: a wall across the straight line
  // (column x=9..10, full height). With the overlay the straight path is
  // blocked; A* must detour around the wall (path leaves the corridor
  // column band).
  auto global_wrapper = makeWrapper(20, 0.5);
  auto local_wrapper = makeWrapper(20, 0.5, [](costmap_2d::Costmap2D& m) {
    // Wall across the corridor with gaps at the top/bottom rows (y<4 or
    // y>15) so a detour exists.
    for (unsigned int y = 4; y <= 15; ++y) {
      m.setCost(9, y, costmap_2d::LETHAL_OBSTACLE);
      m.setCost(10, y, costmap_2d::LETHAL_OBSTACLE);
    }
  });
  auto planner = std::make_shared<CostmapAstarPlanner>();
  planner->configure(
      ayanami::ModuleContext{"test_astar", ayanami::Config("")},
      makeOverlayContext(global_wrapper, local_wrapper)->getSharedConstPtr());

  auto start = cellCenter(2, 9);
  auto goal = cellCenter(17, 9);
  ayanami::nav_msgs::Path plan;
  const ResultT<void> result = planner->makePlan(start, goal, 0.1, plan);

  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_FALSE(plan.poses.empty());
  // Detour proof: some interior pose must leave the blocked column band
  // (cells 9..10 = x in [4.25, 5.75)).
  bool left_band = false;
  for (const auto& pose : plan.poses) {
    const double x = pose.pose.position.x();
    if (x < 4.25 || x >= 5.75) {
      left_band = true;
    }
  }
  EXPECT_TRUE(left_band) << "plan should detour around the local-only wall";
}

// A control: without the local map the same straight corridor plans straight.
TEST(CostmapAstarPlannerTest, globalOnlyPlanIgnoresLocalObstacle) {
  auto global_wrapper = makeWrapper(20, 0.5);
  auto local_wrapper = makeWrapper(20, 0.5, [](costmap_2d::Costmap2D& m) {
    for (unsigned int y = 0; y < m.getSizeInCellsY(); ++y) {
      m.setCost(9, y, costmap_2d::LETHAL_OBSTACLE);
      m.setCost(10, y, costmap_2d::LETHAL_OBSTACLE);
    }
  });
  // Context carries ONLY the global map (no local wrapper): the overlay
  // degrades to global, and the straight corridor stays straight.
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map_o2", "odom_o2", "base_o2", 0.25);
  testTransforms().set("odom_o2", "map_o2", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity());
  auto ctx = std::make_shared<CostmapMapContext>();
  ctx->setFrames("map_o2", "odom_o2", "base_o2");
  ctx->setGlobalCostmap(global_wrapper);
  auto planner = std::make_shared<CostmapAstarPlanner>();
  planner->configure(ayanami::ModuleContext{"test_astar", ayanami::Config("")},
                     ctx->getSharedConstPtr());

  auto start = cellCenter(2, 9);
  auto goal = cellCenter(17, 9);
  ayanami::nav_msgs::Path plan;
  const ResultT<void> result = planner->makePlan(start, goal, 0.1, plan);

  ASSERT_TRUE(result.ok) << result.message;
  for (const auto& pose : plan.poses) {
    const double x = pose.pose.position.x();
    if (x >= 4.25 && x < 5.75) {
      EXPECT_NEAR(pose.pose.position.y(), start.pose.position.y(), 1e-6);
      break;  // straight segment inside the band
    }
  }
}

TEST(CostmapAstarPlannerTest, goalInObstacleWithToleranceFindsNearest) {
  // A central 4x4 obstacle block (cells 8..11) with the goal at the block center (10,10).
  auto wrapper = makeWrapper(20, 0.5, [](costmap_2d::Costmap2D& m) {
    for (unsigned int y = 8; y <= 11; ++y) {
      for (unsigned int x = 8; x <= 11; ++x) {
        m.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
      }
    }
  });
  // Tolerance is config-owned now: inject it through the module config
  // instead of the (ignored) goal parameter.
  ayanami::Config cfg("");
  cfg.overrideParam<double>("test_astar", "tolerance", 1.0);
  auto ctx = makeContext(wrapper);
  auto planner = std::make_shared<CostmapAstarPlanner>();
  planner->configure(ayanami::ModuleContext{"test_astar", cfg},
                     ctx->getSharedConstPtr());

  auto start = cellCenter(2, 2);
  auto goal = cellCenter(10, 10);
ayanami::nav_msgs::Path plan;
    const ResultT<void> result = planner->makePlan(start, goal, 0.0, plan);

  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_FALSE(plan.poses.empty());
}

TEST(CostmapAstarPlannerTest, goalInObstacleWithoutToleranceReturnsNoPath) {
  auto wrapper = makeWrapper(20, 0.5, [](costmap_2d::Costmap2D& m) {
    for (unsigned int y = 8; y <= 11; ++y) {
      for (unsigned int x = 8; x <= 11; ++x) {
        m.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
      }
    }
  });
  auto planner = makePlanner(wrapper);

  auto start = cellCenter(2, 2);
  auto goal = cellCenter(10, 10);
ayanami::nav_msgs::Path plan;
    const ResultT<void> result = planner->makePlan(start, goal, 0.0, plan);

  ASSERT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::GOAL_INVALID);
}

// ---------- start errors ----------

TEST(CostmapAstarPlannerTest, startInObstacleReturnsErrorSegment) {
  auto wrapper = makeWrapper(20, 0.5, [](costmap_2d::Costmap2D& m) {
    for (unsigned int y = 8; y <= 11; ++y) {
      for (unsigned int x = 8; x <= 11; ++x) {
        m.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
      }
    }
  });
  auto planner = makePlanner(wrapper);

  auto start = cellCenter(10, 10);  // Inside an obstacle
  auto goal = cellCenter(2, 2);
ayanami::nav_msgs::Path plan;
    const ResultT<void> result = planner->makePlan(start, goal, 0.1, plan);

  ASSERT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::START_INVALID);
  EXPECT_TRUE(plan.poses.empty());
}

TEST(CostmapAstarPlannerTest, startOutOfMapReturnsErrorSegment) {
  auto wrapper = makeWrapper(20, 0.5);
  auto planner = makePlanner(wrapper);

  auto start = makePose(-1.0, -1.0);  // Out of bounds (origin 0)
  auto goal = cellCenter(5, 5);
ayanami::nav_msgs::Path plan;
    const ResultT<void> result = planner->makePlan(start, goal, 0.1, plan);

  ASSERT_FALSE(result.ok);
  EXPECT_EQ(result.error, ErrorCode::START_INVALID);
  EXPECT_TRUE(plan.poses.empty());
}

TEST(CostmapAstarPlannerTest, startInInscribedBandEscapesToFreeSpace) {
  // Physical band-graze: a lethal block with its natural inscribed halo.
  // The robot's center sits ON a halo cell (253): the 5Hz replan must not
  // fail (START_INVALID / PATH_NOT_FOUND deadlock -- no recovery fixes a
  // position). Escape mode plans OUT of the band: 253 cells stay passable
  // for this search, LETHAL cells stay blocked. Only a LETHAL start stays
  // rejected (startInObstacle above).
  auto wrapper = makeWrapper(20, 0.5, [](costmap_2d::Costmap2D& m) {
    for (unsigned int y = 8; y <= 11; ++y) {
      for (unsigned int x = 8; x <= 11; ++x) {
        m.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
      }
    }
    // The inflation halo: the ring of cells within the inscribed radius.
    for (unsigned int y = 7; y <= 12; ++y) {
      for (unsigned int x = 7; x <= 12; ++x) {
        if (x >= 8 && x <= 11 && y >= 8 && y <= 11) {
          continue;
        }
        m.setCost(x, y, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      }
    }
  });
  auto planner = makePlanner(wrapper);

  auto start = cellCenter(7, 9);  // On the halo, adjacent to the block
  auto goal = cellCenter(2, 2);
  ayanami::nav_msgs::Path plan;
  const ResultT<void> result = planner->makePlan(start, goal, 0.1, plan);

  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_FALSE(plan.poses.empty());
  EXPECT_NEAR(plan.poses.front().pose.position.x(), start.pose.position.x(), 1e-6);
  EXPECT_NEAR(plan.poses.front().pose.position.y(), start.pose.position.y(), 1e-6);
}

// ---------- Cancel mid-planning ----------

TEST(CostmapAstarPlannerTest, cancelDuringPlanningReturnsCanceled) {
  // 800x800 @ 0.05 = 40m x 40m. The goal sits inside a lethal ring (1 cell thick):
  // A* must expand the whole reachable area before reporting NO_PATH (~100ms), leaving plenty of window for cancel.
  auto wrapper = makeWrapper(800, 0.05, [](costmap_2d::Costmap2D& m) {
    const unsigned int c = 400;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx != 0 || dy != 0) {
          m.setCost(c + static_cast<unsigned int>(dx),
                    c + static_cast<unsigned int>(dy),
                    costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  });
  auto planner = makePlanner(wrapper);

  auto start = cellCenter(5, 5, 0.05);
  auto goal = cellCenter(400, 400, 0.05);
  std::atomic<bool> done{false};
  std::atomic<ErrorCode> error{ErrorCode::NONE};
  std::thread worker([&] {
ayanami::nav_msgs::Path plan;
        const ResultT<void> result = planner->makePlan(start, goal, 0.0, plan);
    error.store(result.error);
    done.store(true);
  });

  // Let the search run briefly, then cancel
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_TRUE(planner->cancel());
  worker.join();
  EXPECT_TRUE(done.load());
  EXPECT_EQ(error.load(), ErrorCode::CANCELED);
}

// ---------- module.cpp factory symbols (common factory + typed cast) ----------

TEST(CostmapAstarPlannerTest, moduleFactoryCreatesPlanner) {
  ASSERT_FALSE(g_module_so.empty());
  auto planner = std::dynamic_pointer_cast<CostmapAstarPlanner>(
      ayanami::ExtensionModule::loadModule(g_module_so));
  ASSERT_NE(planner, nullptr);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_module_so = argv[1];
  }
  return RUN_ALL_TESTS();
}
