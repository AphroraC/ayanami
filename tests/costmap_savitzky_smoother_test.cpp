// modules/costmap_savitzky_smoother (CostmapSmoother-family 7-point
// Savitzky-Golay smoothing).
//
// Coverage:
// - an L-shaped zigzag input (10 poses) becomes smoother after smoothPath (the sum of first-difference lengths decreases),
// the path size does not shrink, and both endpoints stay fixed
// - max_duration=0.001s succeeds immediately (may skip some or all points)
// - empty path / fewer than 7 points -> SUCCESS, out=in
// - collision check (always on): if the smoothed path crosses an obstacle cell -> roll back to the original path,
// msg contains "collision"
// - module.cpp exports the unified create_extension_module factory (verified via dlopen)

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/static_layer.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <costmap_savitzky_smoother/costmap_savitzky_smoother.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::ResultT;
using ayanami::costmap::CostmapMapContext;
using ayanami::modules::CostmapSavitzkySmoother;
using ayanami::nav_msgs::Path;

std::string g_module_so;

// ---------------- Test utilities ----------------

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
  // Family read contract: the smoother reads the LOCAL map -- the test grid
  // is installed as the local map (a global copy keeps the context complete).
  ctx->setGlobalCostmap(wrapper);
  ctx->setLocalCostmap(wrapper);
  return ctx;
}

std::shared_ptr<CostmapSavitzkySmoother> makeSmoother(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper) {
  auto smoother = std::make_shared<CostmapSavitzkySmoother>();
  smoother->configure(
      ayanami::ModuleContext{"test_savitzky", ayanami::Config("")},
      makeContext(wrapper)->getSharedConstPtr());
  return smoother;
}

ayanami::geometry_msgs::PoseStamped makePose(double x, double y) {
  auto p = ayanami::geometry_msgs::PoseStamped::fromIsometry(
      "map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

/// L-shaped zigzag path (10 poses): 5 along +x then 5 along +y.
Path makeLPath() {
  Path p;
  p.header.frame_id = "map";
  for (int i = 0; i <= 4; ++i) {
    p.poses.push_back(makePose(static_cast<double>(i), 0.0));
  }
  for (int j = 1; j <= 5; ++j) {
    p.poses.push_back(makePose(5.0, static_cast<double>(j)));
  }
  return p;
}

/// Sum of absolute first differences (Euclidean steps): the smaller, the smoother.
double pathLengthSum(const Path& p) {
  double sum = 0.0;
  for (std::size_t i = 1; i < p.poses.size(); ++i) {
    sum += (p.poses[i].pose.position - p.poses[i - 1].pose.position)
               .norm();
  }
  return sum;
}

bool posesEqual(const Path& a, const Path& b) {
  if (a.poses.size() != b.poses.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.poses.size(); ++i) {
    if ((a.poses[i].pose.position - b.poses[i].pose.position).norm() >
        1e-9) {
      return false;
    }
  }
  return true;
}

}  // namespace

// ---------- Smoothing effectiveness ----------

TEST(CostmapSavitzkySmootherTest, smoothsLShapedPath) {
  auto wrapper = makeWrapper(20, 0.5);
  auto smoother = makeSmoother(wrapper);

  const Path in = makeLPath();
  Path out = in;
  const ResultT<void> result = smoother->smoothPath(out, 10.0);

  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(out.poses.size(), in.poses.size());
  EXPECT_LT(pathLengthSum(out), pathLengthSum(in));
  // Both endpoints stay fixed.
  EXPECT_NEAR(out.poses.front().pose.position.x(),
              in.poses.front().pose.position.x(), 1e-9);
  EXPECT_NEAR(out.poses.front().pose.position.y(),
              in.poses.front().pose.position.y(), 1e-9);
  EXPECT_NEAR(out.poses.back().pose.position.x(),
              in.poses.back().pose.position.x(), 1e-9);
  EXPECT_NEAR(out.poses.back().pose.position.y(),
              in.poses.back().pose.position.y(), 1e-9);
}

TEST(CostmapSavitzkySmootherTest, tinyMaxDurationReturnsSuccess) {
  auto wrapper = makeWrapper(20, 0.5);
  auto smoother = makeSmoother(wrapper);

  const Path in = makeLPath();
  Path out = in;
  const ResultT<void> result = smoother->smoothPath(out, 0.001);

  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_GE(out.poses.size(), in.poses.size());
  EXPECT_FALSE(out.poses.empty());
}

// ---------- Empty / short path ----------

TEST(CostmapSavitzkySmootherTest, emptyPathReturnsSuccessWithEmptyOut) {
  auto wrapper = makeWrapper(20, 0.5);
  auto smoother = makeSmoother(wrapper);

  Path in;  // Empty path: no window to smooth
  const ResultT<void> result = smoother->smoothPath(in, 10.0);

  ASSERT_TRUE(result.ok) << result.message;
  // In-place: an empty path stays empty; no invented points.
  EXPECT_TRUE(in.poses.empty());
  EXPECT_TRUE(in.poses.empty());
}

TEST(CostmapSavitzkySmootherTest, shortPathReturnsSuccessWithSamePath) {
  auto wrapper = makeWrapper(20, 0.5);
  auto smoother = makeSmoother(wrapper);

  Path in;  // 3 < 7 points
  for (int i = 0; i < 3; ++i) {
    in.poses.push_back(makePose(static_cast<double>(i), 0.0));
  }
  const Path original = in;
  const ResultT<void> result = smoother->smoothPath(in, 10.0);

  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_TRUE(posesEqual(in, original));
}

// ---------- Collision-check rollback ----------

TEST(CostmapSavitzkySmootherTest, collisionCheckRollsBackToOriginal) {
  auto wrapper = makeWrapper(20, 0.5);
  auto smoother = makeSmoother(wrapper);
  costmap_2d::Costmap2D* cm = wrapper->get().getCostmap();
  ASSERT_NE(cm, nullptr);

  const Path in = makeLPath();
  Path smoothed = in;
  ASSERT_TRUE(smoother->smoothPath(smoothed, 10.0).ok);
  ASSERT_FALSE(posesEqual(smoothed, in));  // Smoothing did move the interior points

  // Find a free cell that the smoothed path crosses but the original path does not, and set it lethal.
  std::set<std::pair<unsigned int, unsigned int>> orig_cells;
  for (const auto& p : in.poses) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (cm->worldToMap(p.pose.position.x(), p.pose.position.y(), mx,
                       my)) {
      orig_cells.insert({mx, my});
    }
  }

  bool found = false;
  unsigned int cell_x = 0;
  unsigned int cell_y = 0;
  for (std::size_t i = 1; i + 1 < smoothed.poses.size(); ++i) {
    unsigned int mx = 0;
    unsigned int my = 0;
    const Eigen::Vector3d& t = smoothed.poses[i].pose.position;
    if (!cm->worldToMap(t.x(), t.y(), mx, my)) {
      continue;
    }
    const auto cell = std::make_pair(mx, my);
    if (orig_cells.count(cell) == 0 &&
        cm->getCost(mx, my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      cell_x = mx;
      cell_y = my;
      found = true;
      break;
    }
  }
  ASSERT_TRUE(found) << "expected a smoothed corner-cut cell to place obstacle";
  // Single-writer path: drop the obstacle directly onto the master grid under a unique lock (the test only edits the grid,
  // not the layers); the later smoothPath sees it via the shared-lock direct read.
  {
    std::unique_lock<std::shared_mutex> lock(wrapper->mutex());
    wrapper->get().getCostmap()->setCost(cell_x, cell_y,
                                         costmap_2d::LETHAL_OBSTACLE);
  }

  // Re-smooth + collision check -> in-place rollback to the original path (the locked direct read includes the obstacle).
  Path out = in;
  const ResultT<void> result = smoother->smoothPath(out, 10.0);

  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_TRUE(posesEqual(out, in));  // Rolled back to the original
  EXPECT_NE(result.message.find("collision"), std::string::npos);
}

// ---------- module.cpp factory symbols (common factory + typed cast) ----------

TEST(CostmapSavitzkySmootherTest, moduleFactoryCreatesSmoother) {
  ASSERT_FALSE(g_module_so.empty());
  auto smoother = std::dynamic_pointer_cast<CostmapSavitzkySmoother>(
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
