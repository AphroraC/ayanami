// fused_map_view_test: the prepared cell-space probe path must return
// EXACTLY what the original world-coordinate roundtrip returns.
//
// FusedMapView::prepare() folds mapToWorld -> rigid transform -> worldToMap
// into six per-view coefficients so a grid-walk probe is two dot products
// in cell space (the A* inner loop and the MPPI batch scoring probe it
// hundreds of times per expansion / rollout). The fold reassociates the
// floating-point arithmetic, so this test locks result parity over
// randomized rigid transforms, mixed resolutions and both in-window /
// out-of-window probe positions -- plus the deployment-aligned identity
// case (identity transform, cell-aligned origins), where quotients land
// exactly on cell-center halves and floor flips would show up first.

#include <ayanami/costmap/core/fused_map_view.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

namespace {

using ayanami::costmap::CostmapSnapshot;
using ayanami::costmap::FusedMapView;

CostmapSnapshot makeSnapshot(unsigned int sx, unsigned int sy, double res,
                             double ox, double oy, std::uint64_t generation,
                             std::mt19937& rng) {
  CostmapSnapshot snapshot;
  snapshot.size_x = sx;
  snapshot.size_y = sy;
  snapshot.resolution = res;
  snapshot.origin_x = ox;
  snapshot.origin_y = oy;
  snapshot.generation = generation;
  snapshot.frame = "frame";
  snapshot.costs.resize(static_cast<std::size_t>(sx) * sy);
  std::uniform_int_distribution<int> cost_dist(0, 255);
  for (auto& cost : snapshot.costs) {
    cost = static_cast<unsigned char>(cost_dist(rng));
  }
  return snapshot;
}

void expectViewParity(const FusedMapView& slow, const FusedMapView& fast) {
  ASSERT_EQ(slow.global, fast.global);
  ASSERT_EQ(slow.local, fast.local);
  for (unsigned int my = 0; my < slow.global->size_y; ++my) {
    for (unsigned int mx = 0; mx < slow.global->size_x; ++mx) {
      const unsigned char slow_cost = slow.costAtCell(mx, my);
      const unsigned char fast_cost = fast.costAtCell(mx, my);
      ASSERT_EQ(slow_cost, fast_cost)
          << "cost mismatch at (" << mx << ", " << my << ")";
      ASSERT_EQ(slow.blockedAtCell(mx, my), fast.blockedAtCell(mx, my))
          << "blocked mismatch at (" << mx << ", " << my << ")";
    }
  }
}

FusedMapView wireView(const CostmapSnapshot* global,
                      const CostmapSnapshot* local,
                      const Eigen::Isometry3d& local_from_global) {
  FusedMapView view;
  view.global = global;
  view.local = local;
  view.local_from_global = local_from_global;
  return view;
}

TEST(FusedMapView, preparedCellProbeMatchesWorldRoundtripRandomized) {
  std::mt19937 rng(20261006);
  std::uniform_real_distribution<double> yaw_dist(-M_PI, M_PI);
  std::uniform_real_distribution<double> trans_dist(-2.0, 2.0);
  // Mixed local resolutions (0.05 = deployment; 0.1 = coarser overlay) and
  // origins chosen so parts of the global grid fall outside the local
  // window (the bounds path) and parts inside.
  for (int iter = 0; iter < 60; ++iter) {
    CostmapSnapshot global =
        makeSnapshot(36, 28, 0.05, 1.0, -0.5, 7, rng);
    const double local_res = (iter % 2 == 0) ? 0.05 : 0.1;
    CostmapSnapshot local = makeSnapshot(24, 18, local_res, -0.3, 0.7,
                                         100 + static_cast<std::uint64_t>(iter), rng);
    Eigen::Isometry3d local_from_global = Eigen::Isometry3d::Identity();
    local_from_global.linear() =
        Eigen::AngleAxisd(yaw_dist(rng), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    local_from_global.translation() =
        Eigen::Vector3d(trans_dist(rng), trans_dist(rng), 0.0);

    FusedMapView slow = wireView(&global, &local, local_from_global);
    FusedMapView fast = wireView(&global, &local, local_from_global);
    fast.prepare();
    ASSERT_TRUE(fast.valid());
    expectViewParity(slow, fast);
  }
}

TEST(FusedMapView, preparedCellProbeMatchesWorldRoundtripAlignedIdentity) {
  std::mt19937 rng(42);
  // The simulation-shaped case: identity transform, equal resolutions,
  // cell-aligned origins (their difference is an exact multiple of the
  // resolution). Every global cell center lands exactly on a local cell
  // center -- the +0.5 offsets and the folded coefficients must agree.
  CostmapSnapshot global = makeSnapshot(30, 30, 0.05, 0.0, 0.0, 1, rng);
  CostmapSnapshot local = makeSnapshot(20, 20, 0.05, 0.4, -0.65, 2, rng);
  FusedMapView slow = wireView(&global, &local, Eigen::Isometry3d::Identity());
  FusedMapView fast = wireView(&global, &local, Eigen::Isometry3d::Identity());
  fast.prepare();
  expectViewParity(slow, fast);
}

TEST(FusedMapView, unpreparedViewKeepsTheOriginalSemantics) {
  // No local overlay: pure global lookup, prepare() is a harmless no-op.
  std::mt19937 rng(7);
  CostmapSnapshot global = makeSnapshot(10, 10, 0.05, 0.0, 0.0, 1, rng);
  FusedMapView view = wireView(&global, nullptr, Eigen::Isometry3d::Identity());
  view.prepare();
  for (unsigned int my = 0; my < global.size_y; ++my) {
    for (unsigned int mx = 0; mx < global.size_x; ++mx) {
      EXPECT_EQ(view.costAtCell(mx, my), global.getCost(mx, my));
    }
  }
}

}  // namespace
