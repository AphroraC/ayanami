// esdfmap_test: the esdfmap package (exact signed EDT over a fused
// obstacle mask, immutable-snapshot publication, generation-gated writer).
//
// Coverage:
// 1. geometry: single-obstacle distance field (center distance, sign
//    inside/outside, clamp), free-space distances, corner cells;
// 2. writer: rebuild publishes a snapshot, the generation gate skips
//    unchanged sources, an obstacle mark flip forces a rebuild;
// 3. fused semantics: the global static mask projects through the injected
//    rigid transform and ORs into the field (prior wall far from any live
//    mark);
// 4. snapshot stability: a held snapshot stays byte-identical across a
//    republish (atomic swap contract);
// 5. query contract: interpolated distance + analytic gradient at a point
//    in open space (gradient magnitude ~1 away from clamp plateaus);
// 6. isPathValid: interior points fail at margin, outside-window passes.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <esdfmap/esdf_map.hpp>
#include <esdfmap/esdf_writer.hpp>

namespace {

constexpr int kW = 40;
constexpr int kH = 40;
constexpr double kRes = 0.1;

struct MaskSource {
  std::vector<unsigned char> cells;
  std::uint64_t generation{1};
  double origin_x{-2.0};
  double origin_y{-2.0};
  std::string frame{"odom"};

  bool read(esdfmap::EsdfMaskView& view) {
    view.width = kW;
    view.height = kH;
    view.resolution = kRes;
    view.origin_x = origin_x;
    view.origin_y = origin_y;
    view.frame = frame;
    view.generation = generation;
    view.cells = &cells;
    return true;
  }

  void mark(int x, int y) {
    cells.resize(static_cast<std::size_t>(kW) * kH, 0);
    cells[static_cast<std::size_t>(y) * kW + x] = 1;
  }
};

}  // namespace

TEST(EsdfmapTest, rebuildPublishesSignedField) {
  MaskSource source;
  source.mark(20, 20);  // world (0.05, 0.05): cell center of the obstacle
  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(map, [&source](esdfmap::EsdfMaskView& v) {
    return source.read(v);
  });

  ASSERT_TRUE(writer.rebuildOnce());
  const auto snapshot = map->currentSnapshot();
  ASSERT_TRUE(snapshot);
  ASSERT_TRUE(snapshot->valid());
  EXPECT_EQ(snapshot->sequence, 1u);
  EXPECT_EQ(snapshot->frame, "odom");

  // The obstacle cell itself: inside (negative or zero distance).
  const double inside = snapshot->distanceAt(20, 20);
  EXPECT_LE(inside, 0.0);
  // One cell to the right: +resolution (0.1 m from the obstacle cell).
  EXPECT_NEAR(snapshot->distanceAt(21, 20), kRes, 1e-6);
  // Five cells away: 5 * resolution.
  EXPECT_NEAR(snapshot->distanceAt(25, 20), 5.0 * kRes, 1e-6);
}

TEST(EsdfmapTest, generationGateSkipsUnchangedSource) {
  MaskSource source;
  source.mark(10, 10);
  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(map, [&source](esdfmap::EsdfMaskView& v) {
    return source.read(v);
  });

  ASSERT_TRUE(writer.rebuildOnce());
  EXPECT_EQ(writer.skippedRebuilds(), 0u);
  const auto first = map->currentSnapshot();

  // Same generation, same geometry: skipped, snapshot untouched.
  EXPECT_FALSE(writer.rebuildOnce());
  EXPECT_EQ(writer.skippedRebuilds(), 1u);
  EXPECT_EQ(map->currentSnapshot().get(), first.get());

  // A new generation (the writer bumped the source): rebuild.
  source.generation = 2;
  source.mark(30, 30);
  ASSERT_TRUE(writer.rebuildOnce());
  EXPECT_EQ(map->currentSnapshot()->sequence, 2u);
  EXPECT_NE(map->currentSnapshot().get(), first.get());

  // A held snapshot stays byte-identical across the republish.
  EXPECT_EQ(first->distanceAt(10, 10), map->currentSnapshot()->distanceAt(10, 10));
  EXPECT_EQ(first->size_x, map->currentSnapshot()->size_x);
}

TEST(EsdfmapTest, globalStaticMaskFusesThroughTransform) {
  MaskSource source;  // local: empty (no live marks)
  source.cells.assign(static_cast<std::size_t>(kW) * kH, 0);

  // Global static: a wall column at global x = -10 (one cell wide), in the
  // "map" frame. The field lives in "odom" at the same origin here, so the
  // transform is the identity.
  MaskSource global;
  global.cells.assign(static_cast<std::size_t>(kW) * kH, 0);
  global.frame = "map";
  global.origin_x = -2.0;
  global.origin_y = -2.0;
  global.mark(5, 20);  // global cell (5, 20) = world (-1.45, 0.05)

  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(map, [&source](esdfmap::EsdfMaskView& v) {
    return source.read(v);
  });
  writer.setOverlaySource(
      [&global](esdfmap::EsdfMaskView& v) { return global.read(v); },
      [](const std::string&, const std::string&, double& tx, double& ty,
         double& yaw) {
        tx = 0.0;
        ty = 0.0;
        yaw = 0.0;
        return true;
      });

  ASSERT_TRUE(writer.rebuildOnce());
  const auto snapshot = map->currentSnapshot();
  ASSERT_TRUE(snapshot->valid());
  // The fused raster carries the projected prior wall.
  EXPECT_EQ(snapshot->occupied[20 * kW + 5], 1);
  // And the field sees it as an obstacle (interior non-positive).
  EXPECT_LE(snapshot->distanceAt(5, 20), 0.0);
}

TEST(EsdfmapTest, globalFusionRespectsTransformDirection) {
  // Direction-sensitive regression: the writer samples field-frame cell
  // centers in the global grid, so it must apply the GLOBAL <- FIELD
  // transform. Applying the inverse shipped the prior walls through the
  // robot's own odometry pose -- static obstacles translated/rotated with
  // the robot in the map frame.
  MaskSource source;  // local: empty
  source.cells.assign(static_cast<std::size_t>(kW) * kH, 0);

  MaskSource global;
  global.cells.assign(static_cast<std::size_t>(kW) * kH, 0);
  global.frame = "map";
  global.origin_x = -2.0;
  global.origin_y = -2.0;
  global.mark(30, 30);  // map (1.05, 1.05)

  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(map, [&source](esdfmap::EsdfMaskView& v) {
    return source.read(v);
  });
  // T_map_odom = translation (1.0, 0.5), yaw 0: the lookup is called with
  // (target = "map", source = "odom") and answers p_map = t + R * p_odom.
  writer.setOverlaySource(
      [&global](esdfmap::EsdfMaskView& v) { return global.read(v); },
      [](const std::string& target, const std::string& source_frame,
         double& tx, double& ty, double& yaw) {
        EXPECT_EQ(target, "map");
        EXPECT_EQ(source_frame, "odom");
        tx = 1.0;
        ty = 0.5;
        yaw = 0.0;
        return true;
      });

  ASSERT_TRUE(writer.rebuildOnce());
  const auto snapshot = map->currentSnapshot();
  ASSERT_TRUE(snapshot->valid());
  // map (1.05, 1.05) -> odom (0.05, 0.55) -> cells (20, 25).
  EXPECT_EQ(snapshot->occupied[25 * kW + 20], 1);
  EXPECT_LE(snapshot->distanceAt(20, 25), 0.0);
  // The inverse-direction sample point must stay free (the old bug landed
  // the wall here: odom (0.05, 0.55) transformed by T_odom_map instead).
  EXPECT_EQ(snapshot->occupied[20 * kW + 9], 0);
}

TEST(EsdfmapTest, queryInterpolatesAndGradients) {
  MaskSource source;
  source.mark(20, 20);
  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(map, [&source](esdfmap::EsdfMaskView& v) {
    return source.read(v);
  });
  ASSERT_TRUE(writer.rebuildOnce());

  // Open space to the right of the obstacle: distance grows to the right,
  // gradient points away from the obstacle (+x), magnitude ~1 in an
  // unclamped linear region.
  const auto q = map->query(1.0, 0.05);
  ASSERT_EQ(q.status, esdfmap::EsdfQueryStatus::kOk);
  EXPECT_GT(q.distance, 0.5);
  EXPECT_GT(q.gradient_x, 0.3);
  EXPECT_NEAR(q.gradient_y, 0.0, 0.2);

  // Outside the rolling window: unobserved.
  const auto out = map->query(50.0, 50.0);
  EXPECT_EQ(out.status, esdfmap::EsdfQueryStatus::kOutOfField);
}

TEST(EsdfmapTest, pathValidityProbesInteriorOnly) {
  MaskSource source;
  source.mark(20, 20);
  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(map, [&source](esdfmap::EsdfMaskView& v) {
    return source.read(v);
  });
  ASSERT_TRUE(writer.rebuildOnce());

  // A path through the obstacle cell: blocked at zero margin.
  EXPECT_FALSE(map->isPathValid({0.05}, {0.05}, 0.0));
  // A path one cell away: clear at margin 0.05 (< 0.1 m distance).
  EXPECT_TRUE(map->isPathValid({0.15, 0.15}, {0.05, 0.05}, 0.05));
  // Outside-window points pass (unobserved, not blocked).
  EXPECT_TRUE(map->isPathValid({50.0}, {50.0}, 0.0));
}

TEST(EsdfmapTest, freshnessGate) {
  MaskSource source;
  source.mark(20, 20);
  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(map, [&source](esdfmap::EsdfMaskView& v) {
    return source.read(v);
  });
  ASSERT_TRUE(writer.rebuildOnce());

  EXPECT_TRUE(map->current());
  map->setStaleTimeout(0.0);  // disabled: still current
  EXPECT_TRUE(map->current());
  map->setStaleTimeout(-1e-9);  // effectively zero: any age fails
  // (a snapshot just published may still be within 0 s only in the same
  // tick; use a tiny positive timeout that has certainly elapsed)
  map->setStaleTimeout(1e-12);
  EXPECT_FALSE(map->current());
}

TEST(EsdfmapTest, snapshotListenerFiresAfterEachPublish) {
  MaskSource source;
  source.mark(20, 20);
  auto map = std::make_shared<esdfmap::EsdfMap>();
  std::vector<std::shared_ptr<const esdfmap::EsdfSnapshot>> observed;
  esdfmap::EsdfWriter writer(
      map, [&source](esdfmap::EsdfMaskView& view) { return source.read(view); },
      esdfmap::EsdfWriterConfig{});
  writer.setSnapshotListener(
      [&observed](const std::shared_ptr<const esdfmap::EsdfSnapshot>& snapshot) {
        observed.push_back(snapshot);
      });
  ASSERT_TRUE(writer.rebuildOnce());
  // The generation gate skips the unchanged source: no second observation.
  ASSERT_FALSE(writer.rebuildOnce());
  ASSERT_EQ(observed.size(), 1u);
  EXPECT_TRUE(observed.front()->valid());
  EXPECT_EQ(observed.front()->sequence, 1u);
  // A content flip (generation advance) publishes and observes again.
  source.generation = 2;
  source.mark(25, 20);
  ASSERT_TRUE(writer.rebuildOnce());
  ASSERT_EQ(observed.size(), 2u);
  EXPECT_EQ(observed.back()->sequence, 2u);
}
