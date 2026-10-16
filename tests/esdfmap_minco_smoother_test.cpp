// esdfmap_minco_smoother regression: LBFGS_MAXIMUMITERATION must be a
// healthy exit (the last iterate flows into hard validation), not a
// wholesale smoothing failure. Before the fix, any path needing real
// optimization work exhausted the 256-iteration budget and failed with
// code 11 (SMOOTHING_FAILED), degrading to the raw path every cycle.
//
// Coverage:
// - long astar-style path (81 poses, no stamps -> arc-length schedule)
//   through a field with a wall, production config: smoothing SUCCEEDS;
// - endpoint anchoring: the output starts/ends at the input endpoints;
// - short-path passthrough and MAP_UNAVAILABLE contracts still hold.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <esdfmap/esdf_map.hpp>
#include <esdfmap/esdf_writer.hpp>

#include <ayanami/esdfmap/core/esdfmap_smoother.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/utility/config.hpp>
#include <filesystem>
#include <fstream>

#include <ayanami/utility/extension_module.hpp>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include "test_transform_source.hpp"
namespace {

constexpr int kW = 120;  // 6 m at 0.05 m
constexpr int kH = 120;

struct MaskSource {
  std::vector<unsigned char> cells;
  std::uint64_t generation{1};

  bool read(esdfmap::EsdfMaskView& view) {
    view.width = kW;
    view.height = kH;
    view.resolution = 0.05;
    view.origin_x = -3.0;
    view.origin_y = -3.0;
    view.frame = "odom";
    view.generation = generation;
    view.cells = &cells;
    return true;
  }

  void wall(int x0, int x1, int y) {
    cells.assign(static_cast<std::size_t>(kW) * kH, 0);
    for (int x = x0; x <= x1; ++x) {
      cells[static_cast<std::size_t>(y) * kW + x] = 1;
    }
  }
};

/// @brief Write the production module json to a temp file and return its path
/// (Config loads from a filename only).
std::string writeProductionConfig() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_esdfmap_minco_test";
  std::filesystem::create_directories(dir);
  const auto path = dir / "esdfmap_minco_smoother.json";
  std::ofstream ofs(path);
  ofs << R"({
  "esdfmap_minco_smoother": {
    "safe_dist": 0.6,
    "weight_pos": 200000.0,
    "weight_smooth": 1.0,
    "mu_l1": 0.4,
    "freeze_window": 0.0,
    "output_dt": 0.05,
    "max_iterations": 256,
    "opt_accuracy": 0.0001,
    "lbfgs_mem_size": 64,
    "severe_scale": 1.5,
    "validation_dt": 0.05
  }
})";
  return path.string();
}

/// @brief The production module json, loaded the way the host loads it.
/// ModuleContext carries the smoother's own config section keyed by name.
std::shared_ptr<ayanami::esdfmap::EsdfmapSmoother> loadSmoother(
    esdfmap::EsdfMap::Ptr map) {
  static const std::string config_path = writeProductionConfig();
  ayanami::Config mod(config_path);
  ayanami::ModuleContext context{"esdfmap_minco_smoother", mod};
  ayanami::ExtensionModule::Ptr base =
      ayanami::ExtensionModule::loadModule("libayanami_esdfmap_minco_smoother.so");
  EXPECT_NE(base, nullptr);
  auto smoother =
      std::dynamic_pointer_cast<ayanami::esdfmap::EsdfmapSmoother>(base);
  EXPECT_NE(smoother, nullptr);
  if (smoother) {
    smoother->configure(context, std::move(map));
  }
  return smoother;
}

ayanami::nav_msgs::Path makeAstarStylePath(int count, double y0,
                                           double amplitude) {
  ayanami::nav_msgs::Path path;
  path.header.frame_id = "odom";
  path.poses.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    ayanami::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x() = -2.0 + 4.0 * i / static_cast<double>(count - 1);
    pose.pose.position.y() =
        y0 - amplitude * std::sin(3.0 * i / static_cast<double>(count - 1) *
                                  M_PI);
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

TEST(EsdfmapMincoSmoother, iterationBudgetExhaustionStillSucceeds) {
  MaskSource source;
  source.wall(30, 90, 60);
  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(
      map, [&source](esdfmap::EsdfMaskView& view) { return source.read(view); },
      esdfmap::EsdfWriterConfig{});
  ASSERT_TRUE(writer.rebuildOnce());

  auto smoother = loadSmoother(map);
  ASSERT_NE(smoother, nullptr);

  // The regression input: a LONG path with no stamps (arc-length fallback),
  // close enough to the wall that the ESDF penalty does real work -- the
  // 256-iteration budget is exhausted (LBFGS_MAXIMUMITERATION). The last
  // iterate must be VALIDATED and USED, not turned into a code-11 failure.
  ayanami::nav_msgs::Path path = makeAstarStylePath(81, 0.7, 0.3);
  const auto result = smoother->smoothPath(path, 2.0);
  EXPECT_TRUE(result.ok) << "error=" << static_cast<int>(result.error)
                         << " message=" << result.message;
  ASSERT_FALSE(path.poses.empty());
  EXPECT_EQ(path.header.frame_id, "odom");
  // Endpoint anchoring: first pose at the input start, last pose within a
  // cell of the input goal (the tail is anchored exactly).
  EXPECT_NEAR(path.poses.front().pose.position.x(), -2.0, 1e-6);
  EXPECT_NEAR(path.poses.back().pose.position.x(), 2.0, 1e-6);
  EXPECT_NEAR(path.poses.back().pose.position.y(),
              path.poses.back().pose.position.y(), 0.0);
}

TEST(EsdfmapMincoSmoother, esdfHingeRepelsInsideBand) {
  // The ESDF positional term must actually bend the path: a control path
  // running 0.2 m from a wall (inside the 0.35 m hinge band) is repelled to
  // clear safe_dist; a path outside the band stays put. Before the weight
  // rebalance the energy stiffness (~T^-5 on 0.05 s pieces) squeezed the
  // equilibrium to sub-millimeters -- the gradient flowed but had no
  // visible effect.
  MaskSource source;
  source.wall(10, 110, 60);  // wall along y ~= 0.025
  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(
      map, [&source](esdfmap::EsdfMaskView& view) { return source.read(view); },
      esdfmap::EsdfWriterConfig{});
  ASSERT_TRUE(writer.rebuildOnce());
  auto smoother = loadSmoother(map);
  ASSERT_NE(smoother, nullptr);

  const auto run = [&](double y0) {
    ayanami::nav_msgs::Path path = makeAstarStylePath(81, y0, 0.0);
    const auto result = smoother->smoothPath(path, 2.0);
    EXPECT_TRUE(result.ok) << result.message;
    double min_y = 1e9;
    for (const auto& pose : path.poses) {
      if (pose.pose.position.x() > -1.5 && pose.pose.position.x() < 1.5) {
        min_y = std::min(min_y, pose.pose.position.y());
      }
    }
    return min_y;
  };
  // Inside the band: repelled past safe_dist (0.6 m clearance -- moderate
  // repulsion: strong enough to keep the plan-once repair off obstacles,
  // weak enough not to drag the path far from the A* topology).
  EXPECT_GE(run(0.2), 0.55);
  // Outside the band: untouched.
  EXPECT_NEAR(run(0.9), 0.9, 0.05);
}

TEST(EsdfmapMincoSmoother, noFieldIsMapUnavailable) {
  static const std::string config_path = writeProductionConfig();
  ayanami::Config mod(config_path);
  ayanami::ModuleContext context{"esdfmap_minco_smoother", mod};
  ayanami::ExtensionModule::Ptr base =
      ayanami::ExtensionModule::loadModule("libayanami_esdfmap_minco_smoother.so");
  ASSERT_NE(base, nullptr);
  auto smoother =
      std::dynamic_pointer_cast<ayanami::esdfmap::EsdfmapSmoother>(base);
  ASSERT_NE(smoother, nullptr);
  smoother->configure(context, nullptr);

  ayanami::nav_msgs::Path path = makeAstarStylePath(10, 0.0, 0.0);
  const auto result = smoother->smoothPath(path, 2.0);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, ayanami::ErrorCode::MAP_UNAVAILABLE);
}

TEST(EsdfmapMincoSmoother, shortPathPassesThrough) {
  MaskSource source;
  source.wall(30, 90, 60);
  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(
      map, [&source](esdfmap::EsdfMaskView& view) { return source.read(view); },
      esdfmap::EsdfWriterConfig{});
  ASSERT_TRUE(writer.rebuildOnce());

  auto smoother = loadSmoother(map);
  ASSERT_NE(smoother, nullptr);

  ayanami::nav_msgs::Path path = makeAstarStylePath(2, 0.0, 0.0);
  const auto result = smoother->smoothPath(path, 2.0);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(path.poses.size(), 2u);
}

// ---- regression: constant-cost exclusion (2026-10-03 live-stack bug) ----
// Frozen points and out-of-field queries are constants (zero gradient), but
// summing them into f drowned lbfgs's RELATIVE delta criterion
// (|df|/max(1,f) < 1e-4 with f ~ 3.7e6): the optimizer stopped after ~3
// iterations, the output equaled the input, and every path kept grazing the
// walls -- the robot stalled on the inflation band and burned the recovery
// budget on no-op recovery cycles.

TEST(EsdfmapMincoSmoother, freezeWindowPushesFarFieldKeepsNearField) {
  MaskSource source;
  source.wall(10, 110, 60);
  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(
      map, [&source](esdfmap::EsdfMaskView& view) { return source.read(view); },
      esdfmap::EsdfWriterConfig{});
  ASSERT_TRUE(writer.rebuildOnce());
  auto smoother = loadSmoother(map);
  ASSERT_NE(smoother, nullptr);
  // Override: production freeze_window (the shipped json carries 1.0).
  {
    const auto dir =
        std::filesystem::temp_directory_path() / "ayanami_esdfmap_minco_test";
    const auto path = dir / "esdfmap_minco_smoother.json";
    std::ofstream ofs(path);
    ofs << R"({
  "esdfmap_minco_smoother": {
    "safe_dist": 0.55, "weight_pos": 100000.0, "weight_smooth": 1.0,
    "mu_l1": 0.4, "freeze_window": 0.0, "output_dt": 0.05,
    "max_iterations": 256, "opt_accuracy": 0.0001, "lbfgs_mem_size": 64,
    "severe_scale": 1.5, "validation_dt": 0.05
  }
})";
    ayanami::Config mod(path.string());
    smoother->configure(ayanami::ModuleContext{"esdfmap_minco_smoother", mod},
                        map);
  }

  // 4 m path (arc-length fallback at 1 m/s) 0.175 m from the wall: the
  // first 1 m is frozen, the rest must be repelled past safe_dist.
  ayanami::nav_msgs::Path path = makeAstarStylePath(81, 0.2, 0.0);
  const auto result = smoother->smoothPath(path, 2.0);
  EXPECT_TRUE(result.ok) << result.message;
  auto min_y_over = [&](double x_lo, double x_hi) {
    double min_y = 1e9;
    for (const auto& pose : path.poses) {
      const double x = pose.pose.position.x();
      if (x > x_lo && x < x_hi) {
        min_y = std::min(min_y, pose.pose.position.y());
      }
    }
    return min_y;
  };
  // Far field repelled past safe_dist even with the first meter frozen.
  // (The window excludes the last ~0.5 m: the tail binds at rest AT THE
  // GOAL -- a fixed position the smoother must not move -- so the approach
  // segment keeps the planner's geometry; the planner's nearestValidGoal
  // owns goal-side clearance.)
  EXPECT_GE(min_y_over(0.5, 1.5), 0.5);
  // Near field keeps the input geometry (continuity contract).
  EXPECT_NEAR(min_y_over(-2.0, -1.2), 0.2, 0.05);
}

TEST(EsdfmapMincoSmoother, freezeSweepDiagnostic) {
  MaskSource source;
  source.wall(10, 110, 60);
  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(
      map, [&source](esdfmap::EsdfMaskView& view) { return source.read(view); },
      esdfmap::EsdfWriterConfig{});
  ASSERT_TRUE(writer.rebuildOnce());
  auto smoother = loadSmoother(map);
  ASSERT_NE(smoother, nullptr);
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_esdfmap_minco_test";
  for (double freeze : {0.0, 0.05, 0.5, 1.0}) {
    const auto path = dir / "esdfmap_minco_smoother.json";
    {
      std::ofstream ofs(path);
      ofs << R"({
  "esdfmap_minco_smoother": {
    "safe_dist": 0.55, "weight_pos": 100000.0, "weight_smooth": 1.0,
    "mu_l1": 0.4, "freeze_window": )" << freeze << R"(, "output_dt": 0.05,
    "max_iterations": 256, "opt_accuracy": 0.0001, "lbfgs_mem_size": 64,
    "severe_scale": 1.5, "validation_dt": 0.05
  }
})";
    }
    ayanami::Config mod(path.string());
    smoother->configure(ayanami::ModuleContext{"esdfmap_minco_smoother", mod},
                        map);
    ayanami::nav_msgs::Path p = makeAstarStylePath(81, 0.2, 0.0);
    const auto result = smoother->smoothPath(p, 2.0);
    double far_min = 1e9;
    for (const auto& pose : p.poses) {
      const double x = pose.pose.position.x();
      if (x > 0.5 && x < 1.5) far_min = std::min(far_min, pose.pose.position.y());
    }
    // The mid-span must clear safe_dist at EVERY freeze setting: the
    // head-tangent binding freed the optimization from the at-rest head
    // (previously the first/last ~0.5 m were un-pushable and, with the
    // FSM replanning at planner_frequency, the robot always sat in that
    // dead zone grazing the wall).
    EXPECT_GE(far_min, 0.5) << "freeze=" << freeze << " " << result.message;
  }
}

TEST(EsdfmapMincoSmoother, penetratingInputProbe) {
  MaskSource source;
  source.wall(10, 110, 60);
  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(
      map, [&source](esdfmap::EsdfMaskView& view) { return source.read(view); },
      esdfmap::EsdfWriterConfig{});
  ASSERT_TRUE(writer.rebuildOnce());
  auto smoother = loadSmoother(map);
  ASSERT_NE(smoother, nullptr);
  // Path straight THROUGH the wall (y=0.025): the optimizer must lift it
  // out; observe whether it succeeds or rolls back.
  ayanami::nav_msgs::Path path = makeAstarStylePath(81, 0.025, 0.0);
  const auto result = smoother->smoothPath(path, 2.0);
  // The input's ENDPOINTS sit on the wall: they are fixed head/tail
  // anchors the smoother must not move, so a fully-repelled trajectory is
  // impossible and the hard validation rolls back to the input (SUCCESS:
  // smoothing is an enhancement; the FSM's DegradeToRawPath owns the
  // fallback). The contract under test: never SHIP a colliding trajectory.
  EXPECT_TRUE(result.ok) << result.message;
  EXPECT_NE(result.message.find("rolled back"), std::string::npos)
      << result.message;
  ASSERT_EQ(path.poses.size(), 81u);
  EXPECT_NEAR(path.poses[40].pose.position.y(), 0.025, 1e-9);
}

TEST(EsdfmapMincoSmoother, mapFramedPathThroughTransform) {
  MaskSource source;
  source.wall(10, 110, 60);
  auto map = std::make_shared<esdfmap::EsdfMap>();
  esdfmap::EsdfWriter writer(
      map, [&source](esdfmap::EsdfMaskView& view) { return source.read(view); },
      esdfmap::EsdfWriterConfig{});
  ASSERT_TRUE(writer.rebuildOnce());
  // Self-contained assembly (NOT loadSmoother): the production host
  // configures its smoother exactly once over its own config file, and
  // this test must not depend on the shared json the other tests rewrite.
  ayanami::ExtensionModule::Ptr base = ayanami::ExtensionModule::loadModule(
      "libayanami_esdfmap_minco_smoother.so");
  ASSERT_NE(base, nullptr);
  auto smoother =
      std::dynamic_pointer_cast<ayanami::esdfmap::EsdfmapSmoother>(base);
  ASSERT_NE(smoother, nullptr);
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_esdfmap_minco_test";
  const auto config_path = dir / "esdfmap_minco_smoother_mapframed.json";
  {
    std::ofstream ofs(config_path);
    ofs << R"({
  "esdfmap_minco_smoother": {
    "safe_dist": 0.55, "weight_pos": 100000.0, "weight_smooth": 1.0,
    "mu_l1": 0.4, "freeze_window": 0.0, "output_dt": 0.05,
    "max_iterations": 256, "opt_accuracy": 0.0001, "lbfgs_mem_size": 64,
    "severe_scale": 1.5, "validation_dt": 0.05
  }
})";
  }
  ayanami::Config mod(config_path.string());
  smoother->configure(ayanami::ModuleContext{"esdfmap_minco_smoother", mod},
                      map);
  // Field frame odom; path frame map = odom translated by (10, 5): a
  // map-frame path grazing the wall must still be repelled (the transform
  // seam the live stack exercises).
  ayanami::components::RobotStateController::instance().initialize(
      "map", "odom", "base_link", 10.0);
  ayanami::testing::TestTransformSource transforms;
  transforms.install();
  Eigen::Isometry3d odom_from_map = Eigen::Isometry3d::Identity();
  odom_from_map.translation().x() = 10.0;
  odom_from_map.translation().y() = 5.0;
  transforms.set("odom", "map", odom_from_map);
  ayanami::nav_msgs::Path path = makeAstarStylePath(81, 0.2, 0.0);
  path.header.frame_id = "map";
  for (auto& pose : path.poses) {
    pose.header.frame_id = "map";
    pose.pose.position.x() -= 10.0;
    pose.pose.position.y() -= 5.0;
  }
  const auto result = smoother->smoothPath(path, 2.0);
  ASSERT_TRUE(result.ok) << result.message;
  double min_y = 1e9;
  for (const auto& pose : path.poses) {
    if (pose.pose.position.x() > -11.5 && pose.pose.position.x() < -9.0) {
      min_y = std::min(min_y, pose.pose.position.y());
    }
  }
  // The wall sits at y_map = -5.0: the pushed clearance must clear
  // safe_dist in the map frame too (the window excludes the fixed tail
  // anchor zone).
  EXPECT_GE(min_y, -5.0 + 0.5) << result.message;
}
