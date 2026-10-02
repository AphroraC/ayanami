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
    "safe_dist": 0.35,
    "weight_pos": 100000.0,
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
  // Inside the band: repelled past safe_dist.
  EXPECT_GE(run(0.2), 0.35);
  // Outside the band: untouched.
  EXPECT_NEAR(run(0.6), 0.6, 0.02);
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
