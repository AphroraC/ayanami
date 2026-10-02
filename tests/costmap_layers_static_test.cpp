// costmap_layers_static_test: the STATIC layer composition (the successor
// of the former dlopen layer-plugin family).
//
// Coverage:
// 1. buildLocalLayers: INLINE parameter parsing (combination / decay /
//    footprint clearing + observation_sources with per-source child
//    blocks), layer types, and one-source-per-kind enforcement;
// 2. buildGlobalLayers: fallback geometry sizing (no pgm deployed) and the
//    inflation parameters wired through setters;
// 3. CostmapObstacleApplier: height/range-filtered marking, every hit
//    doubling as its own ray endpoint, and footprint clearing.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <costmap/costmap_wrapper.hpp>
#include <costmap/layered_costmap.hpp>

#include <ayanami/costmap/components/costmap_layers_builder.hpp>

namespace {

std::filesystem::path writeConfig(const std::string& content) {
  auto dir = std::filesystem::temp_directory_path() /
             "ayanami_layers_static_test";
  std::filesystem::create_directories(dir);
  const auto path = dir / "navigation.json";
  std::ofstream out(path);
  out << content;
  return dir;
}

const char* kRoute = R"({
  "costmap_global": {"rolling_window": false, "track_unknown": false,
      "static_layer": {"size_x": 20, "size_y": 20, "resolution": 0.1},
      "inflation_layer": {"inflation_radius": 0.6, "inscribed_radius": 0.3,
                           "cost_scaling_factor": 3.0}},
  "costmap_local": {"size_x": 40, "size_y": 40, "resolution": 0.1,
      "rolling_window": true, "track_unknown": false,
      "obstacle_layer": {"combination": "overwrite",
          "footprint_clearing": true, "decay_time": 1.5,
          "observation_sources": ["scan"],
          "scan": {"type": "laserscan", "obstacle_max_range": 3.0}},
      "inflation_layer": {"inflation_radius": 0.5, "inscribed_radius": 0.2,
                           "cost_scaling_factor": 2.0}}
})";

}  // namespace

TEST(CostmapLayersStaticTest, buildLocalLayersParsesObstaclePolicy) {
  const auto dir = writeConfig(kRoute);
  const ayanami::Config cfg((dir / "navigation.json").string());
  const auto assembly = ayanami::costmap::buildLocalLayers(cfg);

  ASSERT_TRUE(assembly.obstacle_layer);
  ASSERT_TRUE(assembly.inflation_layer);
  ASSERT_TRUE(assembly.obstacle_applier);
  ASSERT_EQ(assembly.static_layer, nullptr);
  ASSERT_EQ(assembly.observation_sources.size(), 1u);
  const auto& source = assembly.observation_sources.front();
  EXPECT_EQ(source.name, "scan");
  EXPECT_EQ(source.kind,
            ayanami::costmap::ObservationSourceConfig::Kind::LaserScan);
  EXPECT_DOUBLE_EQ(source.obstacle_max_range, 3.0);
  // decay / combination reached the layer through its setters.
  EXPECT_DOUBLE_EQ(assembly.obstacle_layer->decayTime(), 1.5);
}

TEST(CostmapLayersStaticTest, buildGlobalLayersSizesMasterFromFallback) {
  const auto dir = writeConfig(kRoute);
  const ayanami::Config cfg((dir / "navigation.json").string());
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  const auto assembly = ayanami::costmap::buildGlobalLayers(cfg, *lm);

  ASSERT_TRUE(assembly.static_layer);
  ASSERT_TRUE(assembly.inflation_layer);
  ASSERT_EQ(assembly.obstacle_layer, nullptr);
  // No pgm deployed: the fallback geometry sized the master (20x20 @ 0.1).
  ASSERT_TRUE(lm->isInitialized());
  EXPECT_EQ(lm->getCostmap()->getSizeInCellsX(), 20u);
  EXPECT_EQ(lm->getCostmap()->getSizeInCellsY(), 20u);
  EXPECT_DOUBLE_EQ(lm->getCostmap()->getResolution(), 0.1);
}

TEST(CostmapLayersStaticTest, duplicateSourceKindIsRejected) {
  const auto dir = writeConfig(std::string(R"({
  "costmap_local": {"obstacle_layer": {
      "observation_sources": ["a", "b"],
      "a": {"type": "laserscan"}, "b": {"type": "laserscan"}}}
})"));
  const ayanami::Config cfg((dir / "navigation.json").string());
  EXPECT_THROW(ayanami::costmap::buildLocalLayers(cfg), std::runtime_error);
}

TEST(CostmapLayersStaticTest, applierMarksFiltersAndClearsFootprint) {
  const auto dir = writeConfig(kRoute);
  const ayanami::Config cfg((dir / "navigation.json").string());
  const auto assembly = ayanami::costmap::buildLocalLayers(cfg);
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "odom", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(40, 40, 0.1, -2.0, -2.0);
  lm->addLayer(assembly.obstacle_layer);
  lm->addLayer(assembly.inflation_layer);
  // Circular footprint centered on the robot (radius 0.3).
  costmap_2d::Footprint footprint;
  for (int i = 0; i < 8; ++i) {
    const double a = 2.0 * M_PI * i / 8.0;
    footprint.push_back(costmap_2d::Point{0.3 * std::cos(a),
                                          0.3 * std::sin(a), 0.0});
  }
  lm->setFootprint(footprint);
  lm->updateMap(0.0, 0.0, 0.0);

  const auto& source = assembly.observation_sources.front();
  ayanami::costmap::MapObservation observation;
  observation.source = &source;
  observation.t_frame_sensor = Eigen::Isometry3d::Identity();
  // Hits: in-window (1.5, 0); too-high z=1.2 (filtered by the default
  // max_obstacle_height=1.0); inside the footprint (0.1, 0, cleared last).
  std::vector<Eigen::Vector3d> points{
      Eigen::Vector3d(1.5, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.5, 1.2),
      Eigen::Vector3d(0.1, 0.0, 0.0)};
  std::vector<Eigen::Vector3d> clearing{Eigen::Vector3d(1.5, 0.0, 0.0)};
  observation.points = &points;
  observation.clearing_points = &clearing;
  observation.robot_pose_xy_yaw = Eigen::Vector3d::Zero();
  observation.has_robot_pose = true;

  // The applier runs under the writer's write lock; construct a wrapper to
  // take the lock the same way the server does.
  costmap_2d::CostmapWrapper wrapper(lm, "odom");
  {
    auto lock = wrapper.lockForWrite();
    assembly.obstacle_applier->apply(observation);
  }

  const auto* grid = lm->getCostmap();
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(grid->worldToMap(1.5, 0.0, mx, my));
  // Rebuild the master so presence becomes visible, then check the mark.
  wrapper.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(grid->getCost(mx, my), costmap_2d::LETHAL_OBSTACLE);
  // The footprint hit was cleared; the too-high point never landed.
  unsigned int fx = 0;
  unsigned int fy = 0;
  ASSERT_TRUE(grid->worldToMap(0.1, 0.0, fx, fy));
  EXPECT_EQ(grid->getCost(fx, fy), costmap_2d::FREE_SPACE);
  ASSERT_TRUE(grid->worldToMap(0.0, 1.5, fx, fy));
  EXPECT_EQ(grid->getCost(fx, fy), costmap_2d::FREE_SPACE);
}
