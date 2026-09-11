// costmap_layer_plugin_test: the costmap layer plugin family (modules/
// so+json pattern).
//
// Coverage:
// 1. family loading through ModuleManager (dlopen -> cast -> apiVersion ->
//    configureParameters) for the static / obstacle / inflation plugins;
// 2. the obstacle layer's filtering policy: height/range-filtered marking,
//    previous-window clearing without raytrace endpoints, footprint
//    clearing;
// 3. the server-level plugin path: costmap_global.layers /
//    costmap_local.layers compose the maps, the same observation is stamped
//    into BOTH maps, and a clear command retires the global marks while the
//    StaticLayer restores the base map.
//
// argv: 1=static.so 2=obstacle.so 3=inflation.so

#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_types.h>

#include <spdlog/spdlog.h>

#include <costmap_2d/layered_costmap.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/abstract/components/module_manager.hpp>
#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/costmap/components/costmap_navigation_server.hpp>
#include <ayanami/costmap/core/costmap_layer_plugin.hpp>
#include <ayanami/utility/bootstrap.hpp>
#include <ayanami/utility/config.hpp>

using ayanami::testing::testTransforms;
namespace {

std::filesystem::path makeTempDir() {
  auto dir = std::filesystem::temp_directory_path() /
             ("layer_plugin_" + std::to_string(::getpid()) + "_" +
              std::to_string(std::chrono::steady_clock::now()
                                 .time_since_epoch()
                                 .count()));
  std::filesystem::create_directories(dir);
  return dir;
}

void writeJson(const std::filesystem::path& path, const std::string& text) {
  std::ofstream out(path);
  out << text;
}

std::string g_static_so;
std::string g_obstacle_so;
std::string g_inflation_so;

std::string layerJson(const std::string& name, const std::string& so,
                      const std::string& params) {
  return "{\n  \"layer\": {\n    \"name\": \"" + name + "\",\n    \"so_name\": \"" + so +
         "\"\n  },\n  \"" + name + "\": " + params + "\n}\n";
}

using ayanami::costmap::CostmapLayerPlugin;
using Manager = ayanami::components::ModuleManager<CostmapLayerPlugin>;

CostmapLayerPlugin::Ptr loadLayer(Manager& manager, const std::string& name,
                                  const std::string& so,
                                  const std::string& params) {
  const auto dir = makeTempDir();
  writeJson(dir / "layer.json", layerJson(name, so, params));
  ayanami::Config layer_cfg((dir / "layer.json").string());
  ayanami::components::ModuleSpec spec;
  spec.name = name;
  spec.so_name = so;
  EXPECT_TRUE(manager.loadModule(
      spec, ayanami::ModuleContext{spec.name, layer_cfg}, {}))
      << manager.lastFailureMessage();
  return manager.getModule(name);
}

}  // namespace

TEST(CostmapLayerPluginTest, familyLoadAppliesParameters) {
  Manager manager;
  auto static_plugin =
      loadLayer(manager, "static_layer", g_static_so,
                "{\"static_map\": \"\", \"size_x\": 20, \"size_y\": 20, "
                "\"resolution\": 0.5}");
  ASSERT_NE(static_plugin, nullptr);
  EXPECT_EQ(static_plugin->apiVersion(), CostmapLayerPlugin::kApiVersion);
  EXPECT_NE(static_plugin->layer(), nullptr);

  auto inflation_plugin =
      loadLayer(manager, "inflation_layer", g_inflation_so,
                "{\"inflation_radius\": 1.5, \"inscribed_radius\": 1.0, "
                "\"cost_scaling_factor\": 2.0}");
  ASSERT_NE(inflation_plugin, nullptr);

  auto obstacle_plugin = loadLayer(
      manager, "obstacle_layer", g_obstacle_so,
      "{\"combination\": \"overwrite\", \"clear_to_free\": true, "
      "\"footprint_clearing\": false, \"clear_margin\": 1.5, "
      "\"observation_sources\": [\"cloud\"], "
      "\"cloud\": {\"type\": \"pointcloud\", \"obstacle_max_range\": 5.0}}");
  ASSERT_NE(obstacle_plugin, nullptr);
  const auto& sources = obstacle_plugin->observationSources();
  ASSERT_EQ(sources.size(), 1u);
  EXPECT_EQ(sources.front().name, "cloud");
  EXPECT_EQ(sources.front().kind,
            ayanami::costmap::ObservationSourceConfig::Kind::PointCloud);
  EXPECT_DOUBLE_EQ(sources.front().obstacle_max_range, 5.0);
}

TEST(CostmapLayerPluginTest, obstacleObserveFiltersMarksAndClears) {
  Manager manager;
  auto obstacle_plugin = loadLayer(
      manager, "obstacle_layer", g_obstacle_so,
      "{\"combination\": \"overwrite\", \"clear_to_free\": true, "
      "\"footprint_clearing\": true, \"clear_margin\": 1.5, "
      "\"observation_sources\": [\"cloud\"], "
      "\"cloud\": {\"type\": \"pointcloud\", \"min_obstacle_height\": 0.0, "
      "\"max_obstacle_height\": 1.0, \"obstacle_min_range\": 0.0, "
      "\"obstacle_max_range\": 6.0}}");
  ASSERT_NE(obstacle_plugin, nullptr);

  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(20, 20, 0.5, 0.0, 0.0);
  lm->addLayer(obstacle_plugin->layer());
  // Circular footprint at the robot (5,5): the layer must never let the
  // robot mark itself.
  costmap_2d::Footprint footprint;
  for (int i = 0; i < 16; ++i) {
    const double angle = 2.0 * M_PI * i / 16.0;
    footprint.push_back(costmap_2d::Point{5.0 + 0.2 * std::cos(angle),
                                          5.0 + 0.2 * std::sin(angle), 0.0});
  }
  lm->setFootprint(footprint);

  const auto& source = obstacle_plugin->observationSources().front();
  std::vector<Eigen::Vector3d> hits;
  std::vector<Eigen::Vector3d> no_clearing;
  ayanami::costmap::MapObservation observation;
  observation.source = &source;
  // Sensor 0.1 m ahead of the robot: robot (5,5) -> sensor (5.1,5).
  observation.t_frame_sensor = Eigen::Translation3d(5.1, 5.0, 0.0) *
                               Eigen::Quaterniond::Identity();
  observation.points = &hits;
  observation.clearing_points = &no_clearing;
  observation.robot_pose_xy_yaw = Eigen::Vector3d(5.0, 5.0, 0.0);
  observation.has_robot_pose = true;

  // First hit at world (5.6, 5) -> cell (11, 10); the z=5 m point is
  // dropped by the height window.
  hits.emplace_back(0.5, 0.0, 0.0);
  hits.emplace_back(0.5, 0.0, 5.0);
  obstacle_plugin->observe(observation);
  lm->updateMap(5.0, 5.0, 0.0);

  auto* master = lm->getCostmap();
  EXPECT_EQ(master->getCost(11, 10), costmap_2d::LETHAL_OBSTACLE);
  // The robot's own cells stay free (footprint clearing).
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::FREE_SPACE);

  // Second observation elsewhere, no raytrace endpoints: the previous
  // window (expanded by clear_margin) retires the old mark.
  hits.clear();
  hits.emplace_back(3.5, 0.0, 0.0);  // world (8.6, 5) -> cell (17, 10)
  obstacle_plugin->observe(observation);
  lm->updateMap(5.0, 5.0, 0.0);
  EXPECT_EQ(master->getCost(11, 10), costmap_2d::FREE_SPACE);
  EXPECT_EQ(master->getCost(17, 10), costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapLayerPluginTest, serverComposesMapsFromLayerLists) {
  const auto dir = makeTempDir();
  const auto modules_dir = dir / "modules";
  std::filesystem::create_directories(modules_dir);
  writeJson(dir / "config.json",
            "{\n  \"global\": {\n    \"config_navigation\": \"navigation.json\"\n  }\n}\n");
  writeJson(dir / "navigation.json",
            "{\n"
            "  \"navigation\": {\"robot_frame\": \"base_link\"},\n"
            "  \"costmap\": {\n"
            "    \"global_costmap_frame\": \"map\",\n"
            "    \"local_costmap_frame\": \"odom\",\n"
            "    \"resolution\": 0.5,\n"
            "    \"robot_radius\": 0.2\n"
            "  },\n"
            "  \"costmap_global\": {\n"
            "    \"layers\": [\"modules/static.json\", \"modules/obstacle_global.json\", \"modules/inflation.json\"],\n"
            "    \"rolling_window\": false,\n"
            "    \"track_unknown\": false\n"
            "  },\n"
            "  \"costmap_local\": {\n"
            "    \"layers\": [\"modules/obstacle.json\", \"modules/inflation.json\"],\n"
            "    \"size_x\": 20,\n"
            "    \"size_y\": 20,\n"
            "    \"rolling_window\": true,\n"
            "    \"transform_tolerance\": 0.1\n"
            "  }\n"
            "}\n");
  writeJson(modules_dir / "static.json",
            layerJson("static_layer", g_static_so,
                      "{\"size_x\": 20, \"size_y\": 20, \"resolution\": 0.5}"));
  writeJson(modules_dir / "inflation.json",
            layerJson("inflation_layer", g_inflation_so,
                      "{\"inflation_radius\": 1.0, \"cost_scaling_factor\": "
                      "2.0, \"inscribed_radius\": 0.2}"));
  writeJson(modules_dir / "obstacle.json",
            layerJson("obstacle_layer", g_obstacle_so,
                      "{\"combination\": \"overwrite\", \"clear_to_free\": "
                      "true, \"footprint_clearing\": true, \"clear_margin\": "
                      "1.0, \"observation_sources\": [\"cloud\"], \"cloud\": "
                      "{\"type\": \"pointcloud\", \"obstacle_max_range\": "
                      "6.0}}"));
  writeJson(modules_dir / "obstacle_global.json",
            layerJson("obstacle_layer", g_obstacle_so,
                      "{\"combination\": \"overwrite\", \"clear_to_free\": "
                      "false, \"footprint_clearing\": true, \"clear_margin\": "
                      "1.0, \"observation_sources\": [\"cloud\"], \"cloud\": "
                      "{\"type\": \"pointcloud\", \"obstacle_max_range\": "
                      "6.0}}"));

  ayanami::bootstrap(dir.string());
  ayanami::costmap::CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  // The legacy costmap_global.observation_sources key is unused on the
  // plugin path.
  ASSERT_EQ(server.globalObservationSources().size(), 0u);
  ASSERT_NE(server.globalCostmap(), nullptr);

  // Robot pose in both map frames (identity chains).
  auto& rsc = ayanami::components::RobotStateController::instance();
  testTransforms().set("odom", "base_link", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity());
  testTransforms().set("map", "odom", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity());
  testTransforms().set("base_link", "lidar", ayanami::Timestamp::latest(),
                   Eigen::Translation3d(0.1, 0.0, 0.0) *
                       Eigen::Quaterniond::Identity());

  ayanami::sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  pcl::PointXYZI p;
  p.x = 2.0f;  // world (2.1, 0): outside the footprint-clear box (0.2 m
               // footprint + one 0.5 m cell margin); the +0.1 sensor
               // offset lands the sum strictly inside cell x=4
  p.y = 0.0f;
  p.z = 0.0f;
  p.intensity = 0.0f;
  cloud.cloud.push_back(p);
  server.handleObstaclePointCloud(cloud);

  auto* global_master = server.globalCostmap()->get().getCostmap();
  auto* local_master = server.localCostmap()->get().getCostmap();
  unsigned int gmx = 0;
  unsigned int gmy = 0;
  ASSERT_TRUE(global_master->worldToMap(2.1, 0.0, gmx, gmy));
  // The SAME observation lands on BOTH maps (each layer instance owns its
  // marks): the planner timescale sees real obstacles. The local rolling
  // window recenters on the robot, so its cell math must go through the
  // post-update origin.
  EXPECT_EQ(global_master->getCost(gmx, gmy), costmap_2d::LETHAL_OBSTACLE);
  spdlog::warn("DBG gmx={} gmy={} origin=({},{}) res={}", gmx, gmy,
               global_master->getOriginX(), global_master->getOriginY(),
               global_master->getResolution());
  unsigned int lmx = 0;
  unsigned int lmy = 0;
  ASSERT_TRUE(local_master->worldToMap(2.1, 0.0, lmx, lmy));
  EXPECT_EQ(local_master->getCost(lmx, lmy), costmap_2d::LETHAL_OBSTACLE);
  // The global mark inflates (the obstacle layer sits under the inflation
  // layer in the composition order).
  EXPECT_GT(global_master->getCost(gmx, gmy + 1), 0);

  // A clear command retires the global dynamic marks; the StaticLayer (no
  // map file -> all-free fallback geometry) restores the base map. The
  // local map is untouched.
  ayanami::costmap::ClearCostmapCommand command;
  command.affected_maps = "global";
  command.clear_radius = 3.0;
  ASSERT_TRUE(server.executeClearCommand(command));
  EXPECT_EQ(global_master->getCost(gmx, gmy), costmap_2d::FREE_SPACE);
  EXPECT_EQ(local_master->getCost(lmx, lmy), costmap_2d::LETHAL_OBSTACLE);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 3) {
    g_static_so = argv[1];
    g_obstacle_so = argv[2];
    g_inflation_so = argv[3];
  }
  return RUN_ALL_TESTS();
}
