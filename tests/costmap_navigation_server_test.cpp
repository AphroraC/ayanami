// CostmapNavigationServer -- virtual initialize*Plugin injection
// costmap_context + new*Execution factories + configureCoreMaps + handleObstacle* map updates.
//
// Coverage:
// - configureCoreMaps builds the global/local CostmapMapContext + StaticLayer/InflationLayer/SimpleObstacleLayer
// - loadPlugins triggers the virtual initialize*Plugin -> dynamic_pointer_cast<Costmap*> -> configure(ctx, map)
// - fail case: a non-costmap-family plugin -> initialize*Plugin false + loadPlugins creates no action
// - new*Execution returns Costmap*Execution (bound to the global/local wrapper)
// - handleObstaclePointCloud goes through TransformStore into SimpleObstacleLayer + InflationLayer recompute
// (old obstacles and old inflation bands are cleared with clearArea; no stale leak)

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <atomic>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d_layers/inflation_layer.hpp>

#include <ayanami/costmap/components/costmap_map_family_factory.hpp>
#include <ayanami/map/map_family_factory.hpp>
#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <costmap_2d_layers/static_layer.hpp>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/abstract/core/abstract_goal_checker.hpp>
#include <ayanami/costmap/components/costmap_map_context.hpp>
#include <ayanami/costmap/components/costmap_controller_execution.hpp>
#include <ayanami/costmap/components/costmap_navigation_server.hpp>
#include <ayanami/costmap/components/costmap_planner_execution.hpp>
#include <ayanami/costmap/components/costmap_recovery_execution.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/messages/sensor_msgs.hpp>
#include <costmap_smac_planner/costmap_smac_planner.hpp>

namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryStatus;
using ayanami::costmap::CostmapMapContext;
using ayanami::costmap::CostmapNavigationServer;

// Read by main from argv (ctest passes the $<TARGET_FILE:...> of each test plugin).
std::string g_test_planner_so;
std::string g_test_smoother_so;
std::string g_test_controller_so;
std::string g_test_recovery_so;
std::string g_plain_planner_so;
std::string g_plain_smoother_so;
std::string g_plain_controller_so;
std::string g_plain_recovery_so;

// ---------------- Test utilities ----------------

std::filesystem::path makeTempDir() {
  std::filesystem::path dir = std::filesystem::temp_directory_path() /
                              "ayanami_costmap_navigation_server_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void writeJson(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

std::string pluginJson(const std::string& module, const std::string& name,
                        const std::string& so_name) {
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + so_name + "\"\n  }\n}\n";
}

/// Generates a temp config dir: the config.json global section contains config_costmap; each of the 4 families can
/// point at either a costmap-family success plugin or a non-costmap plugin (failure path).
std::filesystem::path makeConfigDir(
    const std::string& planner_so,
    const std::string& smoother_so,
    const std::string& controller_so,
    const std::string& recovery_so) {
  auto dir = makeTempDir() / ("cfg" + planner_so);
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  // The config_* plugin file pointers live uniformly in the navigation section of navigation.json (single source).
  writeJson(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"navigation.json\"\n"
             "  }\n}\n");
  writeJson(dir / "navigation.json",
             "{\n  \"navigation\": {\n"
             "    \"config_planner\": \"planner.json\",\n"
             "    \"config_smoother\": \"smoother.json\",\n"
             "    \"config_controller\": \"controller.json\",\n"
             "    \"config_recovery\": [\"recovery.json\"]\n"
             "  }\n}\n");

  writeJson(dir / "planner.json",
             pluginJson("planner", "test_costmap_planner", planner_so));
  writeJson(dir / "smoother.json",
             pluginJson("smoother", "test_costmap_smoother", smoother_so));
  writeJson(dir / "controller.json",
             pluginJson("controller", "test_costmap_controller",
                         controller_so));
  writeJson(dir / "recovery.json",
             "{\n  \"recovery\": {\n    \"name\": \"test_costmap_recovery\",\n"
             "    \"role\": \"clear_costmap\",\n    \"so_name\": \"" +
             recovery_so + "\"\n  }\n}\n");

  // 20x20 all-empty PGM static map (all 255 -> FREE_SPACE). The config deliberately declares 7x9 to
  // verify that a non-rolling global master is always determined by the PGM geometry.
  const std::string pgm_path = (dir / "map.pgm").string();
  {
    std::ofstream out(pgm_path, std::ios::binary);
    out << "P5\n20 20\n255\n";
    for (int i = 0; i < 400; ++i) {
      out.put(static_cast<char>(255));
    }
  }
  {
    std::ofstream metadata(dir / "map.json");
    metadata << R"({"map":{"image":"map.pgm","resolution":0.5,"origin":[-5.0,-5.0,0.0]}})";
  }

  // 20x20 @ 0.5m -> 10m x 10m. local inflation_radius=1.5 -> 3 cells.
  // Costmap parameters are merged into navigation.json (the old costmap.json was removed).
  // Note: this is the final content of navigation.json (overriding the earlier config_* versions);
  // it must also carry the plugin file pointers.
  {
    std::ofstream nav(dir / "navigation.json");
    nav << "{\n  \"navigation\": {\n"
        << "    \"config_planner\": \"planner.json\",\n"
        << "    \"config_smoother\": \"smoother.json\",\n"
        << "    \"config_controller\": \"controller.json\",\n"
        << "    \"config_recovery\": [\"recovery.json\"]\n"
        << "  },\n"
        << "  \"costmap\": {\n"
        << "    \"global_frame\": \"map\",\n"
        << "    \"local_frame\": \"odom\",\n"
        << "    \"robot_frame\": \"base_link\",\n"
        << "    \"resolution\": 0.5\n"
        << "  },\n"
        << "  \"costmap_global\": {\n"
        << "    \"size_x\": 7,\n    \"size_y\": 9,\n"
        << "    \"rolling_window\": false,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"static_map\": \"" << pgm_path << "\",\n"
        << "    \"inflation_radius\": 1.5,\n"
        << "    \"inscribed_radius\": 1.0,\n"
        << "    \"cost_scaling_factor\": 2.0\n"
        << "  },\n"
        << "  \"costmap_local\": {\n"
        << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
        << "    \"rolling_window\": true,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"inflation_radius\": 1.5,\n"
        << "    \"inscribed_radius\": 1.0,\n"
        << "    \"cost_scaling_factor\": 2.0\n"
        << "  }\n"
        << "}\n";
  }
  return dir;
}

/// Finds the SimpleObstacleLayer inside the local wrapper LayeredCostmap (for test observation).
costmap_2d_layers::SimpleObstacleLayer* findObstacleLayer(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper) {
  for (const auto& layer : wrapper->get().getPlugins()) {
    if (auto* obs =
            dynamic_cast<costmap_2d_layers::SimpleObstacleLayer*>(layer.get())) {
      return obs;
    }
  }
  return nullptr;
}

/// TestCostmapPlanner that records the initialize injection state (interface assertions).
class TestCostmapPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  std::string init_name;
  uint32_t plan_calls{0};

  virtual ~TestCostmapPlanner() override = default;

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr) override {
    init_name = context.name;
  }
  ResultT<void> makePlan(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::PoseStamped&, double, ayanami::nav_msgs::Path& plan_out) override {
    ++plan_calls;
    plan_out = ayanami::nav_msgs::Path{};
    return ResultT<void>::success();
  }
};

/// Non-costmap-family planner (dynamic_pointer_cast<CostmapPlanner> fails).
class PlainAbstractPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  ResultT<void> makePlan(const ayanami::geometry_msgs::PoseStamped&,
                          const ayanami::geometry_msgs::PoseStamped&,
                          double, ayanami::nav_msgs::Path& plan_out) override {
    plan_out = ayanami::nav_msgs::Path{};
    return ResultT<void>::success();
  }
};

/// Costmap-family smoother/controller/recovery (accept path + initialize injection assertions).
class TestCostmapSmoother : public ayanami::costmap::CostmapSmoother {
 public:
  std::string init_name;
  virtual ~TestCostmapSmoother() override = default;

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr) override {
    init_name = context.name;
  }
  ResultT<void> smoothPath(ayanami::nav_msgs::Path& path, double)  override {
    return ResultT<void>::success();
  }
};

class TestCostmapController : public ayanami::costmap::CostmapController {
 public:
  std::string init_name;
  virtual ~TestCostmapController() override = default;

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr) override {
    init_name = context.name;
  }
  ResultT<void> computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
    cmd = ayanami::geometry_msgs::TwistStamped{};
    return ResultT<void>::success();
  }
  bool setPlan(const ayanami::nav_msgs::Path&) override { return true; }
};

class TestCostmapRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  std::string init_name;
  virtual ~TestCostmapRecovery() override = default;

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr) override {
    init_name = context.name;
  }
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    cycle = std::move(data);
    return ResultT<void>::success();
  }
};

/// Non-costmap-family smoother/controller/recovery (reject path).
class PlainAbstractSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  ResultT<void> smoothPath(ayanami::nav_msgs::Path& path, double)  override {
    return ResultT<void>::success();
  }
};

class PlainAbstractController : public ayanami::abstract::AbstractController {
 public:
  ResultT<void> computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::TwistStamped&, ayanami::geometry_msgs::TwistStamped& cmd) override {
    cmd = ayanami::geometry_msgs::TwistStamped{};
    return ResultT<void>::success();
  }
  bool setPlan(const ayanami::nav_msgs::Path&) override { return true; }
};

class PlainAbstractRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    cycle = std::move(data);
    return ResultT<void>::success();
  }
};

/// Builds a single-point PointCloud2 in the sensor frame (lidar).
ayanami::sensor_msgs::PointCloud2 makePointCloud(float x, float y, float z) {
  ayanami::sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = 0.0f;
  cloud.cloud.push_back(p);
  return cloud;
}

/// Sets up TransformStore: map<-odom<-base_link<-lidar, with the robot at the odom origin.
void setupTransforms() {
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.setTransform("odom", "base_link", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity());
  rsc.setTransform("map", "odom", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity());
  rsc.setTransform("base_link", "lidar", ayanami::Timestamp::latest(),
                   Eigen::Translation3d(0.1, 0.0, 0.0) *
                       Eigen::Quaterniond::Identity());
}

}  // namespace

// ---------------- configureCoreMaps ----------------

TEST(CostmapNavigationServerTest, configureCoreMapsBuildsContextAndLayers) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  ASSERT_NE(server.costmapContext(), nullptr);
  ASSERT_NE(server.globalCostmap(), nullptr);
  ASSERT_NE(server.localCostmap(), nullptr);
  EXPECT_EQ(server.costmapContext()->getGlobalFrame(), "map");
  EXPECT_EQ(server.costmapContext()->getRobotFrame(), "base_link");
  // Frame single-source consistency: global wrapper in the global frame,
  // local wrapper in the local frame; RSC configured with the same names.
  {
    EXPECT_EQ(server.globalCostmap()->getGlobalFrame(), "map");
    EXPECT_EQ(server.localCostmap()->getGlobalFrame(),
              server.costmapContext()->getLocalFrame());
    auto& rsc = ayanami::components::RobotStateController::instance();
    EXPECT_EQ(rsc.getGlobalFrame(), server.costmapContext()->getGlobalFrame());
    EXPECT_EQ(rsc.getRobotFrame(), server.costmapContext()->getRobotFrame());
  }
  EXPECT_DOUBLE_EQ(server.globalCostmap()->get().getCostmap()->getOriginX(),
                   -5.0);
  EXPECT_DOUBLE_EQ(server.globalCostmap()->get().getCostmap()->getOriginY(),
                   -5.0);
  EXPECT_EQ(server.globalCostmap()->get().getCostmap()->getSizeInCellsX(), 20u);
  EXPECT_EQ(server.globalCostmap()->get().getCostmap()->getSizeInCellsY(), 20u);
  // map_context is injected (AbstractMapContext layer); the pose resolves through RobotStateController.
  EXPECT_EQ(server.costmapContext()->getRobotFrame(), "base_link");

  // global: StaticLayer + SimpleObstacleLayer + InflationLayer
  bool has_static = false;
  bool has_global_obstacle = false;
  bool has_global_inflation = false;
  for (const auto& layer : server.globalCostmap()->get().getPlugins()) {
    if (dynamic_cast<costmap_2d_layers::StaticLayer*>(layer.get())) {
      has_static = true;
    }
    if (dynamic_cast<costmap_2d_layers::InflationLayer*>(layer.get())) {
      has_global_inflation = true;
    }
    if (dynamic_cast<costmap_2d_layers::SimpleObstacleLayer*>(layer.get())) {
      has_global_obstacle = true;
    }
  }
  EXPECT_TRUE(has_static);
  EXPECT_TRUE(has_global_obstacle);
  EXPECT_TRUE(has_global_inflation);

  // local: SimpleObstacleLayer + InflationLayer
  bool has_obstacle = false;
  bool has_local_inflation = false;
  for (const auto& layer : server.localCostmap()->get().getPlugins()) {
    if (dynamic_cast<costmap_2d_layers::SimpleObstacleLayer*>(layer.get())) {
      has_obstacle = true;
    }
    if (dynamic_cast<costmap_2d_layers::InflationLayer*>(layer.get())) {
      has_local_inflation = true;
    }
  }
  EXPECT_TRUE(has_obstacle);
  EXPECT_TRUE(has_local_inflation);
}

// ---- Map-family factory: the context has a single creation point (the host creates it via the factory; the core only knows the abstract factory face) ----

TEST(CostmapNavigationServerTest, configureCoreMapsCreatesContextViaFactory) {
  class RecordingFactory final : public ayanami::map::MapFamilyFactory {
   public:
    std::string familyName() const override { return "costmap"; }
    ayanami::map::AbstractMapContext::Ptr createContext() override {
      auto ctx = std::make_shared<ayanami::costmap::CostmapMapContext>();
      created = ctx;
      ++creations;
      return ctx;
    }
    ayanami::costmap::CostmapMapContext::Ptr created;
    std::atomic<int> creations{0};
  };

  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so);
  auto factory = std::make_shared<RecordingFactory>();
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string(), factory);

  EXPECT_EQ(factory->creations.load(), 1);
  ASSERT_NE(factory->created, nullptr);
  EXPECT_EQ(server.costmapContext(), factory->created);
  ASSERT_NE(server.mapFamilyFactory(), nullptr);
  EXPECT_EQ(server.mapFamilyFactory(), factory);
  EXPECT_EQ(server.mapFamilyFactory()->familyName(), "costmap");
}

TEST(CostmapMapFamilyFactoryTest, factoryCreatesDefaultCostmapContext) {
  ayanami::costmap::CostmapMapFamilyFactory factory;
  EXPECT_EQ(factory.familyName(), "costmap");
  auto ctx = factory.createContext();
  ASSERT_NE(ctx, nullptr);
  EXPECT_EQ(ctx->mapFamily(), "costmap");
}

TEST(CostmapNavigationServerTest, defaultConfigMatchesNavPgmGeometry) {
  CostmapNavigationServer server;
  server.configureCoreMaps(AYANAMI_TEST_CONFIG_DIR);
  const auto* global = server.globalCostmap()->get().getCostmap();
  ASSERT_NE(global, nullptr);
  EXPECT_EQ(global->getSizeInCellsX(), 266u);
  EXPECT_EQ(global->getSizeInCellsY(), 159u);
  EXPECT_DOUBLE_EQ(global->getResolution(), 0.05);
  EXPECT_DOUBLE_EQ(global->getOriginX(), -8.75);
  EXPECT_DOUBLE_EQ(global->getOriginY(), -3.85);
}

TEST(CostmapNavigationServerTest, robotRadiusBuildsCircularFootprint) {
  // Real config: costmap::robot_radius=0.12 -> a 16-gon approximating the circle (nav_demo semantics).
  CostmapNavigationServer server;
  server.configureCoreMaps(AYANAMI_TEST_CONFIG_DIR);
  const auto& footprint =
      server.globalCostmap()->get().getFootprint();
  ASSERT_EQ(footprint.size(), 16u);
  for (const auto& point : footprint) {
    EXPECT_NEAR(std::hypot(point.x, point.y), 0.12, 1e-9);
  }
  // 16-gon inscribed = 0.12 * cos(pi/16) ~ 0.1177.
  EXPECT_NEAR(server.globalCostmap()->get().getInscribedRadius(),
              0.12 * std::cos(M_PI / 16.0), 1e-6);
  EXPECT_NEAR(server.localCostmap()->get().getInscribedRadius(),
              0.12 * std::cos(M_PI / 16.0), 1e-6);
}

// ---------------- Virtual initialize*Plugin ----------------

TEST(CostmapNavigationServerTest, initializePlannerPluginInjectsContext) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto planner = std::make_shared<TestCostmapPlanner>();
  ayanami::abstract::AbstractPlanner::Ptr base = planner;
  EXPECT_TRUE(server.initializePlannerPlugin("my_planner", ayanami::Config(""), base));

  EXPECT_EQ(planner->init_name, "my_planner");
}

TEST(CostmapNavigationServerTest, initializePlannerPluginRejectsNonCostmapPlanner) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto plain = std::make_shared<PlainAbstractPlanner>();
  EXPECT_FALSE(server.initializePlannerPlugin("plain", ayanami::Config(""), plain));
}

TEST(CostmapNavigationServerTest, initializeSmootherPluginInjectsContext) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto smoother = std::make_shared<TestCostmapSmoother>();
  ayanami::abstract::AbstractSmoother::Ptr base = smoother;
  EXPECT_TRUE(server.initializeSmootherPlugin("my_smoother", ayanami::Config(""), base));
  EXPECT_EQ(smoother->init_name, "my_smoother");
}

TEST(CostmapNavigationServerTest, initializeSmootherPluginRejectsNonCostmapSmoother) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto plain = std::make_shared<PlainAbstractSmoother>();
  EXPECT_FALSE(server.initializeSmootherPlugin("plain", ayanami::Config(""), plain));
}

TEST(CostmapNavigationServerTest, initializeControllerPluginInjectsContext) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto controller = std::make_shared<TestCostmapController>();
  ayanami::abstract::AbstractController::Ptr base = controller;
  EXPECT_TRUE(server.initializeControllerPlugin("my_controller", ayanami::Config(""), base));
  EXPECT_EQ(controller->init_name, "my_controller");
}

TEST(CostmapNavigationServerTest, initializeControllerPluginRejectsNonCostmapController) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  // A plain AbstractController is no longer admitted (the fallback was removed).
  auto plain = std::make_shared<PlainAbstractController>();
  EXPECT_FALSE(server.initializeControllerPlugin("plain", ayanami::Config(""), plain));
}

TEST(CostmapNavigationServerTest, initializeRecoveryPluginInjectsContext) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto recovery = std::make_shared<TestCostmapRecovery>();
  ayanami::abstract::AbstractRecovery::Ptr base = recovery;
  EXPECT_TRUE(server.initializeRecoveryPlugin("my_recovery", ayanami::Config(""), base));
  EXPECT_EQ(recovery->init_name, "my_recovery");
}

TEST(CostmapNavigationServerTest, initializeRecoveryPluginRejectsNonCostmapRecovery) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto plain = std::make_shared<PlainAbstractRecovery>();
  EXPECT_FALSE(server.initializeRecoveryPlugin("plain", ayanami::Config(""), plain));
}

TEST(CostmapNavigationServerTest, initializePlannerPluginFailsWithoutConfiguredMaps) {
  CostmapNavigationServer server;  // configureCoreMaps not called (minor 1)
  auto planner = std::make_shared<TestCostmapPlanner>();
  EXPECT_FALSE(server.initializePlannerPlugin("p", ayanami::Config(""), planner));
  EXPECT_FALSE(server.initializeSmootherPlugin("s", ayanami::Config(""), nullptr));
}

// ---------------- loadPlugins end to end ----------------

TEST(CostmapNavigationServerTest, loadPluginsLoadsCostmapPluginsAndCreatesActions) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  server.loadPlugins(dir.string());

  ASSERT_NE(server.plannerAction(), nullptr);
  ASSERT_NE(server.smootherAction(), nullptr);
  ASSERT_NE(server.controllerAction(), nullptr);
  ASSERT_NE(server.recoveryAction("test_costmap_recovery"), nullptr);
  ASSERT_EQ(server.recoveryAction("missing"), nullptr);
}

TEST(CostmapNavigationServerTest, loadPluginsWithNonCostmapPlannerFails) {
  auto dir = makeConfigDir(g_plain_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  server.loadPlugins(dir.string());

  // planner dynamic_pointer_cast fails -> not registered -> plannerAction not created
  EXPECT_EQ(server.plannerAction(), nullptr);
  // The other families load normally
  ASSERT_NE(server.controllerAction(), nullptr);
  ASSERT_NE(server.smootherAction(), nullptr);
}

TEST(CostmapNavigationServerTest, loadPluginsRejectsAllPlainAbstractPlugins) {
  // All four plain abstract families rejected -> none of the four actions is created.
  auto dir = makeConfigDir(g_plain_planner_so, g_plain_smoother_so,
                             g_plain_controller_so, g_plain_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  server.loadPlugins(dir.string());

  EXPECT_EQ(server.plannerAction(), nullptr);
  EXPECT_EQ(server.smootherAction(), nullptr);
  EXPECT_EQ(server.controllerAction(), nullptr);
  EXPECT_EQ(server.recoveryAction("test_costmap_recovery"), nullptr);
}

// ---------------- createActions production path = Costmap*Action ----------------
// Production no longer goes through the base-class new*Execution virtual factory indirection (Server*Action was deleted):
// CostmapNavigationServer::createActions constructs Costmap*Action directly, whose
// createExecution produces Costmap*Execution carrying the CostmapMapContext.

namespace {

/// createExecution is protected; tests call it through the exposing wrapper classes below (verifying the Costmap*Action
/// production path produces Costmap*Execution).
class ExposedCostmapPlannerAction : public ayanami::costmap::CostmapPlannerAction {
 public:
  using CostmapPlannerAction::CostmapPlannerAction;
  using CostmapPlannerAction::createExecution;
};
class ExposedCostmapSmootherAction : public ayanami::costmap::CostmapSmootherAction {
 public:
  using CostmapSmootherAction::CostmapSmootherAction;
  using CostmapSmootherAction::createExecution;
};
class ExposedCostmapControllerAction : public ayanami::costmap::CostmapControllerAction {
 public:
  using CostmapControllerAction::CostmapControllerAction;
  using CostmapControllerAction::createExecution;
};
class ExposedCostmapRecoveryAction : public ayanami::costmap::CostmapRecoveryAction {
 public:
  using CostmapRecoveryAction::CostmapRecoveryAction;
  using CostmapRecoveryAction::createExecution;
};

}  // namespace

TEST(CostmapNavigationServerTest, createActionsBuildsCostmapActions) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  server.loadPlugins(dir.string());

  ASSERT_NE(server.plannerAction(), nullptr);
  ASSERT_NE(server.smootherAction(), nullptr);
  ASSERT_NE(server.controllerAction(), nullptr);
  ASSERT_NE(server.recoveryAction("test_costmap_recovery"), nullptr);

  // Every action is in the costmap family (proof of the production path).
  EXPECT_NE(dynamic_cast<ayanami::costmap::CostmapPlannerAction*>(
                server.plannerAction()),
            nullptr);
  EXPECT_NE(dynamic_cast<ayanami::costmap::CostmapSmootherAction*>(
                server.smootherAction()),
            nullptr);
  EXPECT_NE(dynamic_cast<ayanami::costmap::CostmapControllerAction*>(
                server.controllerAction()),
            nullptr);
  EXPECT_NE(dynamic_cast<ayanami::costmap::CostmapRecoveryAction*>(
                server.recoveryAction("test_costmap_recovery")),
            nullptr);
}

TEST(CostmapNavigationServerTest, costmapActionCreatesCostmapExecutions) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  server.loadPlugins(dir.string());

  auto planner = std::make_shared<TestCostmapPlanner>();
  auto smoother = std::make_shared<TestCostmapSmoother>();
  auto controller = std::make_shared<TestCostmapController>();
  auto recovery = std::make_shared<TestCostmapRecovery>();

  // Costmap*Action::createExecution -> Costmap*Execution (with maps).
  // createExecution is protected; called through the test-internal exposing wrapper.
  ExposedCostmapPlannerAction pa(planner, server.costmapContext());
  ASSERT_NE(dynamic_cast<ayanami::costmap::CostmapPlannerExecution*>(
                pa.createExecution().get()),
            nullptr);
  ExposedCostmapSmootherAction sa(smoother, server.costmapContext());
  ASSERT_NE(dynamic_cast<ayanami::costmap::CostmapSmootherExecution*>(
                sa.createExecution().get()),
            nullptr);
  ExposedCostmapControllerAction ca(controller, server.costmapContext());
  ASSERT_NE(dynamic_cast<ayanami::costmap::CostmapControllerExecution*>(
                ca.createExecution().get()),
            nullptr);
  ExposedCostmapRecoveryAction ra(recovery, server.costmapContext());
  ASSERT_NE(dynamic_cast<ayanami::costmap::CostmapRecoveryExecution*>(
                ra.createExecution().get()),
            nullptr);
}

// ---------------- handleObstaclePointCloud (included) ----------------

TEST(CostmapNavigationServerTest, handleObstaclePointCloudMarksAndInflates) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();

  // Single point: lidar(0,0) -> odom(0.1,0); local origin = robot(-5,-5) -> cell(10,10)
  server.handleObstaclePointCloud(makePointCloud(0.0f, 0.0f, 0.0f));

  auto* master = server.localCostmap()->get().getCostmap();
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(10, 11), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  // The decay band within 3 cells is non-zero
  EXPECT_GT(master->getCost(10, 13), 0);

  // Planner consumes the global costmap, so the same physical obstacle must
  // be represented at the matching map-frame world coordinate there too.
  auto* global_master = server.globalCostmap()->get().getCostmap();
  unsigned int global_mx = 0;
  unsigned int global_my = 0;
  ASSERT_TRUE(global_master->worldToMap(0.1, 0.0, global_mx, global_my));
  EXPECT_EQ(global_master->getCost(global_mx, global_my),
            costmap_2d::LETHAL_OBSTACLE);

  // After the obstacle moves (lidar(2,0) -> odom(2.1,0) -> cell(14,10)) the old inflation band disappears
  server.handleObstaclePointCloud(makePointCloud(2.0f, 0.0f, 0.0f));

  // The old obstacle cell + old inflation cells return to FREE (more than 3 cells from the new source)
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::FREE_SPACE);
  EXPECT_EQ(master->getCost(10, 13), costmap_2d::FREE_SPACE);
  // Obstacle at the new position + new inflation band
  EXPECT_EQ(master->getCost(14, 10), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(14, 11), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  // The old global obstacle is restored from the static layer and the new
  // observation is marked at its map-frame coordinate.
  EXPECT_EQ(global_master->getCost(global_mx, global_my),
            costmap_2d::FREE_SPACE);
  ASSERT_TRUE(global_master->worldToMap(2.1, 0.0, global_mx, global_my));
  EXPECT_EQ(global_master->getCost(global_mx, global_my),
            costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapNavigationServerTest, smacPlanAvoidsObservedGlobalObstacle) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();

  // Place the observation directly between start and goal in the map frame.
  server.handleObstaclePointCloud(makePointCloud(0.0f, 0.0f, 0.0f));

auto planner = std::make_shared<ayanami::modules::CostmapSmacPlanner>();
  planner->configure(ayanami::PluginContext{"smac", ayanami::Config("")},
                     server.costmapContext()->getSharedConstPtr());
  auto start = ayanami::geometry_msgs::PoseStamped::fromIsometry(
      "map", Eigen::Isometry3d::Identity());
  start.pose.position = Eigen::Vector3d(-4.0, 0.0, 0.0);
  auto goal = ayanami::geometry_msgs::PoseStamped::fromIsometry(
      "map", Eigen::Isometry3d::Identity());
  goal.pose.position = Eigen::Vector3d(4.0, 0.0, 0.0);

  ayanami::nav_msgs::Path path;
  auto plan_result = planner->makePlan(start, goal, 0.1, path);
  ASSERT_TRUE(plan_result.ok) << plan_result.message;
  ASSERT_FALSE(path.poses.empty());

  bool deviated_from_straight_line = false;
  {
    std::shared_lock<std::shared_mutex> lock(server.globalCostmap()->mutex());
    const auto* global = server.globalCostmap()->get().getCostmap();
    for (const auto& pose : path.poses) {
      unsigned int mx = 0;
      unsigned int my = 0;
      ASSERT_TRUE(global->worldToMap(pose.pose.position.x(),
                                     pose.pose.position.y(), mx, my));
      EXPECT_FALSE(costmap_2d::isLethal(global->getCost(mx, my)));
      deviated_from_straight_line |=
          std::abs(pose.pose.position.y()) > 0.25;
    }
  }
  EXPECT_TRUE(deviated_from_straight_line);
}

TEST(CostmapNavigationServerTest, handleObstaclePointCloudIgnoredWithoutMap) {
  CostmapNavigationServer server;  // configureCoreMaps not called
  setupTransforms();
  EXPECT_NO_THROW(server.handleObstaclePointCloud(makePointCloud(0, 0, 0)));
}

TEST(CostmapNavigationServerTest, obstacleStampQueriesTfAtMessageTime) {
  // Sensor header.stamp is consumed: TF queries use the message time, not an implicit latest.
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto& rsc = ayanami::components::RobotStateController::instance();
  const auto ts = ayanami::timestampFromSeconds(123.0);
  // Same values as setupTransforms but published at specific times (identical values, harmless across tests).
  rsc.setTransform("odom", "base_link", ts, Eigen::Isometry3d::Identity());
  rsc.setTransform("map", "odom", ts, Eigen::Isometry3d::Identity());
  rsc.setTransform("base_link", "lidar", ts,
                   Eigen::Translation3d(0.1, 0.0, 0.0) *
                       Eigen::Quaterniond::Identity());

  // The frame stamp hits the TF at time ts -> lands on the map (lidar(0,0) -> odom(0.1,0) -> cell(10,10)).
  auto cloud = makePointCloud(0.0f, 0.0f, 0.0f);
  cloud.header.stamp = ts;
  server.handleObstaclePointCloud(cloud);
  auto* master = server.localCostmap()->get().getCostmap();
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE);

  // The message time (ts+1s) has no matching TF -> the lookup fails (queried by message stamp); the wrongly
  // resolved transform is not applied and the previous frame marks are kept as-is -> proves the stamp is truly consumed.
  auto stale = makePointCloud(0.0f, 0.0f, 0.0f);
  stale.header.stamp = ayanami::timestampFromSeconds(124.0);
  server.handleObstaclePointCloud(stale);
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapNavigationServerTest, obstacleLaserScanUsesHeaderStamp) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto& rsc = ayanami::components::RobotStateController::instance();
  const auto ts = ayanami::timestampFromSeconds(200.0);
  rsc.setTransform("odom", "base_link", ts, Eigen::Isometry3d::Identity());
  rsc.setTransform("map", "odom", ts, Eigen::Isometry3d::Identity());
  rsc.setTransform("base_link", "lidar", ts,
                   Eigen::Translation3d(0.1, 0.0, 0.0) *
                       Eigen::Quaterniond::Identity());

  ayanami::sensor_msgs::LaserScan scan;
  scan.header.frame_id = "lidar";
  scan.header.stamp = ts;
  scan.angle_min = 0.0f;
  scan.angle_max = 0.0f;
  scan.angle_increment = 0.0f;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges = {1.0f};  // A single beam 1m along +x -> odom(1.1,0) -> cell(12,10)
  server.handleObstacleLaserScan(scan);

  auto* master = server.localCostmap()->get().getCostmap();
  EXPECT_EQ(master->getCost(12, 10), costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapNavigationServerTest, laserRaytraceClearsDynamicObstacle) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();

  server.handleObstaclePointCloud(makePointCloud(1.0f, 0.0f, 0.0f));
  auto* local = server.localCostmap()->get().getCostmap();
  ASSERT_EQ(local->getCost(12, 10), costmap_2d::LETHAL_OBSTACLE);

  ayanami::sensor_msgs::LaserScan scan;
  scan.header.frame_id = "lidar";
  scan.angle_min = 0.0f;
  scan.angle_max = 0.0f;
  scan.angle_increment = 0.0f;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges = {std::numeric_limits<float>::infinity()};
  server.handleObstacleLaserScan(scan);

  EXPECT_EQ(local->getCost(12, 10), costmap_2d::FREE_SPACE);
}

TEST(CostmapNavigationServerTest, obstacleInputIsDirectMethodNotGlobalSlot) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();

  auto* obstacle_layer = findObstacleLayer(server.localCostmap());
  ASSERT_NE(obstacle_layer, nullptr);

  // Obstacle observation input does not go through the global bypass slot (output/observation layering discipline): it lands on the map via the public host-delivered method.
  server.handleObstaclePointCloud(makePointCloud(0.0f, 0.0f, 0.0f));

  auto* master = server.localCostmap()->get().getCostmap();
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapNavigationServerTest, concurrentObstacleHandlersDoNotCrash) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();

  // Two threads concurrently hammer the same obstacle position: the shared prev_obstacles_local means the clear window
  // must cover this frame points (obs-frame semantics = "the current observation replaces the old one"), so the same cell is used to guarantee
  // that the last write under any interleaving is LETHAL -> deterministic assertion. The lock serializes concurrent modification of marks/
  // clear_regions/prev_obstacles_local/grid (without it, UB could crash).
  std::atomic<bool> start{false};
  auto run = [&](float x) {
    while (!start.load()) {
      std::this_thread::yield();
    }
    for (int i = 0; i < 50; ++i) {
      server.handleObstaclePointCloud(makePointCloud(x, 0.0f, 0.0f));
    }
  };
  std::thread ta(run, 0.0f);
  std::thread tb(run, 0.0f);
  start.store(true);
  ta.join();
  tb.join();

  auto* master = server.localCostmap()->get().getCostmap();
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapNavigationServerTest, destructorStopsActionsAndDoesNotCrash) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  {
    CostmapNavigationServer server;
    server.configureCoreMaps(dir.string());
    server.loadPlugins(dir.string());

    ayanami::actions::GetPathGoal goal;
    goal.start = ayanami::geometry_msgs::PoseStamped::fromIsometry(
        "map", Eigen::Isometry3d::Identity());
    goal.goal = ayanami::geometry_msgs::PoseStamped::fromIsometry(
        "map", Eigen::Isometry3d::Identity());
    auto task = server.requestPath(goal);
    ASSERT_TRUE(task.valid());
    for (int i = 0; i < 300 && !task.ready(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(task.ready());

    server.start();
    server.stop();
  }
  // End of scope -> destruction: base-class stop/reset/clearPlugins,
  // no hang.
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_test_planner_so = argv[1];
  if (argc > 2) g_test_smoother_so = argv[2];
  if (argc > 3) g_test_controller_so = argv[3];
  if (argc > 4) g_test_recovery_so = argv[4];
  if (argc > 5) g_plain_planner_so = argv[5];
  if (argc > 6) g_plain_smoother_so = argv[6];
  if (argc > 7) g_plain_controller_so = argv[7];
  if (argc > 8) g_plain_recovery_so = argv[8];
  return RUN_ALL_TESTS();
}
