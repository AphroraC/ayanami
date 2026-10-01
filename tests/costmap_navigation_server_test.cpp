// CostmapNavigationServer -- virtual initialize*Module injection
// costmap_context + new*Execution factories + configureCoreMaps + handleObstacle* map updates.
//
// Coverage:
// - configureCoreMaps builds the global/local CostmapMapContext + StaticLayer/InflationLayer/SimpleObstacleLayer
// - loadModules triggers the virtual initialize*Module -> dynamic_pointer_cast<Costmap*> -> configure(ctx, map)
// - fail case: a non-costmap-family module -> initialize*Module false + loadModules creates no action
// - new*Execution returns Costmap*Execution (bound to the global/local wrapper)
// - handleObstaclePointCloud goes through the RobotStateController bridge into SimpleObstacleLayer + InflationLayer recompute
// (old obstacles and old inflation bands are cleared with clearArea; no stale leak)

#include <gtest/gtest.h>

#include "test_transform_source.hpp"

#include <chrono>
#include <functional>
#include <thread>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <regex>
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
#include <costmap_astar_planner/costmap_astar_planner.hpp>

using ayanami::testing::testTransforms;
namespace {

using ayanami::ErrorCode;
using ayanami::ResultT;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryStatus;
using ayanami::costmap::CostmapMapContext;
using ayanami::costmap::CostmapNavigationServer;

// Read by main from argv (ctest passes the $<TARGET_FILE:...> of each test module).
std::string g_test_planner_so;
std::string g_test_smoother_so;
std::string g_test_controller_so;
std::string g_test_recovery_so;
std::string g_plain_planner_so;
std::string g_plain_smoother_so;
std::string g_plain_controller_so;
std::string g_plain_recovery_so;

// ---------------- Test utilities ----------------

/// Bounded poll on a condition (2ms granularity).
static bool waitUntil(const std::function<bool()>& condition, int timeout_ms = 2000) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return condition();
}

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

std::string moduleJson(const std::string& module, const std::string& name,
                        const std::string& so_name) {
  // The module's own (empty) parameter section: the loader requires a section
  // named after the module, so that "this module reads no parameters" is
  // declared rather than indistinguishable from a name/section mismatch.
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + so_name + "\"\n  },\n  \"" + name +
         "\": {}\n}\n";
}

/// Generates a temp config dir: the config.json global section contains config_costmap; each of the 4 families can
/// point at either a costmap-family success module or a non-costmap module (failure path).
std::filesystem::path makeConfigDir(
    const std::string& planner_so,
    const std::string& smoother_so,
    const std::string& controller_so,
    const std::string& recovery_so,
    double scan_expected_update_rate = 0.0,
    double robot_radius = 0.0,
    bool global_obstacle = false,
    double local_update_hz = 10.0,
    double dump_frequency = 0.0) {
  auto dir = makeTempDir() / ("cfg" + planner_so);
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  // The config_* module file pointers live uniformly in the navigation section of navigation.json (single source).
  // The logging section points the costmap dump (and file logs) into the
  // temp dir so tests never write into the source tree.
  writeJson(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"navigation.json\"\n"
             "  },\n"
             "  \"logging\": {\n"
             "    \"logging_dir\": \"" + (dir / "logs").string() + "\"\n"
             "  }\n}\n");
  writeJson(dir / "navigation.json",
             "{\n  \"navigation\": {\n"
             "    \"config_planner\": \"planner.json\",\n"
             "    \"config_smoother\": \"smoother.json\",\n"
             "    \"config_controller\": \"controller.json\",\n"
             "    \"config_recovery\": [\"recovery.json\"]\n"
             "  }\n}\n");

  writeJson(dir / "planner.json",
             moduleJson("planner", "test_costmap_planner", planner_so));
  writeJson(dir / "smoother.json",
             moduleJson("smoother", "test_costmap_smoother", smoother_so));
  writeJson(dir / "controller.json",
             moduleJson("controller", "test_costmap_controller",
                         controller_so));
  writeJson(dir / "recovery.json",
             "{\n  \"recovery\": {\n    \"name\": \"test_costmap_recovery\",\n"
             "    \"role\": \"clear_costmap\",\n    \"so_name\": \"" +
             recovery_so +
             "\"\n  },\n  \"test_costmap_recovery\": {}\n}\n");

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
  // it must also carry the module file pointers.
  {
    std::ofstream nav(dir / "navigation.json");
    nav << "{\n  \"navigation\": {\n"
        << "    \"config_planner\": \"planner.json\",\n"
        << "    \"config_smoother\": \"smoother.json\",\n"
        << "    \"config_controller\": \"controller.json\",\n"
        << "    \"config_recovery\": [\"recovery.json\"]\n"
        << "  },\n"
        << "  \"costmap\": {\n"
        << "    \"dump_frequency\": " << dump_frequency << ",\n"
        << "    \"global_frame\": \"map\",\n"
        << "    \"local_frame\": \"odom\",\n"
        << "    \"robot_frame\": \"base_link\",\n"
        << "    \"resolution\": 0.5"
        << (robot_radius > 0.0
                ? ",\n    \"robot_radius\": " +
                      std::to_string(robot_radius)
                : std::string())
        << "\n  },\n"
        << "  \"costmap_global\": {\n"
        << "    \"update_frequency\": 5.0,\n"
        << "    \"rolling_window\": false,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"layers\": [\"layer_static.json\", "
        << (global_obstacle ? "\"layer_obstacle_global.json\", " : "")
        << "\"layer_inflation.json\"]\n"
        << "  },\n"
        << "  \"costmap_local\": {\n"
        << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
        << "    \"rolling_window\": true,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"transform_tolerance\": 0.1,\n"
        << "    \"update_frequency\": " << local_update_hz << ",\n"
        << "    \"observation_keep_time\": 0.5,\n"
        << "    \"layers\": [\"layer_obstacle.json\", \"layer_inflation.json\"]\n"
        << "  }\n"
        << "}\n";
  }
  // Layer module jsons (modules/ so+json; bare so names resolve next to
  // libayanami.so via the loader's core-relative fallback).
  writeJson(dir / "layer_static.json",
            "{\"layer\": {\"name\": \"static_layer\", \"so_name\": "
            "\"libayanami_costmap_static_layer.so\"},\n \"static_layer\": "
            "{\"static_map\": \"" + pgm_path + "\"}}\n");
  writeJson(dir / "layer_inflation.json",
            "{\"layer\": {\"name\": \"inflation_layer\", \"so_name\": "
            "\"libayanami_costmap_inflation_layer.so\"},\n \"inflation_layer\": "
            "{\"inflation_radius\": 1.5, \"inscribed_radius\": 1.0, "
            "\"cost_scaling_factor\": 2.0}}\n");
  const std::string scan_block =
      "\"scan\": {\"type\": \"laserscan\", \"expected_update_rate\": " +
      std::to_string(scan_expected_update_rate) + "}";
  writeJson(dir / "layer_obstacle.json",
            "{\"layer\": {\"name\": \"obstacle_layer\", \"so_name\": "
            "\"libayanami_costmap_obstacle_layer.so\"},\n \"obstacle_layer\": {"
            "\"combination\": \"overwrite\", \"footprint_clearing\": true, "
            "\"observation_sources\": [\"scan\", \"cloud\"], "
            + scan_block + ", "
            "\"cloud\": {\"type\": \"pointcloud\"}}}\n");
  writeJson(dir / "layer_obstacle_global.json",
            "{\"layer\": {\"name\": \"obstacle_layer\", \"so_name\": "
            "\"libayanami_costmap_obstacle_layer.so\"},\n \"obstacle_layer\": {"
            "\"combination\": \"overwrite\", \"footprint_clearing\": true, "
            "\"observation_sources\": [\"scan\", \"cloud\"], "
            + scan_block + ", "
            "\"cloud\": {\"type\": \"pointcloud\"}}}\n");
  return dir;
}

/// Finds the SimpleObstacleLayer inside the local wrapper LayeredCostmap (for test observation).
costmap_2d_layers::SimpleObstacleLayer* findObstacleLayer(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper) {
  for (const auto& layer : wrapper->get().getModules()) {
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

  void configure(const ayanami::ModuleContext& context,
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

  void configure(const ayanami::ModuleContext& context,
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

  void configure(const ayanami::ModuleContext& context,
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

  void configure(const ayanami::ModuleContext& context,
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

/// Sets up the bridge stub source: map<-odom<-base_link<-lidar, with the robot at the odom origin.
void setupTransforms() {
  auto& rsc = ayanami::components::RobotStateController::instance();
  testTransforms().set("odom", "base_link", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity());
  testTransforms().set("map", "odom", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity());
  testTransforms().set("base_link", "lidar", ayanami::Timestamp::latest(),
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
  EXPECT_EQ(server.costmapContext()->globalFrame(), "map");
  EXPECT_EQ(server.costmapContext()->robotFrame(), "base_link");
  // Frame single-source consistency: global wrapper in the global frame,
  // local wrapper in the local frame; RSC configured with the same names.
  {
    EXPECT_EQ(server.globalCostmap()->globalFrame(), "map");
    EXPECT_EQ(server.localCostmap()->globalFrame(),
              server.costmapContext()->localFrame());
    auto& rsc = ayanami::components::RobotStateController::instance();
    EXPECT_EQ(rsc.globalFrame(), server.costmapContext()->globalFrame());
    EXPECT_EQ(rsc.robotFrame(), server.costmapContext()->robotFrame());
  }
  EXPECT_DOUBLE_EQ(server.globalCostmap()->get().getCostmap()->getOriginX(),
                   -5.0);
  EXPECT_DOUBLE_EQ(server.globalCostmap()->get().getCostmap()->getOriginY(),
                   -5.0);
  EXPECT_EQ(server.globalCostmap()->get().getCostmap()->getSizeInCellsX(), 20u);
  EXPECT_EQ(server.globalCostmap()->get().getCostmap()->getSizeInCellsY(), 20u);
  // map_context is injected (AbstractMapContext layer); the pose resolves through RobotStateController.
  EXPECT_EQ(server.costmapContext()->robotFrame(), "base_link");

  // global: StaticLayer + InflationLayer ONLY (no dynamic obstacle layer:
  // dynamic marks live exclusively in the local odom-frame rolling map)
  bool has_static = false;
  bool has_global_obstacle = false;
  bool has_global_inflation = false;
  for (const auto& layer : server.globalCostmap()->get().getModules()) {
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
  EXPECT_FALSE(has_global_obstacle)
      << "the global map must be static+inflation only";
  EXPECT_TRUE(has_global_inflation);

  // local: SimpleObstacleLayer + InflationLayer
  bool has_obstacle = false;
  bool has_local_inflation = false;
  for (const auto& layer : server.localCostmap()->get().getModules()) {
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

// Shipped-tree tests below assert COSTMAP-family geometry/loading, but the
// active route (config.json::global.config_navigation) legitimately switches
// between families (e.g. rosemap) and configureCoreMaps follows it. Copy the
// tree and pin the costmap route, so the suite stays green for either active
// route (same pattern as config_consistency_test). The copy must be named
// "config": the route's relative asset paths resolve against the config
// path's PARENT directory.
const std::string& pinnedCostmapConfigDir() {
  static const std::string dir = [] {
    namespace fs = std::filesystem;
    const fs::path root =
        fs::temp_directory_path() / "ayanami_costmap_server_pinned";
    const fs::path pinned = root / "config";
    std::error_code ec;
    fs::remove_all(root, ec);
    fs::create_directories(pinned, ec);
    fs::copy(fs::path(AYANAMI_TEST_CONFIG_DIR), pinned,
             fs::copy_options::recursive, ec);
    const fs::path cfg = pinned / "config.json";
    std::ifstream ifs(cfg);
    std::string doc((std::istreambuf_iterator<char>(ifs)),
                    std::istreambuf_iterator<char>());
    ifs.close();
    const std::regex re("(\"config_navigation\"\\s*:\\s*\")[^\"]*(\")");
    const std::string patched =
        std::regex_replace(doc, re,
                           std::string("$1routes/costmap_navigation.json$2"));
    std::ofstream ofs(cfg);
    ofs << patched;
    return pinned.string();
  }();
  return dir;
}

TEST(CostmapNavigationServerTest, defaultConfigMatchesNavPgmGeometry) {
  CostmapNavigationServer server;
  server.configureCoreMaps(pinnedCostmapConfigDir());
  const auto* global = server.globalCostmap()->get().getCostmap();
  ASSERT_NE(global, nullptr);
  EXPECT_EQ(global->getSizeInCellsX(), 266u);
  EXPECT_EQ(global->getSizeInCellsY(), 159u);
  EXPECT_DOUBLE_EQ(global->getResolution(), 0.05);
  EXPECT_DOUBLE_EQ(global->getOriginX(), -8.75);
  EXPECT_DOUBLE_EQ(global->getOriginY(), -3.85);
}

TEST(CostmapNavigationServerTest, robotRadiusBuildsCircularFootprint) {
  // Real config (aligned to config_sentry): costmap::robot_radius=0.3 -> a
  // 16-gon approximating the circle (nav_demo semantics).
  CostmapNavigationServer server;
  server.configureCoreMaps(pinnedCostmapConfigDir());
  const auto& footprint =
      server.globalCostmap()->get().getFootprint();
  ASSERT_EQ(footprint.size(), 16u);
  for (const auto& point : footprint) {
    EXPECT_NEAR(std::hypot(point.x, point.y), 0.3, 1e-9);
  }
  // 16-gon inscribed = 0.3 * cos(pi/16) ~ 0.2939.
  EXPECT_NEAR(server.globalCostmap()->get().getInscribedRadius(),
              0.3 * std::cos(M_PI / 16.0), 1e-6);
  EXPECT_NEAR(server.localCostmap()->get().getInscribedRadius(),
              0.3 * std::cos(M_PI / 16.0), 1e-6);
}

// ---------------- Virtual initialize*Module ----------------

TEST(CostmapNavigationServerTest, initializePlannerModuleInjectsContext) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto planner = std::make_shared<TestCostmapPlanner>();
  ayanami::abstract::AbstractPlanner::Ptr base = planner;
  EXPECT_TRUE(server.initializePlannerModule("my_planner", ayanami::Config(""), base));

  EXPECT_EQ(planner->init_name, "my_planner");
}

TEST(CostmapNavigationServerTest, initializePlannerModuleRejectsNonCostmapPlanner) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto plain = std::make_shared<PlainAbstractPlanner>();
  EXPECT_FALSE(server.initializePlannerModule("plain", ayanami::Config(""), plain));
}

TEST(CostmapNavigationServerTest, initializeSmootherModuleInjectsContext) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto smoother = std::make_shared<TestCostmapSmoother>();
  ayanami::abstract::AbstractSmoother::Ptr base = smoother;
  EXPECT_TRUE(server.initializeSmootherModule("my_smoother", ayanami::Config(""), base));
  EXPECT_EQ(smoother->init_name, "my_smoother");
}

TEST(CostmapNavigationServerTest, initializeSmootherModuleRejectsNonCostmapSmoother) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto plain = std::make_shared<PlainAbstractSmoother>();
  EXPECT_FALSE(server.initializeSmootherModule("plain", ayanami::Config(""), plain));
}

TEST(CostmapNavigationServerTest, initializeControllerModuleInjectsContext) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto controller = std::make_shared<TestCostmapController>();
  ayanami::abstract::AbstractController::Ptr base = controller;
  EXPECT_TRUE(server.initializeControllerModule("my_controller", ayanami::Config(""), base));
  EXPECT_EQ(controller->init_name, "my_controller");
}

TEST(CostmapNavigationServerTest, initializeControllerModuleRejectsNonCostmapController) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  // A plain AbstractController is no longer admitted (the fallback was removed).
  auto plain = std::make_shared<PlainAbstractController>();
  EXPECT_FALSE(server.initializeControllerModule("plain", ayanami::Config(""), plain));
}

TEST(CostmapNavigationServerTest, initializeRecoveryModuleInjectsContext) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto recovery = std::make_shared<TestCostmapRecovery>();
  ayanami::abstract::AbstractRecovery::Ptr base = recovery;
  EXPECT_TRUE(server.initializeRecoveryModule("my_recovery", ayanami::Config(""), base));
  EXPECT_EQ(recovery->init_name, "my_recovery");
}

TEST(CostmapNavigationServerTest, initializeRecoveryModuleRejectsNonCostmapRecovery) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto plain = std::make_shared<PlainAbstractRecovery>();
  EXPECT_FALSE(server.initializeRecoveryModule("plain", ayanami::Config(""), plain));
}

TEST(CostmapNavigationServerTest, initializePlannerModuleFailsWithoutConfiguredMaps) {
  CostmapNavigationServer server;  // configureCoreMaps not called (minor 1)
  auto planner = std::make_shared<TestCostmapPlanner>();
  EXPECT_FALSE(server.initializePlannerModule("p", ayanami::Config(""), planner));
  EXPECT_FALSE(server.initializeSmootherModule("s", ayanami::Config(""), nullptr));
}

// ---------------- loadModules end to end ----------------

TEST(CostmapNavigationServerTest, loadModulesLoadsCostmapModulesAndCreatesActions) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  server.loadModules(dir.string());

  ASSERT_NE(server.plannerAction(), nullptr);
  ASSERT_NE(server.smootherAction(), nullptr);
  ASSERT_NE(server.controllerAction(), nullptr);
  ASSERT_NE(server.recoveryAction("test_costmap_recovery"), nullptr);
  ASSERT_EQ(server.recoveryAction("missing"), nullptr);
}

// A module the config explicitly names must load: a stack that silently comes
// up without its configured planner only fails later, as an unexplained
// INVALID_INPUT when a goal arrives. The rejection is therefore fatal at
// startup AND the message must name the module and the reason.
TEST(CostmapNavigationServerTest, loadModulesRejectedByFamilyHookIsFatal) {
  auto dir = makeConfigDir(g_plain_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  try {
    server.loadModules(dir.string());
    FAIL() << "a configured planner that is not a CostmapPlanner must abort loading";
  } catch (const std::runtime_error& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("planner"), std::string::npos) << what;
    // The module loaded as an AbstractPlanner, so the manager's own type check
    // passed; the refusal comes from the costmap family's init hook (the
    // family-level cast lives there, next to the map it injects), and the
    // manager reports it as such.
    EXPECT_NE(what.find("rejected by the family init hook"), std::string::npos)
        << what;
    EXPECT_NE(what.find("test_costmap_planner"), std::string::npos) << what;
  }
  // Nothing was registered for the family that failed.
  EXPECT_EQ(server.plannerAction(), nullptr);
}

TEST(CostmapNavigationServerTest, loadModulesRejectsAllPlainAbstractModules) {
  // All four families are plain abstract modules -> the first one (planner)
  // already aborts loading; none of the four actions exists afterwards.
  auto dir = makeConfigDir(g_plain_planner_so, g_plain_smoother_so,
                             g_plain_controller_so, g_plain_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  EXPECT_THROW(server.loadModules(dir.string()), std::runtime_error);

  EXPECT_EQ(server.plannerAction(), nullptr);
  EXPECT_EQ(server.smootherAction(), nullptr);
  EXPECT_EQ(server.controllerAction(), nullptr);
  EXPECT_EQ(server.recoveryAction("test_costmap_recovery"), nullptr);
}

// An unopenable .so is the clearest deployment error: the config names a module
// that is not installed. It must be reported as such (not as a type mismatch).
TEST(CostmapNavigationServerTest, loadModulesWithMissingSoIsFatalAndNamed) {
  auto dir = makeConfigDir("libayanami_nonexistent_module.so", g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  try {
    server.loadModules(dir.string());
    FAIL() << "a configured planner whose .so is absent must abort loading";
  } catch (const std::runtime_error& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("dlopen/factory failed"), std::string::npos) << what;
  }
  EXPECT_EQ(server.plannerAction(), nullptr);
}

// The shipped configuration must actually load all families through the real
// path. This is not a tautology: no other test puts config/ through loadModules
// (the other tests build temp configs), so a shipped module json naming a .so
// that is not deployed, a module whose interface version drifted, or a parameter
// key that no longer exists would otherwise only surface on the robot.
TEST(CostmapNavigationServerTest, shippedConfigLoadsAllFamilies) {
  CostmapNavigationServer server;
  server.configureCoreMaps(pinnedCostmapConfigDir());
  ASSERT_NO_THROW(server.loadModules(pinnedCostmapConfigDir()))
      << "the shipped config must load: see the critical log for the reason";

  EXPECT_NE(server.plannerAction(), nullptr);
  EXPECT_NE(server.smootherAction(), nullptr);
  EXPECT_NE(server.controllerAction(), nullptr);
  EXPECT_FALSE(server.recoveryActionNames().empty());
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
  server.loadModules(dir.string());

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
  server.loadModules(dir.string());

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

  // Single point: lidar(1.1,0) -> odom(1.2,0); local origin = robot(-5,-5)
  // -> cell (12,10). The hit doubles as its own ray endpoint, so the
  // corridor cells before it are freed.
  server.handleObstaclePointCloud(makePointCloud(1.1f, 0.0f, 0.0f));

  auto* master = server.localCostmap()->get().getCostmap();
  EXPECT_EQ(master->getCost(12, 10), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(12, 11), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  // The decay band within 3 cells is non-zero
  EXPECT_GT(master->getCost(12, 13), 0);

  // Map-representation policy: the global map is static+inflation ONLY, so
  // the same physical obstacle must NOT appear there (planner reads the
  // global map; fresh observations never leak into the planning timescale).
  auto* global_master = server.globalCostmap()->get().getCostmap();
  unsigned int global_mx = 0;
  unsigned int global_my = 0;
  ASSERT_TRUE(global_master->worldToMap(1.2, 0.0, global_mx, global_my));
  EXPECT_EQ(global_master->getCost(global_mx, global_my),
            costmap_2d::FREE_SPACE);

  // Obstacle elsewhere (lidar(0,3) -> odom(0.1,3) -> cell (10,16)): the new
  // ray runs vertically at cell column 10 and never passes through the old
  // mark (12,10) -- occlusion persistence, no wholesale window wipe.
  server.handleObstaclePointCloud(makePointCloud(0.0f, 3.0f, 0.0f));
  EXPECT_EQ(master->getCost(12, 10), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(10, 16), costmap_2d::LETHAL_OBSTACLE);

  // After the obstacle moves (lidar(2.9,0) -> odom(3.0,0) -> cell(16,10)),
  // the new ray passes THROUGH the old mark and retires it with its
  // inflation band (the new mark sits 4 cells out, beyond the old band).
  server.handleObstaclePointCloud(makePointCloud(2.9f, 0.0f, 0.0f));

  // The old obstacle cell + old inflation cells return to FREE (more than 3 cells from the new source)
  EXPECT_EQ(master->getCost(12, 10), costmap_2d::FREE_SPACE);
  EXPECT_EQ(master->getCost(12, 13), costmap_2d::FREE_SPACE);
  // Obstacle at the new position + new inflation band
  EXPECT_EQ(master->getCost(16, 10), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(16, 11), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  // The global map is untouched by the whole observation stream.
  EXPECT_EQ(global_master->getCost(global_mx, global_my),
            costmap_2d::FREE_SPACE);
}

TEST(CostmapNavigationServerTest, globalObstacleLayerStampsClearsAndInvalidatesPaths) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so,
                           /*scan_expected_update_rate=*/0.0,
                           /*robot_radius=*/0.0,
                           /*global_obstacle=*/true);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();

  // The same observation is stamped into BOTH maps (world point (0.1, 0)).
  server.handleObstaclePointCloud(makePointCloud(0.0f, 0.0f, 0.0f));
  auto* global_master = server.globalCostmap()->get().getCostmap();
  unsigned int gmx = 0;
  unsigned int gmy = 0;
  ASSERT_TRUE(global_master->worldToMap(0.1, 0.0, gmx, gmy));
  EXPECT_EQ(global_master->getCost(gmx, gmy), costmap_2d::LETHAL_OBSTACLE);
  // The global mark inflates (the obstacle layer sits under the inflation
  // layer), so the planner's cost gradient sees it too.
  EXPECT_EQ(global_master->getCost(gmx, gmy + 1),
            costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(server.localCostmap()->get().getCostmap()->getCost(10, 10),
            costmap_2d::LETHAL_OBSTACLE);

  // A path crossing the global mark is invalid on the planning timescale.
  auto pose = [](double x, double y) {
    auto p = ayanami::geometry_msgs::PoseStamped::fromIsometry(
        "map", Eigen::Isometry3d::Identity());
    p.pose.position = Eigen::Vector3d(x, y, 0.0);
    return p;
  };
  ayanami::nav_msgs::Path blocked;
  blocked.poses = {pose(-1.0, 0.0), pose(0.1, 0.0), pose(1.0, 0.0)};
  EXPECT_FALSE(server.isPathValid(blocked));
  ayanami::nav_msgs::Path clear_path;
  clear_path.poses = {pose(-1.0, 3.0), pose(0.1, 3.0), pose(1.0, 3.0)};
  EXPECT_TRUE(server.isPathValid(clear_path));

  // Clearing the global map retires the dynamic marks and the StaticLayer
  // restores the base map; the local map is untouched.
  ayanami::costmap::ClearCostmapCommand command;
  command.affected_maps = "global";
  command.clear_radius = 3.0;
  ASSERT_TRUE(server.executeClearCommand(command));
  EXPECT_EQ(global_master->getCost(gmx, gmy), costmap_2d::FREE_SPACE);
  EXPECT_EQ(global_master->getCost(gmx, gmy + 1), costmap_2d::FREE_SPACE);
  EXPECT_EQ(server.localCostmap()->get().getCostmap()->getCost(10, 10),
            costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapNavigationServerTest, astarPlanOverlaysLocalObstacle) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();

  // Place the observation directly between start and goal in the map frame.
  // Static-global/overlay policy: the global grid carries static+inflation
  // only, and a fresh LOCAL observation must still shape the plan -- the
  // planner fuses its held global copy with a fresh local snapshot
  // (FusedMapView) and routes around the local obstacle's blocking disc.
  server.handleObstaclePointCloud(makePointCloud(0.0f, 0.0f, 0.0f));

auto planner = std::make_shared<ayanami::modules::CostmapAstarPlanner>();
  planner->configure(ayanami::ModuleContext{"astar", ayanami::Config("")},
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
  EXPECT_TRUE(deviated_from_straight_line)
      << "the overlay planner must route around the local-only obstacle";
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
  // Same values as setupTransforms but published at specific times (identical values, different stamps).
  testTransforms().set("odom", "base_link", ts, Eigen::Isometry3d::Identity());
  testTransforms().set("map", "odom", ts, Eigen::Isometry3d::Identity());
  testTransforms().set("base_link", "lidar", ts,
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
  testTransforms().set("odom", "base_link", ts, Eigen::Isometry3d::Identity());
  testTransforms().set("map", "odom", ts, Eigen::Isometry3d::Identity());
  testTransforms().set("base_link", "lidar", ts,
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

// ---- height filter runs in the MAP frame (nav2 semantics) ----
// A pitched sensor: points whose SENSOR-frame z is far outside the height
// window but whose MAP-frame z is inside must still be marked. Filtering in
// the sensor frame (the old behavior) dropped whole sectors of the cloud for
// any non-level mounting.

TEST(CostmapNavigationServerTest, heightFilterUsesMapFrameZ) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto& rsc = ayanami::components::RobotStateController::instance();
  testTransforms().set("odom", "base_link", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity());
  testTransforms().set("map", "odom", ayanami::Timestamp::latest(),
                   Eigen::Isometry3d::Identity());
  // Lidar pitched 135 degrees about Y: the sensor +z axis points up-back
  // (map: +x 0.707, +z -0.707). A sensor point (0, 0, -0.4) then transforms
  // to map (0.1 - 0.283, 0, +0.283): MAP-frame z = 0.28, inside the [0, 1]
  // height window, while its SENSOR-frame z = -0.4 is outside -- the old
  // sensor-frame filter dropped the point outright.
  Eigen::Isometry3d pitched = Eigen::Isometry3d::Identity();
  pitched.linear() = (Eigen::AngleAxisd(3.0 * M_PI / 4.0,
                                        Eigen::Vector3d::UnitY()))
                         .toRotationMatrix();
  pitched.translation() = Eigen::Vector3d(0.1, 0.0, 0.0);
  // A dedicated child frame: the stub source never overwrites an existing
  // dynamic edge sample, and earlier tests in this binary already pin
  // base_link<-lidar; use a fresh frame for the pitched transform.
  testTransforms().set("base_link", "lidar2", ayanami::Timestamp::latest(),
                       pitched);

  ayanami::sensor_msgs::PointCloud2 cloud =
      makePointCloud(0.0f, 0.0f, -0.4f);
  cloud.header.frame_id = "lidar2";
  server.handleObstaclePointCloud(cloud);
  auto* local = server.localCostmap()->get().getCostmap();
  unsigned int mx = 0;
  unsigned int my = 0;
  // map x = 0.1 + z_s * sin(135 deg) = 0.1 - 0.283 = -0.183
  ASSERT_TRUE(local->worldToMap(-0.1828, 0.0, mx, my));
  EXPECT_EQ(local->getCost(mx, my), costmap_2d::LETHAL_OBSTACLE)
      << "a point inside the map-frame height window must be marked even "
         "when its sensor-frame z is outside";
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

// ---- resident map-writer thread: async mailbox + single writer ----

TEST(CostmapNavigationServerTest, writerThreadAppliesObservationsAsync) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();
  server.start();

  // Async submission: the caller returns at O(enqueue) cost; the resident
  // writer applies the observation and waitForMapIdle observes it.
  server.handleObstaclePointCloud(makePointCloud(1.0f, 0.0f, 0.0f));
  EXPECT_TRUE(server.waitForMapIdle(2.0));
  auto* local = server.localCostmap()->get().getCostmap();
  EXPECT_EQ(local->getCost(12, 10), costmap_2d::LETHAL_OBSTACLE);

  server.stop();
}

// ---- sensor liveness: a silent source marks the map not-current ----

TEST(CostmapNavigationServerTest, silentSourceMarksMapNotCurrent) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so,
                           /*scan_expected_update_rate=*/0.2);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();
  server.start();
  ASSERT_TRUE(server.waitForMapIdle(2.0));

  // While the scan stream flows (one observation just applied), the map is
  // current. The source carries expected_update_rate = 0.2s.
  ayanami::sensor_msgs::LaserScan scan;
  scan.header.frame_id = "lidar";
  scan.angle_min = 0.0f;
  scan.angle_max = 0.0f;
  scan.angle_increment = 0.0f;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges = {5.0f};
  server.handleObstacleLaserScan(scan);
  ASSERT_TRUE(server.waitForMapIdle(2.0));
  EXPECT_TRUE(server.localCostmap()->isCurrent());

  // Silence beyond the expected rate -> the writer loop marks the map
  // not-current (surfaced downstream as MAP_STALE -> BT recovery).
  EXPECT_TRUE(waitUntil([&] { return !server.localCostmap()->isCurrent(); },
                         2000));

  // Fresh data re-marks the map current.
  server.handleObstacleLaserScan(scan);
  ASSERT_TRUE(server.waitForMapIdle(2.0));
  EXPECT_TRUE(server.localCostmap()->isCurrent());

  server.stop();
}

// ---- footprint clearing: the robot can never mark itself ----

TEST(CostmapNavigationServerTest, footprintClearingRemovesSelfHits) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so,
                           /*scan_expected_update_rate=*/0.0,
                           /*robot_radius=*/0.3);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();

  // One hit INSIDE the 0.3m footprint (base_link + (0.2, 0) -> odom
  // (0.3, 0) -> cell (11, 10)) and one OUTSIDE (base_link + (1.0, 0) ->
  // odom (1.1, 0) -> cell (12, 10)).
  server.handleObstaclePointCloud(makePointCloud(0.2f, 0.0f, 0.0f));
  server.handleObstaclePointCloud(makePointCloud(1.0f, 0.0f, 0.0f));
  auto* local = server.localCostmap()->get().getCostmap();
  // nav2 semantics: footprint clearing runs AFTER marking, so the self hit
  // always loses to the clear (its cell may still carry the far hit's
  // inflation decay, but never a lethal mark).
  EXPECT_NE(local->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE)
      << "the self hit inside the 0.3m footprint must be cleared";
  EXPECT_EQ(local->getCost(12, 10), costmap_2d::LETHAL_OBSTACLE)
      << "the hit outside the footprint must be marked";
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
    server.loadModules(dir.string());

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
  // End of scope -> destruction: base-class stop/reset/clearModules,
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

TEST(CostmapNavigationServerTest, writerBatchAppliesEveryQueuedFrame) {
  // P1 regression: frames queued while the writer is between cycles are
  // consumed as one batch (drain-once) -- every mark lands, and the map is
  // current after a single waitForMapIdle. Three beams in different
  // directions: no ray of one frame crosses another frame's endpoint, so
  // the three marks must coexist.
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();
  server.start();

  const auto makeBeam = [](float angle, float range) {
    ayanami::sensor_msgs::LaserScan scan;
    scan.header.frame_id = "lidar";
    scan.angle_min = angle;
    scan.angle_max = angle;
    scan.angle_increment = 0.0f;
    scan.range_min = 0.1f;
    scan.range_max = 10.0f;
    scan.ranges = {range};
    return scan;
  };
  server.handleObstacleLaserScan(makeBeam(0.0f, 3.0f));    // odom (3.1, 0)
  server.handleObstacleLaserScan(makeBeam(3.14159265f, 3.0f));  // odom (-2.9, 0)
  server.handleObstacleLaserScan(makeBeam(1.57079633f, 3.0f));  // odom (0.1, 3)
  ASSERT_TRUE(server.waitForMapIdle(2.0));
  // The endpoint world coordinates land on cell boundaries; float scan
  // geometry can put the mark in the nominal cell or its neighbor, so the
  // assertion accepts either (254 mark, or 253 inflated neighbor -- both
  // prove a source was stamped there).
  const auto blocked = [](costmap_2d::CostValue cost) {
    return cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  };
  auto* local = server.localCostmap()->get().getCostmap();
  EXPECT_TRUE(blocked(local->getCost(16, 10)));
  EXPECT_TRUE(blocked(local->getCost(4, 10)));
  EXPECT_TRUE(blocked(local->getCost(10, 16)));
  EXPECT_TRUE(server.localCostmap()->isCurrent());

  server.stop();
}
TEST(CostmapNavigationServerTest, stampedFrameWithoutStampTfIsDropped) {
  // R3 regression: a frame that CARRIES a stamp but cannot resolve it at
  // stamp time is DROPPED whole (nav2 parity). The old clear-only fallback
  // erased the ray-endpoint obstacles the frame just saw; dropping keeps
  // the map stale-but-stable. A latest-time-by-construction frame keeps
  // marking.
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.setTransformSource(
      [](const std::string& target, const std::string& source,
         const ayanami::Timestamp& stamp, Eigen::Isometry3d& transform,
         double) {
        if (!stamp.isLatest()) {
          return false;  // stamp-time lookups unavailable
        }
        return testTransforms().lookup(target, source, transform);
      });

  // Latest-time frame first: a mark exists at cell (10,10)
  // (lidar(0,0) -> odom(0.1,0)).
  ayanami::sensor_msgs::PointCloud2 fresh = makePointCloud(0.0f, 0.0f, 0.0f);
  fresh.header.stamp = ayanami::Timestamp::latest();
  server.handleObstaclePointCloud(fresh);
  auto* master = server.localCostmap()->get().getCostmap();
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE);

  // Stamped frame, stamp unresolvable: dropped ENTIRELY -- its ray (which
  // would pass through the existing mark, since the hit sits beyond it on
  // the same line) must neither clear nor mark anything.
  ayanami::sensor_msgs::PointCloud2 degraded = makePointCloud(2.0f, 0.0f, 0.0f);
  degraded.header.stamp =
      ayanami::Timestamp(std::chrono::nanoseconds(1000000000));
  server.handleObstaclePointCloud(degraded);
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(14, 10), costmap_2d::FREE_SPACE);

  // Restore the standard stamp-agnostic stub for later tests.
  setupTransforms();
}

TEST(CostmapNavigationServerTest, observationBufferOverflowDropsOldestAndCounts) {
  // R4 regression: the bounded observation buffer must drop the OLDEST
  // frame under a sustained stall and surface it through the counter (the
  // throttled warning is the field-visible part). A slow local cycle
  // (0.5 Hz -> 2 s gate) makes the overflow deterministic: the first cycle
  // drains once, then 12 submissions pile up against the cap of 8.
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so,
                           /*scan_expected_update_rate=*/0.0,
                           /*robot_radius=*/0.0, /*global_obstacle=*/false,
                           /*local_update_hz=*/0.5);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();
  server.start();

  // Prime one real writer cycle first: waitForMapIdle alone can return
  // while the writer has not run its first cycle yet (the queue starts
  // empty), which would interleave the drain below with the submissions
  // and zero the drop counter. One priming frame pins the first cycle and
  // its update_frequency gate; the 12 submissions then land strictly
  // inside the 2 s gate window.
  server.handleObstaclePointCloud(makePointCloud(1.0f, 0.0f, 0.0f));
  ASSERT_TRUE(server.waitForMapIdle(2.0));

  for (int i = 0; i < 12; ++i) {
    server.handleObstaclePointCloud(makePointCloud(1.0f, 0.0f, 0.0f));
  }
  EXPECT_EQ(server.observationsDropped(), 4u)
      << "12 frames against the cap of 8 must drop exactly the 4 oldest";
  // The next cycle (2 s gate) consumes what survived; the aged-out frames
  // are skipped without processing.
  EXPECT_TRUE(server.waitForMapIdle(5.0));

  server.stop();
}

TEST(CostmapNavigationServerTest, periodicDumpWritesPgmAndMetadata) {
  // Dump-cycle regression: at dump_frequency the writer thread writes both
  // master grids as PGM + sidecar into the logging dir (the field
  // diagnostics path; production runs the same code at 0.1 Hz).
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so,
                           g_test_controller_so, g_test_recovery_so,
                           /*scan_expected_update_rate=*/0.0,
                           /*robot_radius=*/0.0, /*global_obstacle=*/false,
                           /*local_update_hz=*/10.0, /*dump_frequency=*/50.0);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();
  server.start();

  // Mark one obstacle so the local dump has a lethal cell to show.
  server.handleObstaclePointCloud(makePointCloud(1.0f, 0.0f, 0.0f));
  ASSERT_TRUE(server.waitForMapIdle(2.0));
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
  bool global_seen = false;
  bool local_seen = false;
  while (std::chrono::steady_clock::now() < deadline &&
         !(global_seen && local_seen)) {
    std::error_code ec;
    global_seen = std::filesystem::exists(dir / "logs" / "costmap_global.pgm",
                                          ec);
    local_seen = std::filesystem::exists(dir / "logs" / "costmap_local.pgm",
                                         ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(global_seen) << "global costmap PGM was never dumped";
  ASSERT_TRUE(local_seen) << "local costmap PGM was never dumped";

  // Sidecars parse and carry the static-layer metadata schema.
  std::ifstream meta((dir / "logs" / "costmap_global.json").string());
  ASSERT_TRUE(meta);
  std::string content((std::istreambuf_iterator<char>(meta)),
                      std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("\"resolution\""), std::string::npos);
  EXPECT_NE(content.find("\"origin\""), std::string::npos);

  server.stop();
  std::filesystem::remove_all(dir / "logs");
}
