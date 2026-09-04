// Task 20: CostmapNavigationServer (spec §5.4) — virtual initialize*Plugin 注入
// costmap_context + new*Execution 工厂 + configureCoreMaps + handleObstacle* 落图。
//
// 覆盖:
// - configureCoreMaps 建 global/local CostmapMapContext + StaticLayer/InflationLayer/SimpleObstacleLayer
// - loadPlugins 触发虚 initialize*Plugin → dynamic_pointer_cast<Costmap*> → configure(ctx, map)
// - fail case: 非 costmap 族插件 → initialize*Plugin false + loadPlugins 不建 action
// - new*Execution 返回 Costmap*Execution (绑 global/local wrapper)
// - handleObstaclePointCloud 经 TransformStore 落 SimpleObstacleLayer + InflationLayer 重算
//   (Ruling #7: 旧障碍与旧膨胀带随 clearArea 清掉, 无 stale leak)

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
using ayanami::ControllerCommand;
using ayanami::ControllerResultT;
using ayanami::PlannerResultT;
using ayanami::RecoveryCycleData;
using ayanami::RecoveryResultT;
using ayanami::RecoveryStatus;
using ayanami::SmootherResultT;
using ayanami::costmap::CostmapMapContext;
using ayanami::costmap::CostmapNavigationServer;

// 由 main() 从 argv 读取 (ctest 传各测试插件 $<TARGET_FILE:...>)。
std::string g_test_planner_so;
std::string g_test_smoother_so;
std::string g_test_controller_so;
std::string g_test_recovery_so;
std::string g_plain_planner_so;
std::string g_plain_smoother_so;
std::string g_plain_controller_so;
std::string g_plain_recovery_so;

// ---------------- 测试工具 ----------------

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

// 生成临时 config 目录: config.json global 段含 config_costmap; 4 族可各自
// 指定 costmap 族成功插件或非 costmap 族插件 (失败路径)。
std::filesystem::path makeConfigDir(
    const std::string& planner_so,
    const std::string& smoother_so,
    const std::string& controller_so,
    const std::string& recovery_so) {
  auto dir = makeTempDir() / ("cfg" + planner_so);
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  writeJson(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"navigation.json\",\n"
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

  // 20x20 全空 PGM 静态地图 (全部 255 -> FREE_SPACE)。配置故意声明 7x9，
  // 验证非 rolling global master 始终由 PGM 几何决定。
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

  // 20x20 @ 0.5m → 10m x 10m。local inflation_radius=1.5 → 3 cells。
  // costmap 参数并入 navigation.json (原 costmap.json 移除)。
  {
    std::ofstream nav(dir / "navigation.json");
    nav << "{\n  \"navigation\": {\n"
        << "    \"planner_id\": \"test_costmap_planner\",\n"
        << "    \"smoother_id\": \"test_costmap_smoother\",\n"
        << "    \"controller_id\": \"test_costmap_controller\",\n"
        << "    \"recovery_id\": \"test_costmap_recovery\"\n"
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

// 在 local wrapper 的 LayeredCostmap 里找 SimpleObstacleLayer (测试观察用)。
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

// 记录 initialize 注入状态的 TestCostmapPlanner (Task 20 接口断言)。
class TestCostmapPlanner : public ayanami::costmap::CostmapPlanner {
 public:
  std::string init_name;
  uint32_t plan_calls{0};

  virtual ~TestCostmapPlanner() override = default;

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr) override {
    init_name = context.name;
  }
  PlannerResultT makePlan(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::PoseStamped&, double) override {
    ++plan_calls;
    return PlannerResultT::success(ayanami::nav_msgs::Path{});
  }
};

// 非 costmap 族 planner (dynamic_pointer_cast<CostmapPlanner> 失败)。
class PlainAbstractPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  PlannerResultT makePlan(const ayanami::geometry_msgs::PoseStamped&,
                          const ayanami::geometry_msgs::PoseStamped&,
                          double) override {
    return PlannerResultT::success(ayanami::nav_msgs::Path{});
  }
};

// costmap 族 smoother/controller/recovery (接受路径 + initialize 注入断言)。
class TestCostmapSmoother : public ayanami::costmap::CostmapSmoother {
 public:
  std::string init_name;
  virtual ~TestCostmapSmoother() override = default;

  void configure(const ayanami::PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr) override {
    init_name = context.name;
  }
  SmootherResultT smoothPath(ayanami::nav_msgs::Path& path, double,
                             bool) override {
    return SmootherResultT::success(std::move(path));
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
  ControllerResultT computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::TwistStamped&) override {
    return ControllerResultT::success(ControllerCommand{});
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
  RecoveryResultT tick() override {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    return RecoveryResultT::success(std::move(data));
  }
};

// 非 costmap 族 smoother/controller/recovery (拒绝路径)。
class PlainAbstractSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  SmootherResultT smoothPath(ayanami::nav_msgs::Path& path, double,
                             bool) override {
    return SmootherResultT::success(std::move(path));
  }
};

class PlainAbstractController : public ayanami::abstract::AbstractController {
 public:
  ControllerResultT computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped&,
      const ayanami::geometry_msgs::TwistStamped&) override {
    return ControllerResultT::success(ControllerCommand{});
  }
  bool setPlan(const ayanami::nav_msgs::Path&) override { return true; }
};

class PlainAbstractRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  RecoveryResultT tick() override {
    RecoveryCycleData data;
    data.status = RecoveryStatus::SUCCESS;
    return RecoveryResultT::success(std::move(data));
  }
};

// 在 sensor frame (lidar) 里造一个单点 PointCloud2 (x/y/z FLOAT32, 12B/point)。
ayanami::sensor_msgs::PointCloud2 makePointCloud(float x, float y, float z) {
  ayanami::sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  cloud.height = 1;
  cloud.width = 1;
  cloud.point_step = 12;
  cloud.row_step = 12;
  cloud.is_dense = true;
  cloud.fields = {
      {"x", 0, ayanami::sensor_msgs::PointField::FLOAT32, 1},
      {"y", 4, ayanami::sensor_msgs::PointField::FLOAT32, 1},
      {"z", 8, ayanami::sensor_msgs::PointField::FLOAT32, 1},
  };
  cloud.data.resize(12);
  std::memcpy(cloud.data.data(), &x, sizeof(float));
  std::memcpy(cloud.data.data() + 4, &y, sizeof(float));
  std::memcpy(cloud.data.data() + 8, &z, sizeof(float));
  return cloud;
}

// 设置 TransformStore: map←odom←base_link←lidar, 机器人位于 odom 原点。
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
  EXPECT_DOUBLE_EQ(server.globalCostmap()->get().getCostmap()->getOriginX(),
                   -5.0);
  EXPECT_DOUBLE_EQ(server.globalCostmap()->get().getCostmap()->getOriginY(),
                   -5.0);
  EXPECT_EQ(server.globalCostmap()->get().getCostmap()->getSizeInCellsX(), 20u);
  EXPECT_EQ(server.globalCostmap()->get().getCostmap()->getSizeInCellsY(), 20u);
  // map_context 已注入 (AbstractMapContext 层); 位姿经 RobotStateController 解析。
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
  // 真实 config: costmap::robot_radius=0.12 → 16 边形近似圆 (nav_demo 语义)。
  CostmapNavigationServer server;
  server.configureCoreMaps(AYANAMI_TEST_CONFIG_DIR);
  const auto& footprint =
      server.globalCostmap()->get().getFootprint();
  ASSERT_EQ(footprint.size(), 16u);
  for (const auto& point : footprint) {
    EXPECT_NEAR(std::hypot(point.x, point.y), 0.12, 1e-9);
  }
  // 16 边形 inscribed = 0.12 * cos(pi/16) ≈ 0.1177。
  EXPECT_NEAR(server.globalCostmap()->get().getInscribedRadius(),
              0.12 * std::cos(M_PI / 16.0), 1e-6);
  EXPECT_NEAR(server.localCostmap()->get().getInscribedRadius(),
              0.12 * std::cos(M_PI / 16.0), 1e-6);
}

// ---------------- 虚 initialize*Plugin ----------------

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

  // 普通 AbstractController 不再放行 (移除 Task 12 stage C fallback)。
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
  CostmapNavigationServer server;  // 未 configureCoreMaps (Task 18 minor 1)
  auto planner = std::make_shared<TestCostmapPlanner>();
  EXPECT_FALSE(server.initializePlannerPlugin("p", ayanami::Config(""), planner));
  EXPECT_FALSE(server.initializeSmootherPlugin("s", ayanami::Config(""), nullptr));
}

// ---------------- loadPlugins 端到端 ----------------

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

  // planner dynamic_pointer_cast 失败 → 不入表 → plannerAction 未创建
  EXPECT_EQ(server.plannerAction(), nullptr);
  // 其余族正常装载
  ASSERT_NE(server.controllerAction(), nullptr);
  ASSERT_NE(server.smootherAction(), nullptr);
}

TEST(CostmapNavigationServerTest, loadPluginsRejectsAllPlainAbstractPlugins) {
  // 普通 abstract 四族全部拒绝 → 四个 action 都不建。
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

// ---------------- createActions 生产路径 = Costmap*Action ----------------
// 生产不再走基类 new*Execution 虚工厂间接层 (Server*Action 已删除):
// CostmapNavigationServer::createActions 直接构造 Costmap*Action, 其
// createExecution 产出带 CostmapMapContext 的 Costmap*Execution。

namespace {

// createExecution 是 protected, 测试经以下暴露包装类调用 (验证 Costmap*Action
// 生产路径产出 Costmap*Execution)。
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

  // 每个 action 都是 costmap 族 (生产路径凭证)。
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

  // Costmap*Action::createExecution → Costmap*Execution (带地图)。
  // createExecution 是 protected, 用测试内暴露包装类调用。
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

// ---------------- handleObstaclePointCloud (含 Ruling #7) ----------------

TEST(CostmapNavigationServerTest, handleObstaclePointCloudMarksAndInflates) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();

  // 单个点: lidar(0,0) → odom(0.1,0); local origin = robot(-5,-5) → cell(10,10)
  server.handleObstaclePointCloud(makePointCloud(0.0f, 0.0f, 0.0f));

  auto* master = server.localCostmap()->get().getCostmap();
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(10, 11), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  // 3 cells 内衰减带非零
  EXPECT_GT(master->getCost(10, 13), 0);

  // Planner consumes the global costmap, so the same physical obstacle must
  // be represented at the matching map-frame world coordinate there too.
  auto* global_master = server.globalCostmap()->get().getCostmap();
  unsigned int global_mx = 0;
  unsigned int global_my = 0;
  ASSERT_TRUE(global_master->worldToMap(0.1, 0.0, global_mx, global_my));
  EXPECT_EQ(global_master->getCost(global_mx, global_my),
            costmap_2d::LETHAL_OBSTACLE);

  // Ruling #7: 障碍移动 (lidar(2,0) → odom(2.1,0) → cell(14,10)) 后旧膨胀带消失
  server.handleObstaclePointCloud(makePointCloud(2.0f, 0.0f, 0.0f));

  // 旧障碍 cell + 旧膨胀带 cell 恢复 FREE (距新源 > 3 cells)
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::FREE_SPACE);
  EXPECT_EQ(master->getCost(10, 13), costmap_2d::FREE_SPACE);
  // 新位置障碍 + 新膨胀带
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

  auto plan_result = planner->makePlan(start, goal, 0.1);
  ASSERT_TRUE(plan_result.ok) << plan_result.message;
  const ayanami::nav_msgs::Path& path = plan_result.data;
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
  CostmapNavigationServer server;  // 未 configureCoreMaps
  setupTransforms();
  EXPECT_NO_THROW(server.handleObstaclePointCloud(makePointCloud(0, 0, 0)));
}

TEST(CostmapNavigationServerTest, obstacleStampQueriesTfAtMessageTime) {
  // 传感器 header.stamp 消费: TF 查询用消息时刻, 而非隐式 latest。
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto& rsc = ayanami::components::RobotStateController::instance();
  const auto ts = ayanami::timestampFromSeconds(123.0);
  // 与 setupTransforms 相同的数值, 但在具体时刻发布 (值一致, 跨测试无害)。
  rsc.setTransform("odom", "base_link", ts, Eigen::Isometry3d::Identity());
  rsc.setTransform("map", "odom", ts, Eigen::Isometry3d::Identity());
  rsc.setTransform("base_link", "lidar", ts,
                   Eigen::Translation3d(0.1, 0.0, 0.0) *
                       Eigen::Quaterniond::Identity());

  // 帧 stamp 命中 ts 时刻的 TF → 落图 (lidar(0,0) → odom(0.1,0) → cell(10,10))。
  auto cloud = makePointCloud(0.0f, 0.0f, 0.0f);
  cloud.header.stamp = ts;
  server.handleObstaclePointCloud(cloud);
  auto* master = server.localCostmap()->get().getCostmap();
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE);

  // 消息时刻 (ts+1s) 无对应 TF → lookup 失败 (按消息 stamp 查询), 不应用
  // 解析错误的变换, 上一帧标记原样保留 → 证明 stamp 被真实消费。
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
  scan.ranges = {1.0f};  // 单束沿 +x 1m → odom(1.1,0) → cell(12,10)
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

  // 障碍观测输入不走全局旁路槽 (输出/观测分层纪律): 经宿主直递的公共方法落图。
  server.handleObstaclePointCloud(makePointCloud(0.0f, 0.0f, 0.0f));

  auto* master = server.localCostmap()->get().getCostmap();
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapNavigationServerTest, concurrentObstacleHandlersDoNotCrash) {
  auto dir = makeConfigDir(g_test_planner_so, g_test_smoother_so, g_test_controller_so, g_test_recovery_so);
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();

  // 两线程并发 hammer 同一障碍位置: 共享 prev_obstacles_local 使 clear 窗口
  // 必覆盖本帧点 (obs 帧语义 = "当前观测覆盖旧观测"), 故用同一 cell 保证
  // 任何交错下的最后写入都是 LETHAL → 断言确定。锁的作用是串行化 marks/
  // clear_regions/prev_obstacles_local/grid 的并发改写 (无锁则 UB 可能崩)。
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
  // 作用域结束 → 析构: 基类 stop/reset/clearPlugins,
  // 不挂 (Ruling #3)。
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
