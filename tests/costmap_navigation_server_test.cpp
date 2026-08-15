// Task 20: CostmapNavigationServer (spec §5.4) — virtual initialize*Plugin 注入
// costmap_context_ + new*Execution 工厂 + configureCoreMaps + handleObstacle* 落图。
//
// 覆盖:
// - configureCoreMaps 建 global/local CostmapContext + StaticLayer/InflationLayer/SimpleObstacleLayer
// - loadPlugins 触发虚 initialize*Plugin → dynamic_pointer_cast<Costmap*> → initialize(name, ctx)
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
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d_layers/inflation_layer.hpp>
#include <costmap_2d_layers/simple_obstacle_layer.hpp>
#include <costmap_2d_layers/static_layer.hpp>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/components/costmap_controller_execution.hpp>
#include <ayanami/costmap/components/costmap_navigation_server.hpp>
#include <ayanami/costmap/components/costmap_planner_execution.hpp>
#include <ayanami/costmap/components/costmap_recovery_execution.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/messages/sensor_msgs.hpp>

namespace {

using ayanami::costmap::CostmapContext;
using ayanami::costmap::CostmapNavigationServer;

std::string g_plugins_so;

// ---------------- 测试工具 ----------------

std::filesystem::path make_temp_dir() {
  std::filesystem::path dir = std::filesystem::temp_directory_path() /
                              "ayanami_costmap_navigation_server_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void write_json(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

std::string plugin_json(const std::string& module, const std::string& name,
                        const std::string& symbol) {
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + g_plugins_so +
         "\",\n    \"create_symbol\": \"" + symbol + "\"\n  }\n}\n";
}

// 生成临时 config 目录: config.json global 段含 config_costmap; planner 符号
// 可选 costmap 族成功工厂或非 costmap 族工厂 (失败路径)。
std::filesystem::path make_config_dir(const std::string& planner_symbol) {
  auto dir = make_temp_dir() / ("cfg_" + planner_symbol);
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  write_json(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"navigation.json\",\n"
             "    \"config_planner\": \"planner.json\",\n"
             "    \"config_smoother\": \"smoother.json\",\n"
             "    \"config_controller\": \"controller.json\",\n"
             "    \"config_recovery\": \"recovery.json\",\n"
             "    \"config_costmap\": \"costmap.json\"\n"
             "  }\n}\n");
  write_json(dir / "navigation.json",
             "{\n  \"navigation\": {\n"
             "    \"planner_id\": \"test_costmap_planner\",\n"
             "    \"smoother_id\": \"test_costmap_smoother\",\n"
             "    \"controller_id\": \"test_costmap_controller\",\n"
             "    \"recovery_id\": \"test_costmap_recovery\"\n"
             "  }\n}\n");
  write_json(dir / "planner.json",
             plugin_json("planner", "test_costmap_planner", planner_symbol));
  write_json(dir / "smoother.json",
             plugin_json("smoother", "test_costmap_smoother",
                         "create_test_costmap_smoother_plugin"));
  write_json(dir / "controller.json",
             plugin_json("controller", "test_costmap_controller",
                         "create_test_costmap_controller_plugin"));
  write_json(dir / "recovery.json",
             plugin_json("recovery", "test_costmap_recovery",
                         "create_test_costmap_recovery_plugin"));

  // 10x10 全空 PGM 静态地图 (全部 255 -> FREE_SPACE)。
  const std::string pgm_path = (dir / "map.pgm").string();
  {
    std::ofstream out(pgm_path, std::ios::binary);
    out << "P5\n10 10\n255\n";
    for (int i = 0; i < 100; ++i) {
      out.put(static_cast<char>(255));
    }
  }

  // 20x20 @ 0.5m → 10m x 10m。local inflation_radius=1.5 → 3 cells。
  write_json(dir / "costmap.json",
             "{\n  \"costmap\": {\n"
             "    \"global_frame\": \"map\",\n"
             "    \"local_frame\": \"odom\",\n"
             "    \"robot_frame\": \"base_link\",\n"
             "    \"resolution\": 0.5\n"
             "  },\n"
             "  \"costmap_global\": {\n"
             "    \"size_x\": 20,\n    \"size_y\": 20,\n"
             "    \"rolling_window\": false,\n"
             "    \"track_unknown\": false,\n"
             "    \"static_map\": \"" +
             pgm_path +
             "\",\n"
             "    \"inflation_radius\": 1.5,\n"
             "    \"inscribed_radius\": 1.0,\n"
             "    \"cost_scaling_factor\": 2.0\n"
             "  },\n"
             "  \"costmap_local\": {\n"
             "    \"size_x\": 20,\n    \"size_y\": 20,\n"
             "    \"rolling_window\": true,\n"
             "    \"track_unknown\": false,\n"
             "    \"inflation_radius\": 1.5,\n"
             "    \"inscribed_radius\": 1.0,\n"
             "    \"cost_scaling_factor\": 2.0\n"
             "  }\n"
             "}\n");
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
  std::string init_name_;
  CostmapContext::Ptr init_ctx_;
  uint32_t plan_calls_{0};

  virtual ~TestCostmapPlanner() override = default;

  void initialize(const std::string& name, CostmapContext::Ptr ctx) override {
    init_name_ = name;
    init_ctx_ = std::move(ctx);
  }
  uint32_t makePlan(const ayanami::geometry_msgs::PoseStamped&,
                    const ayanami::geometry_msgs::PoseStamped&, double,
                    ayanami::nav_msgs::Path&, double&, std::string&) override {
    ++plan_calls_;
    return 0;
  }
};

// 非 costmap 族 planner (dynamic_pointer_cast<CostmapPlanner> 失败)。
class PlainAbstractPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  uint32_t makePlan(const ayanami::geometry_msgs::PoseStamped&,
                    const ayanami::geometry_msgs::PoseStamped&, double,
                    ayanami::nav_msgs::Path&, double&, std::string&) override {
    return 0;
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
  auto& store = ayanami::interfaces::TransformStore::instance();
  const auto t0 = ayanami::interfaces::TransformStore::fromSeconds(0.0);
  store.setTransform("odom", "base_link", t0,
                     Eigen::Isometry3d::Identity());
  store.setTransform("map", "odom", t0, Eigen::Isometry3d::Identity());
  store.setTransform("base_link", "lidar", t0,
                     Eigen::Translation3d(0.1, 0.0, 0.0) *
                         Eigen::Quaterniond::Identity());
}

}  // namespace

// ---------------- configureCoreMaps ----------------

TEST(CostmapNavigationServerTest, ConfigureCoreMapsBuildsContextAndLayers) {
  auto dir = make_config_dir("create_test_costmap_planner_plugin");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  ASSERT_NE(server.costmapContext(), nullptr);
  ASSERT_NE(server.globalCostmap(), nullptr);
  ASSERT_NE(server.localCostmap(), nullptr);
  EXPECT_EQ(server.costmapContext()->getGlobalFrame(), "map");
  EXPECT_EQ(server.costmapContext()->getRobotFrame(), "base_link");
  // map_context_ 已注入 (AbstractMapContext 层)
  EXPECT_EQ(server.costmapContext()->transform_store_,
            &ayanami::interfaces::TransformStore::instance());

  // global: StaticLayer + InflationLayer
  bool has_static = false;
  bool has_global_inflation = false;
  for (const auto& layer : server.globalCostmap()->get().getPlugins()) {
    if (dynamic_cast<costmap_2d_layers::StaticLayer*>(layer.get())) {
      has_static = true;
    }
    if (dynamic_cast<costmap_2d_layers::InflationLayer*>(layer.get())) {
      has_global_inflation = true;
    }
  }
  EXPECT_TRUE(has_static);
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

// ---------------- 虚 initialize*Plugin ----------------

TEST(CostmapNavigationServerTest, InitializePlannerPluginInjectsContext) {
  auto dir = make_config_dir("create_test_costmap_planner_plugin");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto planner = std::make_shared<TestCostmapPlanner>();
  ayanami::abstract::AbstractPlanner::Ptr base = planner;
  EXPECT_TRUE(server.initializePlannerPlugin("my_planner", base));

  EXPECT_EQ(planner->init_name_, "my_planner");
  EXPECT_EQ(planner->init_ctx_, server.costmapContext());
}

TEST(CostmapNavigationServerTest, InitializePlannerPluginRejectsNonCostmapPlanner) {
  auto dir = make_config_dir("create_test_costmap_planner_plugin");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto plain = std::make_shared<PlainAbstractPlanner>();
  EXPECT_FALSE(server.initializePlannerPlugin("plain", plain));
}

TEST(CostmapNavigationServerTest, InitializePlannerPluginFailsWithoutConfiguredMaps) {
  CostmapNavigationServer server;  // 未 configureCoreMaps (Task 18 minor 1)
  auto planner = std::make_shared<TestCostmapPlanner>();
  EXPECT_FALSE(server.initializePlannerPlugin("p", planner));
  EXPECT_FALSE(server.initializeSmootherPlugin("s", nullptr));
}

// ---------------- loadPlugins 端到端 ----------------

TEST(CostmapNavigationServerTest, LoadPluginsLoadsCostmapPluginsAndCreatesActions) {
  auto dir = make_config_dir("create_test_costmap_planner_plugin");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  server.loadPlugins(dir.string());

  ASSERT_NE(server.plannerAction(), nullptr);
  ASSERT_NE(server.smootherAction(), nullptr);
  ASSERT_NE(server.controllerAction(), nullptr);
  ASSERT_NE(server.recoveryAction("test_costmap_recovery"), nullptr);
  ASSERT_EQ(server.recoveryAction("missing"), nullptr);
}

TEST(CostmapNavigationServerTest, LoadPluginsWithNonCostmapPlannerFails) {
  auto dir = make_config_dir("create_plain_abstract_planner_plugin");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  server.loadPlugins(dir.string());

  // planner dynamic_pointer_cast 失败 → 不入表 → plannerAction 未创建
  EXPECT_EQ(server.plannerAction(), nullptr);
  // 其余族正常装载
  ASSERT_NE(server.controllerAction(), nullptr);
  ASSERT_NE(server.smootherAction(), nullptr);
}

// ---------------- new*Execution 工厂 ----------------

TEST(CostmapNavigationServerTest, NewPlannerExecutionReturnsCostmapExecution) {
  auto dir = make_config_dir("create_test_costmap_planner_plugin");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto planner = std::make_shared<TestCostmapPlanner>();
  auto exec = server.newPlannerExecution("test_planner", planner);
  auto* cast =
      dynamic_cast<ayanami::costmap::CostmapPlannerExecution*>(exec.get());
  ASSERT_NE(cast, nullptr);
  EXPECT_EQ(cast, exec.get());
}

TEST(CostmapNavigationServerTest, NewControllerExecutionReturnsCostmapExecution) {
  auto dir = make_config_dir("create_test_costmap_planner_plugin");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto exec = server.newControllerExecution("test_controller", nullptr);
  auto* cast =
      dynamic_cast<ayanami::costmap::CostmapControllerExecution*>(exec.get());
  ASSERT_NE(cast, nullptr);
  EXPECT_EQ(cast, exec.get());
}

TEST(CostmapNavigationServerTest, NewRecoveryExecutionReturnsCostmapExecution) {
  auto dir = make_config_dir("create_test_costmap_planner_plugin");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());

  auto exec = server.newRecoveryExecution("test_recovery", nullptr);
  auto* cast =
      dynamic_cast<ayanami::costmap::CostmapRecoveryExecution*>(exec.get());
  ASSERT_NE(cast, nullptr);
  EXPECT_EQ(cast, exec.get());
}

// ---------------- handleObstaclePointCloud (含 Ruling #7) ----------------

TEST(CostmapNavigationServerTest, HandleObstaclePointCloudMarksAndInflates) {
  auto dir = make_config_dir("create_test_costmap_planner_plugin");
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

  // Ruling #7: 障碍移动 (lidar(2,0) → odom(2.1,0) → cell(14,10)) 后旧膨胀带消失
  server.handleObstaclePointCloud(makePointCloud(2.0f, 0.0f, 0.0f));

  // 旧障碍 cell + 旧膨胀带 cell 恢复 FREE (距新源 > 3 cells)
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::FREE_SPACE);
  EXPECT_EQ(master->getCost(10, 13), costmap_2d::FREE_SPACE);
  // 新位置障碍 + 新膨胀带
  EXPECT_EQ(master->getCost(14, 10), costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(14, 11), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

TEST(CostmapNavigationServerTest, HandleObstaclePointCloudIgnoredWithoutMap) {
  CostmapNavigationServer server;  // 未 configureCoreMaps
  setupTransforms();
  EXPECT_NO_THROW(server.handleObstaclePointCloud(makePointCloud(0, 0, 0)));
}

TEST(CostmapNavigationServerTest, ObstacleCallbacksForwardToObstacleLayer) {
  auto dir = make_config_dir("create_test_costmap_planner_plugin");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();

  auto* obstacle_layer = findObstacleLayer(server.localCostmap());
  ASSERT_NE(obstacle_layer, nullptr);

  ayanami::interfaces::NavigationCallbacks::on_obstacle_pointcloud(
      makePointCloud(0.0f, 0.0f, 0.0f));

  auto* master = server.localCostmap()->get().getCostmap();
  EXPECT_EQ(master->getCost(10, 10), costmap_2d::LETHAL_OBSTACLE);
}

TEST(CostmapNavigationServerTest, ConcurrentObstacleHandlersDoNotCrash) {
  auto dir = make_config_dir("create_test_costmap_planner_plugin");
  CostmapNavigationServer server;
  server.configureCoreMaps(dir.string());
  setupTransforms();

  // 两线程并发 hammer 同一障碍位置: 共享 prev_obstacles_local_ 使 clear 窗口
  // 必覆盖本帧点 (obs 帧语义 = "当前观测覆盖旧观测"), 故用同一 cell 保证
  // 任何交错下的最后写入都是 LETHAL → 断言确定。锁的作用是串行化 marks_/
  // clear_regions_/prev_obstacles_local_/grid 的并发改写 (无锁则 UB 可能崩)。
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

TEST(CostmapNavigationServerTest, DestructorStopsActionsAndDoesNotCrash) {
  auto dir = make_config_dir("create_test_costmap_planner_plugin");
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
  // 作用域结束 → 析构: remove obstacle callbacks + 基类 stop/reset/clearPlugins,
  // 不挂 (Ruling #3)。
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_plugins_so = argv[1];
  }
  return RUN_ALL_TESTS();
}
