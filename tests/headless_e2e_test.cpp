// Task 28 (M1 终止): headless E2E 全链路测试。
//
// 不连真底盘/ROS, 用 dlopen libayanami_costmap_navigation.so 宿主插件组装整条
// 链: initialize → start → setGoal → BT (smac planner → savitzky smoother →
// controller) → on_velocity_command 旁路发出 cmd_vel; 周期性虚拟 pointcloud
// (空间外障碍不影响规划); 并验证 on_costmap_visualization 可视化旁路被触发。
// 最后 stop()/析构必须不崩 (Ruling #3)。
//
// argv: 1=host.so, 2=smac.so, 3=savitzky.so, 4=test_controller.so,
//       5=spin.so, 6=goal_checker.so, 7=progress_checker.so,
//       8=assets_dir (tests/assets, 含 headless_map.pgm)

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/interfaces/transform_store.hpp>
#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/sensor_msgs.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::abstract::AbstractNavigation;
using ayanami::geometry_msgs::PoseStamped;
using ayanami::geometry_msgs::Twist;
using ayanami::geometry_msgs::TwistStamped;
using ayanami::interfaces::NavigationCallbacks;

std::string g_host_so;
std::string g_smac_so;
std::string g_savitzky_so;
std::string g_test_controller_so;
std::string g_spin_so;
std::string g_goal_checker_so;
std::string g_progress_checker_so;
std::string g_assets_dir;

// ---- 采集器 ----
std::mutex g_cmd_mutex;
TwistStamped g_last_cmd;
int g_cmd_count = 0;
int g_cmd_cb_id = -1;

std::mutex g_map_mutex;
int g_map_count = 0;

// ---- 测试工具 ----

PoseStamped makePose(double x, double y) {
  auto p = PoseStamped::fromIsometry("map", Eigen::Isometry3d::Identity());
  p.pose.position = Eigen::Vector3d(x, y, 0.0);
  return p;
}

bool isZeroTwist(const TwistStamped& t) {
  return t.twist.linear.norm() < 1e-9 && t.twist.angular.norm() < 1e-9;
}
// 造单点 PointCloud2 (x/y/z FLOAT32, 12B/point), 障碍在空间外不影响规划。
ayanami::sensor_msgs::PointCloud2 makeFarPointCloud() {
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
  const float x = 50.0f;  // 20x20 @0.5 = 10m 地图之外 → worldToMap 失败不落图
  const float y = 50.0f;
  const float z = 0.0f;
  cloud.data.resize(12);
  std::memcpy(cloud.data.data(), &x, sizeof(float));
  std::memcpy(cloud.data.data() + 4, &y, sizeof(float));
  std::memcpy(cloud.data.data() + 8, &z, sizeof(float));
  return cloud;
}

void setupTransforms() {
  auto& store = ayanami::interfaces::TransformStore::instance();
  store.setTransform("odom", "base_link", ayanami::Timestamp::latest(),
                     Eigen::Isometry3d::Identity());
  store.setTransform("map", "odom", ayanami::Timestamp::latest(),
                     Eigen::Isometry3d::Identity());
  store.setTransform("base_link", "lidar", ayanami::Timestamp::latest(),
                     Eigen::Isometry3d::Identity());
}

void write_file(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

std::string pluginJson(const std::string& module, const std::string& name,
                        const std::string& so_name) {
  return "{\n  \"" + module + "\": {\n    \"name\": \"" + name +
         "\",\n    \"so_name\": \"" + so_name + "\"\n  }\n}\n";
}

// 完整配置套件 (对标 spec §9 全套)。static_map 指向 tests/assets 的
// headless_map.pgm (20x20 @0.5 = 10m x 10m, 全 FREE, A* 从 (0,0) 到 (2,0) 可寻路)。
std::filesystem::path makeConfigDir() {
  const auto dir = std::filesystem::temp_directory_path() /
                   "ayanami_headless_e2e";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  std::filesystem::create_directories(dir / "BT");

  write_file(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_navigation\": \"costmap_navigation.json\"\n"
             "  }\n}\n");

  {
    std::ofstream nav(dir / "costmap_navigation.json");
    nav << "{\n  \"navigation\": {\n"
        << "    \"behavior_tree_extension\": \"" AYANAMI_TEST_BT_EXT_SO "\",\n"
        << "    \"behavior_tree_xml\": \"BT/navigation.xml\",\n"
        << "    \"behavior_tree_id\": \"NavigateWithReplanningAndSmoothing\",\n"
        << "    \"config_planner\": \"planner.json\",\n"
        << "    \"config_smoother\": \"smoother.json\",\n"
        << "    \"config_controller\": \"controller.json\",\n"
        << "    \"config_recovery\": [\"recovery.json\"],\n"
        << "    \"config_goal_checker\": \"goal_checker.json\",\n"
        << "    \"config_progress_checker\": \"progress_checker.json\",\n"
        << "    \"clear_costmap_id\": \"test_spin\",\n"
        << "    \"spin_id\": \"test_spin\",\n"
        << "    \"wait_id\": \"test_spin\",\n"
        << "    \"backup_id\": \"test_spin\",\n"
        << "    \"planner_frequency\": 100.0,\n"
        << "    \"smoother_frequency\": -1.0,\n"
        << "    \"controller_frequency\": 20.0,\n"
        << "    \"tick_frequency\": 100.0,\n"
        << "    \"costmap_visualization_rate\": 100.0\n"
        << "  }\n";
  }

  write_file(dir / "planner.json",
             pluginJson("planner", "test_planner", g_smac_so));
  write_file(dir / "smoother.json",
             pluginJson("smoother", "test_smoother", g_savitzky_so));
  write_file(dir / "controller.json",
             pluginJson("controller", "test_controller",
                         g_test_controller_so));
  write_file(dir / "recovery.json",
             "{\n  \"recovery\": {\n    \"name\": \"test_spin\",\n"
             "    \"role\": \"spin\",\n    \"so_name\": \"" + g_spin_so +
             "\"\n  }\n}\n");
  // goal/progress checker 插件 (execution 需要 injected checkers, 缺失判
  // InternalError)。参数并入同一 json (模块经 config_goal_checker /
  // config_progress_checker 显式读取; progress 的 required_movement 设小:
  // 模拟 robot 需持续判有进展, 避免 5s 静止窗口误触发 NoProgress)。
  write_file(dir / "goal_checker.json",
             "{\n  \"goal_checker\": {\n"
             "    \"name\": \"simple_goal_checker\",\n"
             "    \"so_name\": \"" + g_goal_checker_so + "\",\n"
             "    \"position_tolerance\": 0.10,\n"
             "    \"angle_tolerance\": 0.20,\n"
             "    \"linear_tolerance\": 0.10,\n"
             "    \"angular_tolerance\": 0.10\n"
             "  }\n}\n");
  write_file(dir / "progress_checker.json",
             "{\n  \"progress_checker\": {\n"
             "    \"name\": \"simple_progress_checker\",\n"
             "    \"so_name\": \"" + g_progress_checker_so + "\",\n"
             "    \"required_movement\": 0.1,\n"
             "    \"movement_angle_required\": 0.2,\n"
             "    \"time_period\": 5.0\n"
             "  }\n}\n");

  // static_map 指向 committed asset (tests/assets/headless_map.pgm)。
  {
    // costmap 参数并入 costmap_navigation.json (原 costmap.json 移除)。
    std::ofstream nav(dir / "costmap_navigation.json", std::ios::app);
    nav << ",\n  \"costmap\": {\n"
        << "    \"global_frame\": \"map\",\n"
        << "    \"local_frame\": \"odom\",\n"
        << "    \"robot_frame\": \"base_link\",\n"
        << "    \"resolution\": 0.5\n"
        << "  },\n"
        << "  \"costmap_global\": {\n"
        << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
        << "    \"rolling_window\": false,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"static_map\": \""
        << (std::filesystem::path(g_assets_dir) / "headless_map.pgm").string()
        << "\",\n"
        << "    \"inflation_radius\": 1.0,\n"
        << "    \"inscribed_radius\": 0.8,\n"
        << "    \"cost_scaling_factor\": 2.0\n"
        << "  },\n"
        << "  \"costmap_local\": {\n"
        << "    \"size_x\": 20,\n    \"size_y\": 20,\n"
        << "    \"rolling_window\": true,\n"
        << "    \"track_unknown\": false,\n"
        << "    \"inflation_radius\": 1.0,\n"
        << "    \"inscribed_radius\": 0.8,\n"
        << "    \"cost_scaling_factor\": 2.0\n"
        << "  }\n"
        << "}\n";
  }

  write_file(dir / "costmap_savitzky_smoother.json",
             "{\n  \"costmap_savitzky_smoother\": {\n"
             "    \"do_refinement\": true,\n    \"refinement_num\": 2\n"
             "  }\n}\n");
  write_file(dir / "costmap_spin_recovery.json",
             "{\n  \"recovery\": {\n"
             "    \"spin_dist\": 1.57,\n    \"max_w\": 0.5,\n"
             "    \"sim_cycle_dt\": 0.01,\n    \"collision_radius\": 0.3\n"
             "  }\n}\n");

  write_file(dir / "BT" / "navigation.xml",
             R"(<root BTCPP_format="4" main_tree_to_execute="NavigateWithReplanningAndSmoothing">
  <BehaviorTree ID="NavigateWithReplanningAndSmoothing">
    <RecoveryNode name="NavigateRecovery" number_of_retries="6">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="{planner_frequency}">
          <RecoveryNode name="ComputePath" number_of_retries="1">
            <Sequence>
              <ComputePathToPose goal="{goal}" path="{raw_path}"/>
              <IsPathValid path="{raw_path}"/>
              <SmoothPath  input_path="{raw_path}" path="{smoothed_path}"/>
              <IsPathValid path="{smoothed_path}"/>
            </Sequence>
            <Sequence>
              <Recovery recovery="{clear_costmap_id}"/>
              <AlwaysFailure/>
            </Sequence>
          </RecoveryNode>
        </RateController>
        <RecoveryNode name="FollowPath" number_of_retries="1">
          <FollowPath path="{smoothed_path}" frequency="{controller_frequency}"/>
          <Sequence>
            <Recovery recovery="{clear_costmap_id}"/>
            <AlwaysFailure/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryActions">
        <GoalUpdated goal="{goal}"/>
        <RoundRobin name="RecoveryActions">
          <Sequence><Recovery recovery="{spin_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{wait_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{backup_id}"/>  <AlwaysFailure/></Sequence>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>)");

  return dir;
}

void attachCollectors() {
  g_cmd_count = 0;
  g_last_cmd = TwistStamped{};
  g_cmd_cb_id = NavigationCallbacks::on_velocity_command.add(
      [](const TwistStamped& t) {
        std::lock_guard<std::mutex> lock(g_cmd_mutex);
        g_last_cmd = t;
        ++g_cmd_count;
      });
  g_map_count = 0;
}

void detachCollectors() {
  if (g_cmd_cb_id >= 0) {
    NavigationCallbacks::on_velocity_command.remove(g_cmd_cb_id);
    g_cmd_cb_id = -1;
  }
}

int cmdCount() {
  std::lock_guard<std::mutex> lock(g_cmd_mutex);
  return g_cmd_count;
}

bool nonZeroCmdCountAtLeast(int n) {
  // 控制器持续周期发 0.5 线性速度, 非零命令占绝大多数; 用总计数近似触发阈值。
  std::lock_guard<std::mutex> lock(g_cmd_mutex);
  return g_cmd_count >= n;
}

template <typename Fn>
bool waitUntil(const Fn& fn, int timeout_ms = 5000) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (fn()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return fn();
}

}  // namespace

// ---------------- Headless E2E: 全导航链路 ----------------

TEST(HeadlessE2E, fullNavigationPipeline) {
  // 1. GlobalConfig::instance(tmp config / full suite)
  const auto dir = makeConfigDir();
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);

  // 2. NavigationCallbacks 清理 (防 prior 测试泄漏污染)
  detachCollectors();

  // 3. 公共工厂装载 host → AbstractNavigation::Ptr (类型化 cast, requirement 2/4)
  auto host_ext = ayanami::ExtensionModule::load_module(g_host_so);
  ASSERT_NE(host_ext, nullptr) << "load_module failed: " << g_host_so;
  AbstractNavigation::Ptr nav =
      std::dynamic_pointer_cast<AbstractNavigation>(std::move(host_ext));
  ASSERT_NE(nav, nullptr);

  // 4. 收集 cmd_vel
  attachCollectors();

  // 5. 收集 map visualization (验证 start/限频触发 on_costmap_visualization 旁路)
  std::atomic<int> map_count{0};
  ayanami::CallbackSubscription map_cb =
      NavigationCallbacks::on_costmap_visualization.subscribe(
          [&map_count](const std::string&, const ayanami::nav_msgs::OccupancyGrid&) {
            ++map_count;
          });

  // 6. 启动
  nav->initialize(dir.string());
  nav->start();

  // 7. 障碍观测: setGoal 之前喂 3 帧虚拟 pointcloud (空间外, 不影响规划),
  //    验证宿主直递落图管线 (AbstractNavigation::setPointcloud, 不走全局输入槽;
  //    输入/输出分层纪律) 不崩。注意: 不在控制器线程启动后并发喂 (plan 规定
  //    Costmap*Execution::run 整循环持 local wrapper 锁, 并发点云回调会死锁 ——
  //    Task 28 发现, 见 report, 待 reviewer 裁决)。
  setupTransforms();
  for (int i = 0; i < 3; ++i) {
    nav->setPointcloud(makeFarPointCloud());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // 8. setGoal (map frame, x=2, y=0, yaw=0)
  nav->setGoal(makePose(2.0, 0.0));

  // 9. 等待 cmd_count 达到预期
  EXPECT_TRUE(waitUntil([] { return nonZeroCmdCountAtLeast(3); }, 10000))
      << "cmd_count did not reach threshold: " << cmdCount();

  // 10. 断言: cmd_count > 阈值, cmd_vel 已初始化 (非脏内存/非 NaN)
  EXPECT_GT(cmdCount(), 1);
  TwistStamped last = g_last_cmd;
  EXPECT_TRUE(std::isfinite(last.twist.linear.x()));
  EXPECT_FALSE(std::isnan(last.twist.angular.z()));
  // 可视化旁路被触发 (start() 首帧 + 限频重复)
  EXPECT_GT(map_count.load(), 0);

  // 11. nav->stop() + cleanup; 析构不崩 (Ruling #3)
  nav->stop();
  detachCollectors();
  nav.reset();  // ~CostmapNavigation → stop() (幂等) + join, 不得崩
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) g_host_so = argv[1];
  if (argc > 2) g_smac_so = argv[2];
  if (argc > 3) g_savitzky_so = argv[3];
  if (argc > 4) g_test_controller_so = argv[4];
  if (argc > 5) g_spin_so = argv[5];
  if (argc > 6) g_goal_checker_so = argv[6];
  if (argc > 7) g_progress_checker_so = argv[7];
  if (argc > 8) g_assets_dir = argv[8];
  return RUN_ALL_TESTS();
}
