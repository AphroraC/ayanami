# ayanami / ayanami_ros M2 架构设计 (ROS1 边界接入)

- 状态：草案 v1（已与用户逐节确认）
- 日期：2026-08-14
- 适用范围：`src/ayanami`（M1 已就绪，本 spec 仅修订 carry 项）+ `src/ayanami_ros`（重新铺 ROS1 边界层）
- 性质：**延续重写**——在 M1 redesign 分支之上接续；3 BLOCK carry 优先修；之后 ayanami_ros 重新铺
- 参考项目：`test/mapping/glim` + `test/mapping/glim_ros`、`test/move_base_flex`、`test/navigation2`、M1 spec
- 端到端联调：`roslaunch nav_demo ayanami.launch`（Gazebo + amcl + map_server + ayanami_ros + RViz）

本工作空间的对话与文档一律使用中文。

---

## 0. 总纲

M1 spec §0 立项的"纯 C++ 核心 + ROS 薄壳"在 Plan 1 已完成核心库 (33 ctest pass, push origin/redesign head `9cc56af`)。本 M2 spec 接续完成：

| 维度 | 本 round 工作 |
|---|---|
| M1 carry BLOCK | 3 项必修 (controller 整循环锁死锁 / abstract base 虚析构 UB / recovery 锁不全) |
| M1 carry minor | 5 项一起修 (robot pose wiring / robot_frame 单一来源 / find_dependency / 其他) |
| ayanami_ros 重新铺 | ROS1 catkin 边界层 (AyanamiROS + TF2Listener + ros_msg_converter + ExtensionModuleROS + rviz_viewer .so) |
| 仿真端到端 | Gazebo nav_demo 验证 |

**不做** (延 Plan 3)：
- 全套 gridmap_navigation (GridmapContext/Gridmap*Exec/Action/GridmapNavigation)。
- 振荡两级检测 (spec §10 延后)。

> **重要约定**：`src/_legacy_ayanami_ros/` 是 ROS 边界参考（提取思路不 copy）；`src/_legacy_ayanami/` 仍作 algorithm 参考。`src/ayanami/thirdparty/` 与 `docs/superpowers/` 由 M1 验证保留。

---

## 1. 顶层架构 (本 round 修订点)

### 1.1 依赖方向（与 M1 spec §1.1 一致）

```
ayanami_ros (catkin, ROS1)            ← 重新铺
    |  only depends on libayanami (catkin 导出, find_package(ayanami CONFIG))
    |  + 依赖 ROS1 (roscpp/tf2/tf2_ros/nav_msgs/sensor_msgs/geometry_msgs/pcl_conversions)
    v
libayanami (M1 已就绪, 仅修 3 BLOCK + minor carry) ← 修订
    |
    +---> modules/*.so  (M1 已就绪, M2 不动)
    +---> packages/*    (M1 已就绪)
    +---> thirdparty/*  (vendored, M1 已 commit)
```

`libayanami` 至 M2 仍零 ROS include（只修订 internal 实现，接口不破坏）。`ayanami_ros` 是 ROS1 唯一接触面。

### 1.2 本 round 目录修订

```
src/ayanami/   # 修订
  include/ayanami/
    core/abstract_controller.hpp   ← +virtual ~ =default (BLOCK #2)
    core/abstract_smoother.hpp      ← +virtual ~ =default
    core/abstract_recovery.hpp      ← +virtual ~ =default
    components/controller_execution.hpp        ← fallback robot pose wiring (minor)
    costmap/components/costmap_controller_execution.hpp  ← per-cycle short lock (BLOCK #1)
    costmap/components/costmap_recovery_execution.hpp    ← lock global+local (BLOCK #3)
    modules/costmap_smac_planner/costmap_smac_planner.hpp  ← start 留空时 getRobotPose fallback (minor)
  config/costmap_navigation.json    ← robot_frame 单一来源 + 仿真对齐 (minor)
  cmake/ayanami-config.cmake.in      ← +find_dependency(Eigen3/spdlog) (minor)

src/ayanami_ros/   # 全新铺设
  include/ayanami_ros/
    ayanami_ros.hpp
    ros_msg_converter.hpp
    tf2_listener.hpp
    extension_module_ros.hpp
  src/ayanami_ros/
    ayanami_ros.cpp
    tf2_listener.cpp
    extensions/
      rviz_viewer.cpp            ← ROS extension 实现 (导出 create_extension_module)
  apps/ayanami_rosnode.cpp
  launch/
    navigation.launch
    config.rviz
  config/
    config_ros.json    ← 不动 src/ayanami/config/ 下的全局 master config; 本文件仅 ROS 边界侧
  CMakeLists.txt
  package.xml
```

> **注**：`config_ros.json` 位置沿用 M1 spec §9.2 约定 —— 由 `GlobalConfig::get_config_path("config_ros")` 解析。考虑到 ROS 边界侧配置不属于核心库资产，但 GlobalConfig 实例化需在核心层初始化前完成，沿用 `src/ayanami/config/config_ros.json` 更合理（M1 已按此规划）。如发现 catkin install 后路径失稳，M2 实施时再回中心化或 ayanami_ros 子包单独配置。

---

## 2. M1 carry BLOCK 修法（第一批必做）

### 2.1 BLOCK #1: controller 整循环持锁 → per-cycle short lock

**问题**：`costmap_controller_execution.hpp::run()` 整 control loop 持 `local_costmap_->mutex()`（recursive_mutex）。M2 接 ROS 障碍回调 (走同一 local 锁 via `handleObstaclePointCloud`) + `publishMapsForVisualization` (via spinOnce 50Hz) 会饿死 controller 线程，进而令 AyanamiROS loop thread 停滞 (TF sync + goal 停响应)。

**方案**：拆为每 cycle 短锁。
- `controller_frequency` cycle (~50ms @20Hz) 内：
  1. 不持锁取机器人 pose (用 TransformStore，已原子)
  2. `std::lock_guard<std::recursive_mutex>(local_costmap_->mutex())` 短锁：copy 相关 costmap slice 不必要，直接调 `AbstractController::computeVelocityCommands(...)`（controller 内部读 costmap cell 时与 handleObstacle 互斥）
  3. 释锁后发 cmd_vel via 旁路 `NavigationCallbacks::on_velocity_command`
  4. `std::this_thread::sleep_for(1/freq)` 不持锁
- `handleObstaclePointCloud/LaserScan` 内已持 local recursive_mutex (T20 R1 fix)；与 controller short lock 互斥正确。
- `publishMapsForVisualization` (T25) 已持 wrapper mutex (per-wrapper lock_guard)，与 controller short lock 互斥正确。

**ctest**：`ConcurrentControllerAndObstacleFeed` — 两个线程：一持续 controller 跑 50 cycle，另一每 5ms 投一次点云；不死锁；总时间 < 200 cycle × 1/freq × 2 倍预算（防饿死）。

### 2.2 BLOCK #2: abstract base 3 个虚析构 → 修 Ruling #3 UB

**问题**：M1 Ruling #3 决议 "派生类衍生 `virtual ~Derived() override = default` 修 UB" 是无效的——基类指针 `shared_ptr<AbstractRecovery/Controller/Smoother>` delete 时只调基类析构 (`AbstractRecovery` 无 `virtual ~`) → 派生析构不被调 → 派生非 POD 成员 leak。

**方案**：在 abstract 层补虚析构（spec §5.1 planner/navigation 已是 virtual ~，本 BLOCK 补 3 个）。
```cpp
// core/abstract_controller.hpp
class AbstractController {
public:
    using Ptr = std::shared_ptr<AbstractController>;
    virtual ~AbstractController() = default;     // ★ 加
    // ... 原接口 ...
};
// 同理 abstract_smoother.hpp / abstract_recovery.hpp
```
派生类已经写 `~Derived() override = default`，加 abstract 虚构后 `override` 真生效。

**ctest**：`PluginTeardownNoUB` —— 一个 FakeRecovery 派生 (持非 POD std::string 成员)；`PluginManager<AbstractRecovery>::loadPlugin` 后 `clearPlugins`；全局 `static int dtor_called = 0;` 在派生析构里 ++ 验证 `dtor_called == 1`。

### 2.3 BLOCK #3: recovery 锁不全 → 同时锁 global+local

**问题**：`costmap_recovery_execution.hpp::run()` 只锁 global wrapper，但 `CostmapClearCostmapRecovery::runRecovery` 写 local master grid (clear_area)，与 `handleObstaclePointCloud` 并发竞态。

**方案**：run() 内 `std::scoped_lock(global_mutex, local_mutex)` 或两个顺序 `lock_guard`（顺序固定 global→local 防死锁）。

**ctest**：`RecoveryVsObstacleFeedSameMap` —— 实例化 recovery + runRecovery 跑 + 同时投点云到 local map；verify 不崩 (cell 写冲突无 UB)。

---

## 3. M1 carry minor 项（Task 3 合并）

| 项 | 修法 |
|---|---|
| 4. robot pose fallback | `AbstractPlanner::makePlan(start=空时)` 由 `smac_planner` 调 `CostmapContext::getRobotPose()` fallback；`ControllerExecution` inject真实 RobotStateProvider (来自 on_robot_state_updated); 不再硬编码 identity |
| 5. robot_frame 单一来源 | `CostmapNavigationServer::configureCoreMaps` 读 `costmap_navigation.json::robot_frame`（不再 `costmap.json::robot_frame`），写 RobotStateProvider + CostmapContext; 启动期一致校验 与 `config_ros.json::robot_frame`一致 |
| 6. `cmake/ayanami-config.cmake.in` | 加 `include(CMakeFindDependencyMacro)` + `find_dependency(Eigen3)` + `find_dependency(spdlog)` + `find_dependency(Threads)`；下游 `find_package(ayanami CONFIG)` 自动解析 |
| 7. `pid_controller.plan_signature` bit-exact | 改用 goal pos (x,y) + path.size 粗签名（half byte tolerance），re-smooth 后同 goal 不重置 PID |
| 8. `configureCoreMaps` 二次调用 | 加 `AssertionError` warn + spdlog error return（不替换 context_），防已 initialize 插件悬空 |

---

## 4. ayanami_ros 边界层

### 4.1 AyanamiROS 类

```cpp
namespace ayanami_ros {
class AyanamiROS {
  ros::NodeHandle nh_;
  std::string config_path_;
  ayanami::abstract::AbstractNavigation::Ptr navigation_plugin_;
  std::vector<ExtensionModuleROS::Ptr> extension_modules_;
  std::vector<ros::Subscriber> extension_subs_;
  std::vector<ExtensionModuleROS::GenericTopicSubscription> extension_topic_subs_;
  TF2Listener tf_listener_;
  ros::Subscriber goal_sub_, pointcloud_sub_, odom_sub_, laserscan_sub_;
  sensor_msgs::PointCloud2 last_cloud_;   // PCL 转
  nav_msgs::Odometry last_odom_;
  std::atomic<bool> kill_switch_{false};
  std::thread loop_thread_;
  std::mutex goal_mutex_;
  double tf_sync_rate_ = 50.0;
public:
  explicit AyanamiROS(ros::NodeHandle& nh);
  ~AyanamiROS();
  void start();
  void stop();
private:
  void loadExtensionModules(const ayanami::Config& config_ros);
  void loadNavigationPlugin(const ayanami::Config& config_ros);
  void subscribeTopics();
  void goalCallback(const geometry_msgs::PoseStampedConstPtr&);
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr&);
  void laserscanCallback(const sensor_msgs::LaserScanConstPtr&);
  void odomCallback(const nav_msgs::OdometryConstPtr&);
  void syncTransformsToCore();
  void loop();   // tf_sync_rate Hz: syncTransformsToCore + navigation_plugin->spinOnce + ros::spinOnce
};
}
```

### 4.2 初始化顺序（M1 spec §7.2 关键）

```
1. nh_ 私有句柄
2. nh_.param<std::string>("config_path", config_path_, ros::package::getPath("ayanami") + "/config")
3. ayanami::GlobalConfig::instance(config_path_, true)
4. ayanami::Config config_ros(ayanami::GlobalConfig::instance()->get_config_path("config_ros"))
5. tf_listener_ 构造 (tf2_ros::Buffer + TransformListener)
6. loadExtensionModules(config_ros)        ← 先 (rviz_viewer 注册 NavigationCallbacks 订阅)
7. loadNavigationPlugin(config_ros)       ← dlopen libayanami_costmap_navigation.so
                                              create_navigation_module → initialize(config_path_) → start()
8. subscribeTopics()
9. loop_thread_ 启
```

**关键**：rviz_viewer 等 ROS 扩展必须**先于** navigation_plugin->start() 注册其 NavigationCallbacks 订阅，否则启动期第一波可视化 emit 会被错过。

### 4.3 ROS → 核心

| ROS 消息 | 转换 | 核心 |
|---|---|---|
| `geometry_msgs::PoseStamped` (`/move_base_simple/goal`) | ros_msg_converter_pose | `navigation_plugin_->setGoal(Eigen PoseStamped)` |
| `sensor_msgs::PointCloud2` | ros_msg_converter_pointcloud2 (PCL) | `NavigationCallbacks::on_obstacle_pointcloud(ayanami::sensor_msgs::PointCloud2)` |
| `sensor_msgs::LaserScan` | ros_msg_converter_laserscan | `NavigationCallbacks::on_obstacle_laserscan(ayanami::sensor_msgs::LaserScan)` |
| `nav_msgs::Odometry` | 取 pose + twist 经 syncTransformsToCore 合成 Pose+Twist | `NavigationCallbacks::on_robot_state_updated(PoseStamped, TwistStamped)` |

### 4.4 TF -> 核心

`syncTransformsToCore`（tf_sync_rate 50Hz）：
- 对三条边 `map→odom`、`odom→base_link`、`map→base_link` 调 `tf2_listener_.lookup_transform(target, source, ros::Time(0))`
- 转 `Eigen::Isometry3d` 后 `TransformStore::instance().setTransform`（stamp 用消息自带时间戳替代 M1 默认 0s，支持精确时间查询）
- 合成 `PoseStamped{map, robot pose}` + `TwistStamped{odom, robot vel}` → emit `on_robot_state_updated`

### 4.5 核心 -> ROS（由扩展模块订阅 NavigationCallbacks）

`rviz_viewer` ROS extension (`src/ayanami_ros/extensions/rviz_viewer.cpp`) 构造期：
```cpp
NavigationCallbacks::on_get_path_result.add(...bind(publishPlannerPath...));
NavigationCallbacks::on_smooth_path_result.add(...bind(publishSmootherPath...));
// on_compute_velocity_command  (Ruling: M2 加 fix-after-T27 — 具体 callback 名映射 M1 实际 on_velocity_command)
NavigationCallbacks::on_velocity_command.add(...bind(publishCmdVel...));
NavigationCallbacks::on_costmap_visualization.add(...bind(publishCostmap...));
```
静态地图初始 publish 一次（StaticLayer 加载后 → publishMapsForVisualization 经 emit）。

`create_subscriptions(nh)` 可注册额外 ROS sub（rviz interactive marker / manual goal 等），本轮可为空。

### 4.6 职责红线

- `ayanami_ros` **不 include `costmap_2d/*`**。
- `ayanami_ros` **不持有 BT**（已由 AbstractNavigation 宿主封装；ROS 层只 dlopen + initialize + start）。
- `ayanami_ros` 不知 planner_id / costmap 尺寸 / BT xml 等核心参数。
- `rviz_viewer` 取核心 `OccupancyGrid` 转发为 ROS `nav_msgs::OccupancyGrid`，不暴露 costmap_2d 类型。

---

## 5. ExtensionModuleROS 基类

```cpp
// include/ayanami_ros/extension_module_ros.hpp
#include <ros/ros.h>
#include <ayanami/utility/extension_module.hpp>

namespace ayanami_ros {
class ExtensionModuleROS : public ayanami::ExtensionModule {
public:
  using Ptr = std::shared_ptr<ExtensionModuleROS>;
  struct GenericTopicSubscription {
    ros::Subscriber sub;   // 句柄保存
    std::string topic;
  };
  virtual std::vector<GenericTopicSubscription> create_subscriptions(ros::NodeHandle& nh) { return {}; }
  virtual bool needs_wait() const { return false; }
  virtual bool ok() const { return true; }
  virtual void at_exit(const std::string& dump_path) { (void)dump_path; }
};
}
```

`AyanamiROS::loadExtensionModules(config_ros)` 流程：
1. 读 `config_ros.json::extension_modules` 列表（如 `["librviz_viewer.so"]`）
2. 逐个 `ExtensionModuleROS::load_module` (dlopen `create_extension_module` 工厂，docman Ruling — spec §4.5 不 dlclose)
3. `dynamic_pointer_cast<ExtensionModuleROS>` 成功则保存到 `extension_modules_`，并调 `create_subscriptions(nh)` 保存话题句柄

---

## 6. ros_msg_converter.hpp

Eigen ↔ ROS：
- `fromRosPoseStamped(const geometry_msgs::PoseStamped&, ayanami::geometry_msgs::PoseStamped&)`
- `toRosPath(const ayanami::nav_msgs::Path&, nav_msgs::Path&)`
- `fromRosTwist(const geometry_msgs::TwistStamped&, ayanami::geometry_msgs::TwistStamped&)`
- `fromRosPointCloud2(const sensor_msgs::PointCloud2&, ayanami::sensor_msgs::PointCloud2&)`（PCL include 用于字段解析，但 ayanami_ros catkin 已依赖 pcl_conversions）
- `toRosOccupancyGrid(const ayanami::nav_msgs::OccupancyGrid&, nav_msgs::OccupancyGrid&)`
- `fromRosLaserScan(const sensor_msgs::LaserScan&, ayanami::sensor_msgs::LaserScan&)`

---

## 7. TF2Listener

```cpp
class TF2Listener {
  tf2_ros::Buffer buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::vector<std::pair<std::string, std::string>> tf_sync_edges_;
public:
  TF2Listener(ros::NodeHandle& nh, const std::vector<std::pair<std::string, std::string>>& edges);
  bool lookup_transform(const std::string& target, const std::string& source, const ros::Time& stamp, Eigen::Isometry3d& out, double timeout = 0.0);
};
```

构造时 `buffer_` + `TransformListener(&buffer_)`，list edge 由 `config_ros.json` 控 (默认 [map→odom, odom→base_link, map→base_link])。

---

## 8. 配置文件

### 8.1 `src/ayanami/config/config_ros.json`（位置沿用 M1 spec §9.2）

```jsonc
{
  "ayanami_ros": {
    "pointcloud_topic": "/velodyne_points",   // nav_demo Gazebo 用 laser; 改 "/scan" 或留 las scan 单独处理
    "laserscan_topic": "/scan",                // Gazebo laser
    "odom_topic": "/odom",
    "goal_topic": "/move_base_simple/goal",
    "cmd_vel_topic": "/cmd_vel",
    "planner_path_topic": "/ayanami/planner/path",
    "smoother_path_topic": "/ayanami/smoother/path",
    "global_costmap_topic": "/ayanami/global_costmap",
    "local_costmap_topic": "/ayanami/local_costmap",
    "static_map_topic": "/ayanami/map",
    "map_frame": "map",
    "odom_frame": "odom",
    "robot_frame": "base_footprint",
    "tf_sync_rate": 50.0,
    "navigation_so_name": "libayanami_costmap_navigation.so",
    "navigation_create_symbol": "create_navigation_module",
    "extension_modules": ["librviz_viewer.so"]
  }
}
```

### 8.2 `src/ayanami/config/costmap_navigation.json` 修订

- `robot_frame`: `"base_footprint"` （与 nav_demo amcl 一致）
- `static_map_path`: 改为指向 `nav_demo/map/nav.yaml`（或 catkin install 后 `$(find nav_demo)/map/nav.yaml` 形式，M2 实施时加 launch arg 或 keep core config 传 Gazebo 仿真静态地图）

### 8.3 `src/ayanami/config/costmap.json` 修订

- global costmap 尺寸对齐静态地图（box_house world）
- local rolling 5m×5m, 0.05 分辨率
- obstacle_layer 用 LaserScan observation_source (Gazebo 提供的是 /scan)

### 8.4 launch 配置

- `src/ayanami_ros/launch/navigation.launch`:
  ```xml
  <launch>
    <arg name="rviz" default="true"/>
    <node name="ayanami_ros" pkg="ayanami_ros" type="ayanami_rosnode" output="screen"/>
    <group if="$(arg rviz)">
      <node pkg="rviz" type="rviz" name="rviz" args="-d $(find ayanami_ros)/launch/config.rviz"/>
    </group>
  </launch>
  ```
- `src/ayanami_ros/launch/config.rviz`: 显示 global_costmap/local_costmap/static_map/planner_path/smoother_path/cmd_vel/goal

**nav_demo/ayanami.launch 不动**（M1 兼容）。

---

## 9. CMakeLists.txt (src/ayanami_ros)

```cmake
cmake_minimum_required(VERSION 3.16)
project(ayanami_ros VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib tf2 tf2_eigen tf2_geometry_msgs tf2_ros
  nav_msgs sensor_msgs geometry_msgs pcl_conversions
  ayanami
)

find_package(ayanami CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(spdlog REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ayanami_ros rviz_viewer
  CATKIN_DEPENDS roscpp roslib tf2 tf2_eigen tf2_geometry_msgs tf2_ros nav_msgs sensor_msgs geometry_msgs pcl_conversions ayanami
)

add_library(ayanami_ros
  src/ayanami_ros/ayanami_ros.cpp
  src/ayanami_ros/tf2_listener.cpp
)
target_link_libraries(ayanami_ros PUBLIC
  ayanami::ayanami
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  PCL::common PCL::io
  spdlog::spdlog
  tf2_eigen
)

# rviz_viewer extension .so (导出 create_extension_module)
add_library(rviz_viewer SHARED
  src/ayanami_ros/extensions/rviz_viewer.cpp
)
target_link_libraries(rviz_viewer PUBLIC
  ayanami_ros
  ${catkin_LIBRARIES}
)
set_target_properties(rviz_viewer PROPERTIES PREFIX "lib")

add_executable(ayanami_rosnode apps/ayanami_rosnode.cpp)
target_link_libraries(ayanami_rosnode ayanami_ros)

install(TARGETS ayanami_ros rviz_viewer ayanami_rosnode
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include launch config DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
```

---

## 10. package.xml (src/ayanami_ros)

```xml
<package format="2">
  <name>ayanami_ros</name>
  <version>1.0.0</version>
  <description>ayanami ROS1 边界适配</description>
  <maintainer email="dev@example.com">ayanami</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>roslib</depend>
  <depend>tf2</depend>
  <depend>tf2_eigen</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>pcl_conversions</depend>
  <depend>ayanami</depend>
</package>
```

---

## 11. 端到端联调（Plan 2 末里程碑）

```bash
# 1. 构建
cd /share/nav_ws
catkin build ayanami ayanami_ros

# 2. 启动仿真
roslaunch nav_demo ayanami.launch

# 3. RViz 点 goal
# 观察验证:
# - `rostopic hz /cmd_vel > 10Hz`
# - `rostopic echo /ayanami/planner/path` 有非空 path
# - Gazebo 中 robot 按 path 移动直到 goal
# - 中途改 goal 不死锁 (验证 session_id 抢占 + 零速度反发)
# - `Ctrl+C` 关闭不 segfault (析构链通过 Ruling #3 修后干净)
```

**关键**：使用仿真世界的真实 odom + scan + amcl map→odom TF，验证 ayanami 核心与 ROS 真实时序/约束兼容。

---

## 12. 已知坑的规避汇总 (carry from M1 + M2 新增)

| 历史/现状坑 | 根因 | 本 round 规避 |
|---|---|---|
| 跨 .so 静态 slot 失效 | 槽 inline static | M1 已修 (lib .cpp 内一份) |
| BT.CPP vendored tinyxml2 符号冲突 | 系统 libtinyxml2 重名 | M1 已加 `-Wl,-Bsymbolic` |
| 重新创建树 | 每 goal 整树重建 | M1 已修 (树常驻 pending_goal 仅重写黑板) |
| BT action 结果竞态 | 回调里 goal_id 0 丢 | M1 已修 (session/version 校验) |
| CallbackSlot 无锁竞态 | glim 无锁 | M1 已修 (shared_mutex) |
| `config/` vs `configs/` 漂移 | CMake 不一致 | M1 已统一 |
| `logging_dir` 写死绝对路径 | 直接写死 | M1 已改相对 |
| 帧名 `base_footprint` / `base_link` 混用 | 不同子配置自声明 | M2 修 (单一来源 `costmap_navigation.json::robot_frame` + 一致校验) |
| `recovery_id` 与插件 `name` 不匹配 | unsuffixed id | M1 已修 (Ruling #8 full plugin name) |
| CMake 缺 `ayanami-config.cmake.in` find_dependency | 无外部依赖引导 | M2 补 (Task 3 minor carry) |
| TwistStamped 未初始化脏内存 | `{}` 缺省后未清零 | M1 已修 + M2 在 ros_msg_converter 里同样守纪律 |
| RateController 语义混乱 | hz<=0 当每 tick 运行 | M1 已对齐 nav2 |
| dlopen 句柄管理 | 早期无策略 | M1 已守 §4.5 顺序纪律 |
| 振荡检测只有一级 | 现状未做 | 延 Plan 3 |
| **BLOCK #1** controller 整循环持锁 | `run()` 整循环持 `recursive_mutex` | **M2 Task 1 修 per-cycle short lock** |
| **BLOCK #2** abstract base 非虚析构 | AbstractRecovery/Controller/Smoother 无 virtual ~ | **M2 Task 0 补 virtual ~ =default** |
| **BLOCK #3** recovery 锁不全 | recovery run 只锁 global | **M2 Task 2 加 scoped_lock(global, local)** |
| robot pose fallback 未实 | smac planner 用 start.pose 直 | **M2 Task 3 minor** |
| `plan_signature` bit-exact | 精度过高同样的微分导致重置 PID | **M2 Task 3 minor** |
| 二次 `configureCoreMaps` 非幂等 | 替换 `costmap_context_` 已 initialize 插件悬空 | **M2 Task 3 minor** |

---

## 13. 实施步骤概要

退出 plan mode 后调用 `writing-plans` skill 出可执行实施计划。

**阶段 A：BLOCK 修 + Wiring + minor carry (M1 修订)**
1. Task 0: BLOCK #2 — abstract base 3 个补 `virtual ~ = default` + `PluginTeardownNoUB` ctest
2. Task 1: BLOCK #1 — controller per-cycle short lock + `ConcurrentControllerAndObstacleFeed` ctest
3. Task 2: BLOCK #3 — recovery 锁 global+local (scoped_lock) + `RecoveryVsObstacleFeedSameMap` ctest
4. Task 3: minor carry — robot pose fallback / robot_frame 单一来源 / find_dependency / plan_signature 粗签名 / 二次 configureCoreMaps 错误

**阶段 B：ayanami_ros 边界铺设**
5. Task 4: ayanami_ros 骨架 (CMakeLists + package.xml + 空类)
6. Task 5: `ExtensionModuleROS` 基类 + `.so` 模板
7. Task 6: `AyanamiROS` 类构造 + 加载链 + 失败 fallback
8. Task 7: `TF2Listener` 实现
9. Task 8: `ros_msg_converter.hpp` 实现
10. Task 9: `rviz_viewer` ROS extension `.so` (导出 create_extension_module) + dlopen 实调测试
11. Task 10: config_ros.json + costmap.json 修订 + nav_demo 适配
12. Task 11: launch/navigation.launch + config.rviz

**阶段 C：端到端联调 + push**
13. Task 12: `catkin build ayanami ayanami_ros` + `roslaunch nav_demo ayanami.launch` 端到端验证
14. Task 13: 全套回归 `ctest` + push origin/redesign

---

## 14. 出 spec 自审清单

- [x] 无 TBD/TODO 占位。
- [x] §2.1-§2.3 与 §1 一致。
- [x] §4.4 TF stamp 用消息自带替代 M1 默认 0s（M1 carry 已记录待改进）。
- [x] §6 ros_msg_converter 引 PCL 在 ayanami_ros 边界 (core 仍零 PCL/ROS)。
- [x] §11 端到端用 `roslaunch nav_demo ayanami.launch` (已确认仿真路径)。
- [x] §10 ayanami_config.cmake.in 加 find_dependency。
- [x] gridmap_navigation 延 Plan 3 明示。
- [x] 振荡两级检测延 Plan 3 明示。
- [x] carry list 3 BLOCK + 5 minor 逐项 §2 + §3 对应 Task 0/1/2/3。

---

## 附：参考项目对应表 (M2 特定)

| ayanami M2 抽象 | glim_ros 等价 | nav2 等价 | nav_demo 等价 |
|---|---|---|---|
| `AyanamiROS` | `GlimROS` | (rclcpp node) | (none) |
| `TF2Listener` | (无, glim 不连 ROS TF) | `tf2_ros::Buffer+Listener` | `amcl_diff.launch` 提供 TF |
| `ros_msg_converter` | `glim::ros_cloud_converter` | (直接 ROS msg) | (none) |
| `ExtensionModuleROS` | `glim::ExtensionModuleROS` | (`nav2_bt_navigator` pluginlib) | (none) |
| `rviz_viewer ext .so` | `glim::RvizViewer` (在 glim_ros) | (rviz display) | `config.rviz` 消费 |
| Gazebo | — | — | `urdf_gazebo_demo/env.launch` |
| AMCL → map→odom TF | — | — | `amcl_diff.launch` |
| map_server | — | — | `nav_demo/map_server.launch` |