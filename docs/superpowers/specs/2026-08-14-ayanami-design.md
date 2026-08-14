# ayanami / ayanami_ros 架构设计

- 状态：草案 v1（已与用户逐节确认）
- 日期：2026-08-14
- 适用范围：`src/ayanami`（纯 C++ 核心导航库）与 `src/ayanami_ros`（ROS1 边界层）
- 性质：**推翻重写**——保留 `thirdparty/` 与 `tests/` 接口契约，其余按本 spec 重新铺
- 参考项目：`test/mapping/glim`、`test/mapping/glim_ros`、`test/move_base_flex`、`test/navigation2`

本工作空间的对话与文档一律使用中文。

---

## 0. 总纲

ayanami 是一个**纯 C++、不依赖 ROS** 的导航框架；`ayanami_ros` 是只做 ROS1 边界适配的薄壳。设计同时吸纳四个参考项目的关键机制：

| 维度 | 模仿对象 | 关键机制 |
|---|---|---|
| 参数加载 | glim `GlobalConfig` | `config.json` 逻辑名→文件名映射 + `get_config_path(name)` 取子配置 |
| 全局回调插槽 | glim `CallbackSlot` | 多播 vector + shared_mutex；静态对象定义在 lib .cpp（不 inline） |
| extension 插件 | glim `ExtensionModule` | `dlopen` + `extern "C" create_extension_module`；能 add 回调槽 |
| 算法核心与 ROS 分离 | glim 核心零 ROS include | 核心 ROS-free；ROS 侧只 dlopen AbstractNavigation 插件并转发输入 |
| 抽象地图 | move_base_flex 分层 | abstract 层零地图 include；地图对象由具体 NavigationServer 通过 `virtual` 钩子注入 |
| 地图形式无关 | MBF `AbstractPluginManager` + 虚钩子 | `initialize*Plugin` 改 virtual，注入 `MapContext*` |
| 四组件 | nav2 planner+smoother+controller+behavior | "behavior" 插件在 ayanami 更名为 **recovery** |
| 行为树编排 | nav2 bt_navigator | `BehaviorTreePilot` + nav2 风格精简 BT XML；BT 节点直接调 NavigationServer，不走 ROS action |

> **重要约定**：把 nav2 命名的 `behavior` 插件在 ayanami 中更名为 **recovery**。所有核心抽象类、action、execution、模块目录、配置键、BT 节点名一律用 `recovery`。仅保留 BT 语境用词（`BehaviorTree.CPP`、`<BehaviorTree ID=...>` XML 根标签、`BehaviorTreePilot`）。

---

## 1. 顶层架构

### 1.1 依赖方向

```
ayanami_ros (catkin, ROS1)
    |  only depends on libayanami (catkin 导出)
    v
libayanami  (纯 C++ shared 库, 零 ROS include)
    |
    +---> modules/*.so  (planner/smoother/controller/recovery/costmap_navigation 插件,
    |       dlopen 加载, 依赖 libayanami + packages)
    +---> packages/*    (costmap_2d / costmap_layers / costmap_server, 纯 C++ 子库)
    +---> thirdparty/*   (BehaviorTree.CPP v4, fast_tf, nlohmann, ankerl)
```

`libayanami` 不 include `ros/ros.h`、`tf2_ros`、`sensor_msgs/*`、`nav_msgs/*` 等任何 ROS 头；所有跨 ROS/核心 的消息都由 ayanami_ros 的 `ros_msg_converter.hpp` 在边界转换。

### 1.2 目录结构

```
src/ayanami/
  include/ayanami/
    core/                # 抽象插件接口与地图上下文
      abstract_planner.hpp abstract_controller.hpp
      abstract_smoother.hpp abstract_recovery.hpp
      abstract_navigation.hpp
      map_context.hpp      # 抽象 MapContext 基类
    components/          # 抽象执行/动作/服务器
      execution_base.hpp
      planner_execution.hpp smoother_execution.hpp
      controller_execution.hpp recovery_execution.hpp
      action_base.hpp action_server.hpp
      planner_action.hpp smoother_action.hpp
      controller_action.hpp recovery_action.hpp
      plugin_manager.hpp
      navigation_server_base.hpp   # 无地图, 4 PluginManager + 4 Action + virtual 钩子
      robot_state_provider.hpp
    behavior_tree/       # 行为树执行器与节点 (无地图依赖)
      pilot.hpp
      action_nodes.hpp
      condition_nodes.hpp
      pipeline_sequence.hpp rate_controller.hpp recovery_node.hpp
      navigation.hpp            # 注册节点 + 内置默认 XML
      blackboard_protocol.hpp   # 端口名常量 + 加载后校验
    interfaces/
      callbacks.hpp transform_store.hpp
    utility/
      config.hpp config_impl.hpp
      callback_slot.hpp load_module.hpp extension_module.hpp
      logging.hpp
    messages/            # Eigen 版自研消息
      nav_msgs/ geometry_msgs/ sensor_msgs/ actions/
  packages/              # 纯 C++ 子库 (从现 thirdparty/costmap_2d 迁移)
    costmap_2d/ costmap_layers/ costmap_server/
  modules/               # 每个插件一个独立 .so
    costmap_navigation/            # AbstractNavigation 宿主插件
    costmap_smac_planner/
    costmap_navfn_planner/
    costmap_savitzky_smoother/
    costmap_savitzky_golay_smoother/   # 可选
    costmap_pid_controller/
    costmap_mpc_controller/          # 可选
    costmap_spin_recovery/
    costmap_back_up_recovery/
    costmap_wait_recovery/
    costmap_clear_costmap_recovery/
  thirdparty/
    BehaviorTree.CPP/ fast_tf/ nlohmann/ ankerl/
  config/
    config.json
    config_ros.json
    costmap_navigation.json
    costmap.json
    costmap_smac_planner.json  costmap_navfn_planner.json
    costmap_savitzky_smoother.json  costmap_pid_controller.json
    costmap_spin_recovery.json  costmap_back_up_recovery.json
    costmap_wait_recovery.json  costmap_clear_costmap_recovery.json
    logging.json
    BT/navigation.xml
  tests/                # gtest 契约
  assets/                # rmuc26.json/pgm 等静态地图
  cmake/
    ayanami-config.cmake.in
  CMakeLists.txt

src/ayanami_ros/
  include/ayanami_ros/
    ayanami_ros.hpp
    rviz_viewer.hpp
    tf2_listener.hpp
    extension_module_ros.hpp      # 扩展模块的 ROS 形态
    ros_msg_converter.hpp
  src/ayanami_ros/
    ayanami_ros.cpp
    rviz_viewer.cpp
    tf2_listener.cpp
    extensions/
      rviz_viewer.cpp             # ROS 扩展实现 (导出 create_extension_module)
      # 也可后续添加 odom_saver.cpp 等 ROS 扩展
  apps/
    ayanami_rosnode.cpp
  launch/
    navigation.launch
    config.rviz
  CMakeLists.txt
  package.xml
```

### 1.3 工程组织（推翻重写的第一步）

- 新建 git 分支 `redesign`（在 ayanami 与 ayanami_ros 各自仓库）。
- 把当前 `src/ayanami/{include,modules,packages,config,assets,tests,CMakeLists.txt,package.xml,.clang-format}` 整体移到 `src/_legacy_ayanami/` 作只读参考（仍在 git 工作树，但不再被 CMake 扫描）；`thirdparty/` 原样保留。
- 同理 `src/ayanami_ros/*` → `src/_legacy_ayanami_ros/`。
- 在 `src/ayanami/` 与 `src/ayanami_ros/` 重新按 §1.2 铺骨架。
- 现有 6 个 gtest 用例（`tests/` 下）是稳定的接口契约，新核心上要复刻（不必逐行抄，要等价覆盖断言）。

---

## 2. 参数加载 (GlobalConfig / Config)

### 2.1 设计要点

- 对标 glim 的 `GlobalConfig` 单例 + `Config` 子配置。核心与 ayanami_ros 都只读 JSON，不依赖 ROS parameter server。
- **修复 glim 坑**：`GlobalConfig::instance` 用 `std::once_flag` 保护初始化；显式 `init(config_path) / shutdown()` 生命周期，避免裸指针 + `override_path=true` 时 `delete inst` 让旧引用失效的隐患。
- **`std::any` 隐藏 `nlohmann::json`**：公开头 `utility/config.hpp` 不 include `nlohmann/json.hpp`，重型 JSON 头挡在私有 `.cpp` 里（见现 ayanami 已实现，沿用）。
- `traits<T>` 适配：标量 `int / double / bool / string`、`Eigen::Vector2d/3d/4d`、`Eigen::Quaterniond`、`Eigen::Isometry3d`（TUM 格式 `[x,y,z,qx,qy,qz,qw]`）、`std::vector<Eigen::Isometry3d>`。沿用现 ayanami 实现。
- JSON 解析开启 `ignore_comments=true`，配置文件可带注释自解释。

### 2.2 路径解析

`GlobalConfig::instance(config_path)` 路径解析顺序（修现状 `config/` vs `configs/` 漂移坑）：

1. 若 `config_path` 是**文件**，取其父目录当作 `directory`。
2. 若 `config_path` 是**目录**：
   - 目录下直接有 `config.json` → 用之；
   - 否则若目录下有 `config/config.json` → 切到该子目录；
   - 否则若目录下有 `configs/config.json` → 切到 `configs/` 子目录（向后兼容）。
3. `override_param("global", "config_path", directory)` 把解析后的目录写回内存 JSON，形成自我引导闭环。

`GlobalConfig::get_config_path(config_name)`：`directory + "/" + param("global", config_name, config_name + ".json")`。

### 2.3 config.json 的 global 段

```jsonc
{
  "global": {
    "config_path": "",                      // 由 GlobalConfig 引导时覆盖
    "config_ros": "config_ros.json",        // ayanami_ros 专属参数
    "config_navigation": "costmap_navigation.json",   // BT + planner/smoother/controller/recovery id + 频率
    "config_costmap": "costmap.json",       // global/local costmap 尺寸/层/footprint
    "config_planner": "costmap_smac_planner.json",
    "config_smoother": "costmap_savitzky_smoother.json",
    "config_controller": "costmap_pid_controller.json",
    "config_recovery": "costmap_clear_costmap_recovery.json",
    "config_logging": "logging.json"
  }
}
```

换算法 = 改这里的文件名（对标 glim）。

### 2.4 ROS 入口

`AyanamiROS` 构造函数从 ROS param `~/config_path` 读取，默认 `ros::package::getPath("ayanami") + "/config"`；相对路径解析为绝对路径；随后 `GlobalConfig::instance(config_path)` 初始化。

### 2.5 红线

- `config_ros.json` 不出现 `planner_id`、`costmap` 尺寸、`BT` xml 等**核心参数**；只放 ROS 边界专属：topic 名、frame 名、tf 同步频率、`navigation_so_name` + `navigation_create_symbol`、`extension_modules` 列表。
- `logging_dir` 改为相对 `config_path/logs`（修现状写死 `/home/aphrora/...` 绝对路径的坑）。

---

## 3. 全局回调插槽 (CallbackSlot + NavigationCallbacks)

### 3.1 CallbackSlot 设计

`utility/callback_slot.hpp` 模板类：

```cpp
template <typename Func>
class CallbackSlot {
  std::vector<std::function<Func>> callbacks_;
  mutable std::shared_mutex mutex_;
public:
  int add(std::function<Func> cb);     // 返回 id (回调 vector 下标)
  void remove(int id);                 // 把该位置置 nullptr (洞复用)
  template <typename... Args>
  void call(Args&&... args) const;     // 遍历调用所有非空回调
  template <typename... Args>
  void operator()(Args&&... args) const { call(std::forward<Args>(args)...); }
  explicit operator bool() const;
};
```

**比 glim 强**：glim 的 `CallbackSlot` 完全无锁，依赖"构造期 add、永不 remove"约束；ayanami 用 `shared_mutex` 让运行时动态订阅/退订安全。

### 3.2 跨 .so 可靠性（关键）

**CallbackSlot 的静态聚合对象必须定义在 libayanami 的 `.cpp` 里，不能用 `static inline` 或 `inline static`**。
原因：用 header-only `inline static` 时，每个 `.so` 会各持一份静态对象实例，插件 `.so` 里 add 的回调和核心 emit 的回调落在不同实例上，静默失效（glim 文档明确警告；现 ayanami 已踩过此坑，已正确做法）。

`interfaces/callbacks.hpp` 只声明 `static CallbackSlot<...> on_xxx;`，实际 `inline` 定义在 `interfaces/impl/callbacks.cpp` 里编入 libayanami；插件 .so 链接 libayanami（DT_NEEDED），引用解析到同一个实例。

### 3.3 NavigationCallbacks 槽全集

```cpp
struct NavigationCallbacks {
  // 四组件 action 流 (goal / feedback / result)
  static CallbackSlot<void(const GetPathGoal&)>       on_get_path_goal;
  static CallbackSlot<void(const GetPathFeedback&)>    on_get_path_feedback;
  static CallbackSlot<void(const GoalResult<GetPathResult>&)>    on_get_path_result;
  static CallbackSlot<void(const SmoothPathGoal&)>     on_smooth_path_goal;
  static CallbackSlot<void(const SmoothPathFeedback&)> on_smooth_path_feedback;
  static CallbackSlot<void(const GoalResult<SmoothPathResult>&)> on_smooth_path_result;
  static CallbackSlot<void(const ExePathGoal&)>        on_exe_path_goal;
  static CallbackSlot<void(const ExePathFeedback&)>    on_exe_path_feedback;
  static CallbackSlot<void(const GoalResult<ExePathResult>&)>   on_exe_path_result;
  static CallbackSlot<void(const RecoveryGoal&)>       on_recovery_goal;
  static CallbackSlot<void(const RecoveryFeedback&)>   on_recovery_feedback;
  static CallbackSlot<void(const GoalResult<RecoveryResult>&)>  on_recovery_result;
  // 状态/输入
  static CallbackSlot<void(const PoseStamped&, const TwistStamped&)> on_robot_state_updated;
  static CallbackSlot<void(const TwistStamped&)>       on_velocity_command;
  static CallbackSlot<void(const std::string&, const OccupancyGrid&)> on_costmap_visualization;
  static CallbackSlot<void(const PointCloud2&)>        on_obstacle_pointcloud;
  static CallbackSlot<void(const LaserScan&)>          on_obstacle_laserscan;
};
```

### 3.4 主流 vs 旁路分工（关键架构决策）

主流（驱动核心运行）：**BT 节点直接调 `NavigationServerBase::requestPath/requestSmoothing/requestExecution/requestRecovery` 拿 `TaskHandle`**，completion 回调里 `BehaviorTree::emitWakeUpSignal()` 唤醒 BT `run()` 循环。**这条线不走 NavigationCallbacks**。

旁路（观测/扩展/可视化/向前兼容）：`NavigationCallbacks::on_*_result / on_velocity_command / on_costmap_visualization` 给：
- ROS extension（rviz_viewer 订阅 → 发 path / cmd_vel / costmap topic）；
- `registerPlannerFallback`（规划失败 → 自动触发默认 recovery action）；
- 用户自定义外部监控/可视化/日志。

**这一分工把核心控制流的因果清晰性放在 BT/Server direct-call 之上，避免 callback 隐式时序带来的竞态**（参考 ayanami 历史 markdown 中 `action_nodes.hpp` 由全局 callback 改为 TaskHandle 直连的修复轨迹）。

---

## 4. 插件 + extension 机制 (dlopen)

### 4.1 加载底层

`utility/load_module.hpp / impl/load_module.cpp`：
- `dlopen(so_name, RTLD_LAZY | RTLD_GLOBAL) + dlsym(handle, symbol)`。
- 模板 `load_module_from_so<Module>(so_name, symbol)` 返回 `std::shared_ptr<Module>`。
- **句柄从不 dlclose**——插件生命周期等于进程生命周期（对标 glim）。析构纪律在 §4.5。
- `dlopen` 失败只 `spdlog::error + return nullptr`。

### 4.2 BT.CPP 符号冲突规避（关键）

`libbehaviortree_cpp.so` 用 vendored tinyxml2；ROS 系统的 `libtinyxml2.so` 会被 ROS 加载，符号重名会导致 BT.CPP 在 `registerBehaviorTreeFromFile()` / `createTreeFromFile()` 内 XML 解析崩溃。**必须在 BehaviorTree.CPP 的 CMake 里给 `behaviortree_cpp` 加 `-Wl,-Bsymbolic`**，让其内部对 tinyxml2 的引用绑定到自身实例（现状已修复，本 spec 沿用）。

### 4.3 工厂符号约定

各模块导出 `extern "C"` 工厂函数，**只做无参构造**（对标 MBF；地图对象由后续 `initialize` 注入）：

| 模块 | 符号 |
|---|---|
| AbstractNavigation 宿主 | `create_navigation_module` → `AbstractNavigation*` |
| costmap planner | `create_costmap_planner_module` → `costmap::CostmapPlanner*` |
| costmap smoother | `create_costmap_smoother_module` → `costmap::CostmapSmoother*` |
| controller (costmap/local) | `create_controller_module` → `abstract::AbstractController*` |
| costmap recovery | `create_costmap_recovery_module` → `costmap::CostmapRecovery*` |

工厂符号名在子配置 json 的 `create_symbol` 字段中显式声明（对标现 ayanami），便于支持同族多种实现。

### 4.4 PluginManager

`components/plugin_manager.hpp` 模板类：

```cpp
template <typename PluginType>
class PluginManager {
  std::map<std::string, std::shared_ptr<PluginType>> plugins_;
  std::map<std::string, std::string> so_names_;
public:
  using InitFn = std::function<bool(const std::string&, std::shared_ptr<PluginType>)>;
  bool loadPlugin(const PluginSpec& spec, InitFn init_fn);
  bool loadSingleFromConfig(const Config& cfg,
                            const std::string& module_name,
                            const std::string& default_name,
                            InitFn init_fn);
  std::shared_ptr<PluginType> getPlugin(const std::string& name);
  bool hasPlugin(const std::string& name) const;
  std::vector<std::string> getLoadedNames() const;
  void clearPlugins();   // 析构前调用
};
```

`loadSingleFromConfig` 从子 json 读 `name + so_name + create_symbol`，`dlopen` 后调 init_fn 让调用方注入地图对象。

### 4.5 析构纪律

退出顺序（对标 MBF simple_navigation_server.cpp:60-73 的纪律，规避"插件对象析构时符号已被卸载"崩溃）：

1. 停所有 `RecoveryAction / ControllerAction / SmootherAction / PlannerAction`（先 join 线程，再 reset）。
2. `clearPlugins()` —— 释放 4 个 `PluginManager` 持有的插件对象（在 dlopen 句柄仍存活时析构）。
3. 卸载 AbstractNavigation 宿主插件（`CostmapNavigation`）。
4. dlopen 句柄随进程退出回收。

### 4.6 extension_modules 通用化（glim 式，关键）

**核心层**：
```cpp
// utility/extension_module.hpp
class ExtensionModule {
public:
  using Ptr = std::shared_ptr<ExtensionModule>;
  virtual ~ExtensionModule() = default;
  // 派生模块可在构造期 add NavigationCallbacks 订阅
  static Ptr load_module(const std::string& so_name);  // dlsym "create_extension_module"
  virtual void export_classes(const std::string& so_name) {}  // 序列化识别（可选）
};
extern "C" ExtensionModule* create_extension_module();
```

核心层 `ExtensionModule` 纯 C++，**不 include ros**；扩展模块在构造期可 `NavigationCallbacks::on_xxx.add(...)`。

**ayanami_ros 层**：
```cpp
// include/ayanami_ros/extension_module_ros.hpp
#include <ros/ros.h>
#include <ayanami/utility/extension_module.hpp>
class ExtensionModuleROS : public ayanami::ExtensionModule {
public:
  using Ptr = std::shared_ptr<ExtensionModuleROS>;
  struct GenericTopicSubscription {
    std::string topic; uint32_t queue_size{};
    std::type_infoConstPtr md5{};  // 简化, 见详化
    std::function<void(const TopicEvent&)> callback;
  };
  virtual std::vector<GenericTopicSubscription> create_subscriptions(ros::NodeHandle& nh) {
    return {};
  }
};
```

`AyanamiROS::loadExtensionModules(config_ros)` 读 `config_ros.json` 的 `extension_modules: ["librviz_viewer.so", ...]`，逐个 `ExtensionModule::load_module`，保存到 `extension_modules_` 列表；`dynamic_pointer_cast<ExtensionModuleROS>` 成功则 `create_subscriptions(nh)` 并保存订阅句柄。

**rviz_viewer 改为 ROS 扩展 .so**：导出 `create_extension_module`，构造期 `NavigationCallbacks::on_get_path_result.add(...)` 等订阅，把核心消息转 ROS topic 发布。删除现状 ayanami_ros.cpp 的手工特判 `if name == "rviz_viewer" / "librviz_viewer.so"`。

---

## 5. 抽象导航层 + 地图形式无关 (对标 move_base_flex)

### 5.1 抽象插件接口 (core/)

```cpp
// abstract_planner.hpp
class AbstractPlanner {
public:
  using Ptr = std::shared_ptr<AbstractPlanner>;
  virtual ~AbstractPlanner() = default;
  virtual uint32_t makePlan(const PoseStamped& start, const PoseStamped& goal,
                            double tolerance, Path& plan,
                            double& cost, std::string& message) = 0;
  virtual bool cancel() { return false; }
  // 错误码: 0=SUCCESS, 50+ 为错误段 (NO_PATH_FOUND=58 等)
};

// abstract_controller.hpp
class AbstractController {
public:
  using Ptr = std::shared_ptr<AbstractController>;
  virtual uint32_t computeVelocityCommands(const PoseStamped& pose,
                                            const TwistStamped& velocity,
                                            TwistStamped& cmd_vel,
                                            std::string& message) = 0;
  virtual bool isGoalReached(double dist_tol, double angle_tol) = 0;
  virtual bool setPlan(const Path& plan) = 0;
  virtual bool cancel() { return false; }
  // 错误码: 100+ 为错误段
};

// abstract_smoother.hpp
class AbstractSmoother {
public:
  using Ptr = std::shared_ptr<AbstractSmoother>;
  virtual uint32_t smoothPath(const Path& path, Path& smoothed_path,
                              double max_duration, bool check_for_collision,
                              std::string& message) = 0;
  virtual bool cancel() { return false; }
  // 错误码: 200+
};

// abstract_recovery.hpp  (注意: 不叫 AbstractBehavior)
class AbstractRecovery {
public:
  using Ptr = std::shared_ptr<AbstractRecovery>;
  virtual uint32_t runRecovery(std::string& message) = 0;
  virtual bool cancel() { return false; }
  // 错误码: 300+
};

// abstract_navigation.hpp  (宿主插件入口)
class AbstractNavigation {
public:
  using Ptr = std::shared_ptr<AbstractNavigation>;
  virtual ~AbstractNavigation() = default;
  virtual void initialize(const std::string& config_path) = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void spinOnce() = 0;
  virtual void setGoal(const PoseStamped& goal) = 0;
  using MapCallback = std::function<void(const std::string&, const OccupancyGrid&)>;
  virtual void setMapCallback(MapCallback cb) { (void)cb; }
};
```

`include/ayanami/core/` 全部只依赖自研 `messages/`，**零地图类型**。

### 5.2 抽象 MapContext 基类 (新引入，关键)

```cpp
// core/map_context.hpp
class AbstractMapContext {
public:
  using Ptr = std::shared_ptr<AbstractMapContext>;
  virtual ~AbstractMapContext() = default;
  virtual const std::string& getGlobalFrame() const = 0;
  virtual const std::string& getRobotFrame() const = 0;
  virtual bool isCurrent() const { return true; }   // 地图是否新鲜
  virtual void lock() {}                            // 写锁 (规划/控制加锁用)
  virtual void unlock() {}
  // 获取机器人在 global_frame 的位姿 (由 TransformStore 提供, 这里只是转发)
  virtual bool getRobotPose(PoseStamped& pose) const = 0;
};
```

`NavigationServerBase` 持 `AbstractMapContext::Ptr`，能做**通用 `safetyCheck`**（`if (!map_context_->isCurrent()) return false;`），无需知道是 costmap 还是 gridmap。

`CostmapContext : public AbstractMapContext` 包 global/local `LayeredCostmap`，覆写 `isCurrent/lock/unlock` 为 costmap 的真实语义。

本轮 `GridmapContext` 只预留**空头基类**（沿用 AbstractMapContext 默认实现），供未来 gridmap_navigation 实例化。

### 5.3 NavigationServerBase (无地图)

```cpp
class NavigationServerBase {
protected:
  PluginManager<AbstractPlanner>    planner_plugins_;
  PluginManager<AbstractSmoother>  smoother_plugins_;
  PluginManager<AbstractController> controller_plugins_;
  PluginManager<AbstractRecovery>   recovery_plugins_;
  std::unique_ptr<PlannerAction>    planner_action_;
  std::unique_ptr<SmootherAction>   smoother_action_;
  std::unique_ptr<ControllerAction> controller_action_;
  std::map<std::string, std::unique_ptr<RecoveryAction>> recovery_actions_;
  RobotStateProvider robot_state_provider_;
  AbstractMapContext::Ptr map_context_;    // 通用, 派生层注入
  std::string default_planner_, default_smoother_, default_controller_, default_recovery_;
public:
  virtual ~NavigationServerBase() = default;
  void loadPlugins(const std::string& config_path);     // 读 config.json + 各 sub-config, 调虚 initialize*
protected:
  // ★ 改虚: 派生 (CostmapNavigationServer) 覆写以注入地图
  virtual bool initializePlannerPlugin(const std::string& name, AbstractPlanner::Ptr) { return true; }
  virtual bool initializeSmootherPlugin(const std::string& name, AbstractSmoother::Ptr) { return true; }
  virtual bool initializeControllerPlugin(const std::string& name, AbstractController::Ptr) { return true; }
  virtual bool initializeRecoveryPlugin(const std::string& name, AbstractRecovery::Ptr) { return true; }
  // ★ 地图钩子: 派生覆写以激活/停用地图, 加锁等
  virtual std::unique_ptr<PlannerExecution>    newPlannerExecution(...) { return /* abstract 版 */; }
  virtual std::unique_ptr<ControllerExecution> newControllerExecution(...) { return /* abstract 版 */; }
  virtual std::unique_ptr<RecoveryExecution>   newRecoveryExecution(...) { return /* abstract 版 */; }
  // ★ Task API: BT 节点直接调用 (不走 NavigationCallbacks)
  TaskHandle requestPath(const GetPathGoal&);
  TaskHandle requestSmoothing(const SmoothPathGoal&);
  TaskHandle requestExecution(const ExePathGoal&);
  TaskHandle requestRecovery(const RecoveryGoal&);
};
```

### 5.4 CostmapNavigationServer : NavigationServerBase (costmap 特化)

```cpp
namespace costmap {
class CostmapNavigationServer : public NavigationServerBase {
  CostmapContext::Ptr costmap_context_;   // 持 global/local LayeredCostmap
  CostmapWrapper global_costmap_, local_costmap_;
public:
  // 覆写虚钩子
  bool initializePlannerPlugin(...) override {
    auto p = std::dynamic_pointer_cast<CostmapPlanner>(ptr);
    p->initialize(name, costmap_context_);
    return p != nullptr;
  }
  // controller / recovery / smoother 同理 (按族分裂接口 §5.5)
  std::unique_ptr<PlannerExecution> newPlannerExecution(...) override {
    return std::make_unique<CostmapPlannerExecution>(name, p, global_costmap_);
  }
  std::unique_ptr<ControllerExecution> newControllerExecution(...) override {
    return std::make_unique<CostmapControllerExecution>(name, c, local_costmap_);
  }
  // 配置 StaticLayer / InflationLayer / SimpleObstacleLayer
  void configureCoreMaps(const std::string& config_path);
  // 处理 on_obstacle_pointcloud / on_obstacle_laserscan -> 经 TransformStore 转 frame -> 更新 obstacle layer
  void handleObstaclePointCloud(const PointCloud2&);
  void handleObstacleLaserScan(const LaserScan&);
};
}
```

### 5.5 地图族分裂接口 (按族继承)

```cpp
// costmap/core/costmap_planner.hpp
namespace costmap {
class CostmapPlanner : public abstract::AbstractPlanner {
public:
  using Ptr = std::shared_ptr<CostmapPlanner>;
  virtual void initialize(const std::string& name, CostmapContext::Ptr) = 0;
};
// costmap_smoother.hpp / costmap_controller.hpp / costmap_recovery.hpp 同理
}
```

> **本轮不动 gridmap**：gridmap/core/ 仅留 `GridmapPlanner/GridmapSmoother`（保留），新增 `GridmapController/GridmapRecovery/GridmapNavigationServer/GridmapNavigation` 留到下一轮。但本配置下的 `initialize*Plugin` virtual + 抽象 MapContext 让 gridmap_navigation 未来能对称派生。

### 5.6 CostmapNavigation 宿主插件 (modules/costmap_navigation/)

```cpp
class CostmapNavigation : public AbstractNavigation {
  CostmapNavigationServer navigation_server_;
  BehaviorTreePilot pilot_;
  MapCallback map_callback_;
  std::unique_ptr<std::thread> navigation_thread_;
public:
  void initialize(const std::string& config_path) override {
    navigation_server_.loadPlugins(config_path);    // 触发 initialize*Plugin virtual 链
    navigation_server_.createActions();
    pilot_.configure(config_path);                  // 读 navigation 段 (BT xml, 频率, ids)
    pilot_.setNavigationServer(&navigation_server_);
  }
  void start() override;
  void stop() override;
  void spinOnce() override { publishMapsForVisualization(); }
  void setGoal(const PoseStamped& goal) override;  // 启 BT 线程
  void setMapCallback(MapCallback cb) override { map_callback_ = std::move(cb); }
};
}
extern "C" CostmapNavigation* create_navigation_module();
```

### 5.7 决策红线

- `include/ayanami/core/**` 与 `include/ayanami/components/**` **禁止 include `costmap_2d/*` 或 `gridmap_2d/*`**。
- 一个新文件被 review 前要单测"零地图 include"（脚本 grep 检查）。
- 地图对象经 `initialize*Plugin` 虚函数注入；create 工厂不做参数构造。

---

## 6. 行为树编排 (BT.CPP v4, nav2 风格精简)

### 6.1 BehaviorTreePilot (无地图依赖)

```cpp
// components/behavior_tree/pilot.hpp
class BehaviorTreePilot {
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  NavigationServerBase* nav_server_ = nullptr;
  std::atomic<bool> running_{false};
  std::atomic<bool> kill_switch_{false};
  std::thread tick_thread_;
  double tick_frequency_ = 100.0;
  // 任务会话
  std::atomic<uint64_t> session_id_{0};
  std::atomic<uint64_t> plan_version_{0};
  std::mutex goal_mutex_;
  std::optional<PoseStamped> pending_goal_;
public:
  void configure(const std::string& config_path);   // 读 navigation 段
  void setNavigationServer(NavigationServerBase* s) { nav_server_ = s; }
  void loadFromFile(const std::string& xml_path, const std::string& tree_id);
  void beginNavigation(const PoseStamped& goal);   // 设置 pending_goal_, emitWakeUpSignal
  void run();                                       // tick 循环 (100Hz, sleep 可唤醒)
  void halt();                                      // 停所有 RUNNING action
  void stop();                                      // join tick_thread_
private:
  void applyPendingGoal();                          // 把 pending_goal_ 写入黑板
};
```

**关键纪律**（规避历史坑）：

- **树常驻不重建**。新 goal 只 `applyPendingGoal()` 重写黑板（规避 `recreateTree()` 重量级 + `IsRobotStuckNode` 静态状态跨任务残留坑）。`session_id_++`、`plan_version_ = 0`。
- 新 goal 抢占：先经 `NavigationCallbacks::on_velocity_command` 发零速度；`controllerAction()->cancelAll()`；`pilot_.halt()`；移出锁外 `join()` 旧导航线程；再 `beginNavigation(new_goal)`。

### 6.2 BT 节点直接调 NavigationServer

```cpp
// components/behavior_tree/action_nodes.hpp
class ComputePathToPoseNode : public BT::StatefulActionNode {
  NavigationServerBase* nav_server_;
  TaskHandle task_;
  uint64_t session_id_, plan_version_;
public:
  BT::NodeStatus onStart() override {
    PoseStamped goal = getInput<PoseStamped>("goal").value();
    PoseStamped start; // 留空, 由 planner 默认取 robot pose
    GetPathGoal g; g.goal = goal; g.start = start;
    g.session_id = nav_server_->currentSessionId();
    g.plan_version = nav_server_->currentPlanVersion();
    task_ = nav_server_->requestPath(g);
    task_.onComplete([this](auto){ BehaviorTree::emitWakeUpSignal(); });
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if (!task_.ready()) return BT::NodeStatus::RUNNING;
    auto r = task_.result();
    if (!nav_server_->sessionIsCurrent(r.session_id)) return BT::NodeStatus::FAILURE;
    if (r.status != GoalStatus::SUCCEEDED) return BT::NodeStatus::FAILURE;
    setOutput("raw_path", r.path);
    nav_server_->incrementPlanVersion();   // smoother / controller 据此识别新路径
    return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override { task_.cancel(); }
};
// SmoothPathNode / FollowPathNode / RecoveryActionNode 同构
```

**session_id / plan_version 校验**：规避 BT action 结果竞态（`goal_id == 0` 的"新 goal 绑定窗口"丢结果的旧 bug），用 session/version 替代 goal_id 比较。

### 6.3 默认 BT XML (nav2 风格精简)

`config/BT/navigation.xml`（与内置字符串同构，文件可覆盖）：

```xml
<root BTCPP_format="4" main_tree_to_execute="NavigateWithReplanningAndSmoothing">
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
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence><Recovery recovery="{spin_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{wait_id}"/>    <AlwaysFailure/></Sequence>
          <Sequence><Recovery recovery="{backup_id}"/>  <AlwaysFailure/></Sequence>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

**保留节点**（对标 nav2 必须项）：PipelineSequence、RateController、RecoveryNode（内+外两层）、RoundRobin、GoalUpdated、Recovery + AlwaysFailure 封装。

**剪枝节点**（非核心，本轮不做）：`PathLongerOnApproach + CancelControl + Wait` 临近目标耐心、`IsPathValid` 触发按失效重规划的 Fallback、电池类条件节点。

### 6.4 频率分层（对标 nav2 解耦）

| 层面 | 频率控制者 | 机制 |
|---|---|---|
| BT tick | `BehaviorTreePilot::run()` 的 `WallRate(100Hz)` | BT.CPP `sleep` 可被 `emitWakeUpSignal` 唤醒 |
| 重规划 | BT XML `RateController hz={planner_frequency}` | 门控 ComputePathToPose；`-1` 表示单次后缓存 SUCCESS |
| 平滑 | SmoothPath 一次性 action | `max_duration` 限时 |
| cmd_vel | controller_server 自己的循环（`controller_frequency`，默认 20Hz） | 调 AbstractController::computeVelocityCommands |
| recovery | `RecoveryExecution::run()` 单次 | `recovery_patience` |

**关键**：BT tick 频率下降不会导致 cmd_vel 卡顿（应用的现 ayanami 修复轨迹）；重规划与控制完全解耦。

### 6.5 黑板协议 (规避端口运行时 lookup 失败坑)

`blackboard_protocol.hpp` 集中定义所有端口名常量：`kGoal / kRawPath / kSmoothedPath / kPlannerFrequency / kControllerFrequency / kRecoveryId / kClearCostmapId / kSpinId / kWaitId / kBackupId / kSessionId / kPlanVersion`。

`loadFromFile` 后立即 `validateRequiredBlackboardEntries`（在 BT.CPP `createTree` 之后扫一次所有端口，缺失立即 throw），把运行时端口名错误变编译期/启动期失败。

带记忆节点（`IsRobotStuckNode` 等）用 `reset_stuck_token` 黑板令牌在每轮导航开始时复位（规避 static 状态跨任务残留坑）。

### 6.6 recovery 命名统一（修现状 bug）

| recovery 插件 | 模块目录 | 类名 | json name | create 符号 |
|---|---|---|---|---|
| 原地旋转 | `modules/costmap_spin_recovery/` | `CostmapSpinRecovery` | `costmap_spin_recovery` | `create_costmap_recovery_module` |
| 后退 | `modules/costmap_back_up_recovery/` | `CostmapBackUpRecovery` | `costmap_back_up_recovery` | 同上 |
| 等待 | `modules/costmap_wait_recovery/` | `CostmapWaitRecovery` | `costmap_wait_recovery` | 同上 |
| 清代价地图 | `modules/costmap_clear_costmap_recovery/` | `CostmapClearCostmapRecovery` | `costmap_clear_costmap_recovery` | 同上 |

`costmap_navigation.json` 中 `recovery_id` 列表与 `config_recovery` 应指向同一组 `name`（修现状 `recovery_id="costmap_cleanup"` 与插件 `name="costmap_cleanup_recovery"` 不匹配的坑，导致 BT 恢复分支实际失败）。

---

## 7. ayanami_ros 边界 (ROS1 catkin)

### 7.1 AyanamiROS 职责

```cpp
class AyanamiROS {
  ros::NodeHandle nh_;
  std::string config_path_;
  AbstractNavigation::Ptr navigation_plugin_;   // dlopen costmap_navigation 宿主插件
  std::vector<ayanami::ExtensionModule::Ptr> extension_modules_;
  std::vector<ros::Subscriber> extension_subs_;
  TF2Listener tf_listener_;
  // ROS 订阅
  ros::Subscriber goal_sub_, pointcloud_sub_, odom_sub_, laserscan_sub_;
  TwistStamped latest_velocity_;
  std::atomic<bool> kill_switch_{false};
  std::thread loop_thread_;
  double tf_sync_rate_ = 50.0;
public:
  AyanamiROS(ros::NodeHandle& nh);
  ~AyanamiROS();
  void stop();
private:
  void loadNavigationPlugin(const Config& config_ros);
  void loadExtensionModules(const Config& config_ros);
  void subscribeTopics();
  void goalCallback(const geometry_msgs::PoseStampedConstPtr&);
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr&);
  void laserscanCallback(const sensor_msgs::LaserScanConstPtr&);
  void odomCallback(const nav_msgs::OdometryConstPtr&);
  void syncTransformsToCore();    // ROS tf -> TransformStore + on_robot_state_updated
  void loop();                    // tf_sync_rate Hz: syncTransformsToCore + navigation_plugin->spinOnce + ros::spinOnce
};
```

### 7.2 初始化流程（关键顺序）

```
1. ROS param "config_path" -> GlobalConfig::instance(config_path)
2. Config config_ros(GlobalConfig::get_config_path("config_ros"))
3. TF2Listener 构造 (tf2_ros::Buffer + TransformListener)
4. loadExtensionModules(config_ros)        ← 先加载 (rviz_viewer 等注册 callback)
5. loadNavigationPlugin(config_ros)       ← dlopen CostmapNavigation, initialize(config_path), start
6. subscribeTopics()                       ← goal / pointcloud / odom / laserscan
7. loop_thread_ 启动
```

> 顺序关键：rviz_viewer 等 ROS 扩展必须**先于** navigation_plugin->start() 注册其 NavigationCallbacks 订阅，否则启动期 `navigateOrRecover` 立即触发的第一波发布会错过。

### 7.3 ROS -> 核心 (subscribeTopics)

| ROS 消息 | 核心 |
|---|---|
| `geometry_msgs::PoseStamped` (`/move_base_simple/goal`) | 转 Eigen `PoseStamped` → `navigation_plugin->setGoal` |
| `sensor_msgs::PointCloud2` | 转 `ayanami::sensor_msgs::PointCloud2` → `NavigationCallbacks::on_obstacle_pointcloud` |
| `sensor_msgs::LaserScan` (可选) | 转 `ayanami::sensor_msgs::LaserScan` → `on_obstacle_laserscan` |
| `nav_msgs::Odometry` | 存 `latest_velocity_;` 由 `syncTransformsToCore` 合成 `PoseStamped + TwistStamped` 经 `on_robot_state_updated` 推入 |

### 7.4 TF -> 核心

`syncTransformsToCore`（`tf_sync_rate` 50Hz）：对三条边 `map->odom`、`odom->base_link`、`map->base_link` 调 `tf2_listener_.lookup_transform`，转 `Eigen::Isometry3d` 后 `TransformStore::instance().setTransform`。同时把机器人 Pose + latest_velocity_ 合成经 `NavigationCallbacks::on_robot_state_updated`。

**改进**（相对现状）：`stamp` 统一用消息自带时间戳（替代现状一律写 `fromSeconds(0.0)`），支持精确时间查询（controller 延迟控制等场景）。

### 7.5 核心 -> ROS (由扩展模块订阅 NavigationCallbacks)

`rviz_viewer` ROS 扩展（`src/extensions/rviz_viewer.cpp`）构造期：

```cpp
NavigationCallbacks::on_get_path_result.add([this](auto& r){ publishPlannerPath(r); });
NavigationCallbacks::on_smooth_path_result.add([this](auto& r){ publishSmootherPath(r); });
NavigationCallbacks::on_velocity_command.add([this](auto& v){ publishCmdVel(v); });
NavigationCallbacks::on_costmap_visualization.add([this](auto& name, auto& grid){ publishCostmap(name, grid); });
```

扩展还可在 `create_subscriptions(nh)` 注册自己的额外话题（自定义注入信号等）。

### 7.6 职责红线

- `ayanami_ros` **不 include `costmap_2d/*`**（由 `ros_msg_converter.hpp` 间接发布的 costmap 视图只接核心消息 `OccupancyGrid`，不暴露 costmap 类型）。
- `ayanami_ros` 不直接持有 `BehaviorTreePilot`（已由 `AbstractNavigation` 宿主插件封装；ROS 层只调 `initialize/start`）。
- `ayanami_ros` 不知道 `planner_id / costmap` 尺寸 / BT xml 等核心参数。

---

## 8. FastTF / TransformStore

`interfaces/transform_store.hpp`：单例，封装 `fast_tf::detail::transform_buffer`（线程安全，带 static/timed cache）。

```cpp
class TransformStore {
  fast_tf::detail::transform_buffer buffer_;
public:
  static TransformStore& instance();
  void setTransform(const std::string& parent, const std::string& child,
                    TimePoint stamp, const Eigen::Isometry3d& T, bool is_static = false);
  bool lookupTransform(const std::string& target, const std::string& source,
                       TimePoint stamp, Eigen::Isometry3d& out, double timeout = 0.0) const;
  static TimePoint fromSeconds(double s);
};
```

写入者：`AyanamiROS::syncTransformsToCore`（周期）+ `laserscanCallback` 的即时同步。
读取者：核心 `NavigationServer` 把点云从 sensor frame 转 costmap frame；`CostmapServer` 让 rolling costmap 跟随机器人；BT `ComputePathToPoseNode` 取 robot pose 作 start。

`fast_tf` 为 glim 移植版（thirdparty/fast_tf/），原样复用，不做改动。

---

## 9. 配置文件组织

### 9.1 `config/config.json`

见 §2.3。仅含 `global` 段映射 + `logging` 段（`logging_dir` 相对 `config_path/logs`）。

### 9.2 `config/config_ros.json`

```jsonc
{
  "ayanami_ros": {
    "pointcloud_topic": "/velodyne_points",
    "odom_topic":       "/odom",
    "goal_topic":       "/move_base_simple/goal",
    "laserscan_topic":  "",
    "cmd_vel_topic":    "/cmd_vel",
    "planner_path_topic":     "/ayanami/planner/path",
    "smoother_path_topic":    "/ayanami/smoother/path",
    "controller_velocity_topic": "/ayanami/controller/velocity_commands",
    "map_frame":        "map",
    "odom_frame":       "odom",
    "robot_frame":      "base_footprint",
    "tf_sync_rate":     50.0,
    "navigation_so_name":      "libayanami_costmap_navigation.so",
    "navigation_create_symbol":"create_navigation_module",
    "extension_modules": ["librviz_viewer.so"]
  }
}
```

### 9.3 `config/costmap_navigation.json`

```jsonc
{
  "navigation": {
    "behavior_tree_xml": "BT/navigation.xml",
    "behavior_tree_id":  "NavigateWithReplanningAndSmoothing",
    "planner_id":        "costmap_smac",
    "smoother_id":       "costmap_savitzky",
    "controller_id":     "costmap_pid",
    "recovery_id":       "costmap_clear_costmap",
    "clear_costmap_id":  "costmap_clear_costmap",
    "spin_id":           "costmap_spin",
    "wait_id":           "costmap_wait",
    "backup_id":         "costmap_back_up",
    "planner_frequency":   -1.0,    // -1 单次; >0 按 Hz
    "smoother_frequency":  -1.0,
    "controller_frequency":20.0,
    "goal_tolerance":      0.25,
    "angle_tolerance":     0.20,
    "robot_frame":         "base_footprint",
    "costmap_visualization_rate": 1.0
  }
}
```

### 9.4 `config/costmap.json`

global/local 尺寸 / 分辨率 / origin / rolling `bool` / static_layer（map_path）/ inflation_layer（radius / clamp）/ obstacle_layer（obstacle_range / raytrace_range / observation_sources） / footprint。

### 9.5 各插件 json + `BT/navigation.xml`

每个模块一个子 json，含 `name / so_name / create_symbol` + 算法参数；用户每个子 json 可独立。

### 9.6 红线

- `config_ros.json` 不出现 `planner_id` / costmap 尺寸 / BT xml 等核心参数。
- `costmap_navigation.json` 不出现 topic 名 / frame 名。
- 单一来源帧名：`costmap_navigation.json::robot_frame`（核心消费）；`config_ros.json::robot_frame`（ROS TF 同步消费）—— 构造期校验二者一致，否则报错。

---

## 10. 已知坑的规避汇总

| 历史/现状坑 | 根因 | 本 spec 规避 |
|---|---|---|
| 跨 .so 静态 slot 失效 | 槽对象 `inline static` 在每个 .so 各持一份 | 槽对象定义在 libayanami 的 .cpp 内一次（不 inline） |
| BT.CPP vendored tinyxml2 符号冲突 | 系统 libtinyxml2 重名符号抢绑 | `behaviortree_cpp` 加 `-Wl,-Bsymbolic` |
| recreateTree 重量级 + static 状态泄漏 | 每 goal 整树破坏重建 | 树常驻不重建；pending_goal 仅重写黑板 |
| BT action 结果竞态（goal_id 0 窗口丢结果） | 先注册回调再 call action，快速完成时回调里 goal_id 还是 0 | 用 `session_id / plan_version` 校验，替代 `goal_id` 比较 |
| CallbackSlot 无锁竞态 | glim 无锁严格依赖"启动期 add" | `shared_mutex` 保护，运行时 add/remove 安全；主流不走 callback |
| `config/` vs `configs/` 漂移 | CMake install 与运行时拼接目录名不一致 | 统一 `config/`；GlobalConfig 路径解析兜底兼容 `configs/` |
| `logging_dir` 写死绝对路径 | `config.json::logging_dir` 写 `/home/aphrora/...` | 改为相对 `config_path/logs` |
| 帧名 base_footprint / base_link 混用 | 不同子配置各自声明 | 单一来源 `costmap_navigation.json::robot_frame`，与 `config_ros.json::robot_frame` 构造期一致校验 |
| recovery_id 与插件 name 不匹配 | `recovery_id="costmap_cleanup"` 但插件 `name="costmap_cleanup_recovery"` | 统一命名 `costmap_*_recovery`，json 与 BT XML 端口一致 |
| CMake 缺 `ayanami-config.cmake.in` | downstream `find_package(ayanami)` 解析不到 | 补 `cmake/ayanami-config.cmake.in`，导出 libayanami + 组件目标 + INCLUDE_DIRS |
| TwistStamped 未初始化脏内存 | `{}` 缺省构造后未显式清零 | `cmd_vel = geometry_msgs::TwistStamped{};` 纪律保留；所有 stop/park 命令全字段清零 |
| RateController 语义混乱 | hz<=0 当每 tick 运行 | 对齐 nav2：hz>0 最大频率且首 tick 立即；`-1` 单次后缓存 SUCCESS；明确文档化 |
| dlopen 句柄管理 | 早期版本未 dlclose 也没问题，但插件对象析构时机容易错 | 退出顺序纪律 §4.5：先清 action → clearPlugins → 卸载宿主 → 句柄随进程 |
| 振荡检测只有一级 | 现状未做 oscillation 检测 | 后续阶段补两级：controller action 级（单路径内不前进）+ Navigation/BT 级（resolve fast-failure 链） |

---

## 11. 实施步骤概要（覆盖下一阶段）

退出 plan mode 后调用 `writing-plans` skill 产出可执行实施计划。本节先给出阶段切分。

### 阶段 A：核心打通（能跑一次导航）
1. 建 git 分支；旧代码归档 `_legacy_*`；`thirdparty` 复用。
2. 铺顶层骨架；CMakeLists（libayanami shared、`ayanami_add_module` 宏、packages 子库、`cmake/ayanami-config.cmake.in`）。
3. **核心层**（按依赖序）：
   - `utility/`：config / callback_slot / load_module / extension_module / logging
   - `interfaces/`：TransformStore + NavigationCallbacks（槽体定义在 .cpp）
   - `messages/`：nav_msgs / geometry_msgs / sensor_msgs / actions
   - `core/`：AbstractPlanner/Controller/Smoother/Recovery/Navigation + AbstractMapContext
   - `components/`：ExecutionBase + *_execution + ActionBase + action_server + *_action + PluginManager + NavigationServerBase + RobotStateProvider
   - `behavior_tree/`：Pilot + action_nodes + condition_nodes + 控制节点 + RegisterNodes + 默认 XML + blackboard_protocol
4. **地图层**：
   - `packages/costmap_2d`（从现 thirdparty/costmap_2d 迁回 packages，改用 `namespace costmap_2d`）
   - `packages/costmap_layers`（StaticLayer / InflationLayer / SimpleObstacleLayer）
   - `packages/costmap_server`（CostmapWrapper / Lifetime 管理）
   - `CostmapContext : AbstractMapContext`
   - `CostmapNavigationServer : NavigationServerBase`
   - `CostmapPlanner/Controller/Recovery Execution + Action`
5. **模块**：costmap_smac_planner / costmap_navfn_planner / costmap_savitzky_smoother / costmap_pid_controller / costmap_spin_recovery / costmap_back_up_recovery / costmap_wait_recovery / costmap_clear_costmap_recovery
6. **宿主**：costmap_navigation 宿主（导出 `create_navigation_module`）

### 阶段 B：ROS 边界 + 测试 + 联调
7. **ROS 边界**：ExtensionModule 基类 + ExtensionModuleROS + rviz_viewer ROS 扩展 .so + ayanami_ros（AyanamiROS + TF2Listener + ros_msg_converter）+ launch/config
8. **gtest 契约**：复刻 6 个接口契约 + 新增 BT 集成测试 + "零地图 include"脚本检查
9. 构建验证：`catkin build ayanami ayanami_ros` 全绿；rviz 联调一次完整导航（点 goal → 路径 → 速度 → 抵达）
10. spec doc + 实施 plan 双双 commit 到 git；阶段性 commit 推到 origin

### 工作量评估

- 核心库 + ~8 插件 .so + 3 packages + ROS 边界 + 6+ 测试。
- 阶段 A 是主体（最大）；阶段 B 验证 + 收尾。
- 建议每个里程碑独立 commit，便于回滚和 review。

---

## 12. 出 spec 自审清单

- [x] 无 TBD / TODO 占位。
- [x] §5.7 与 §6.1 红线一致（abstract 层零地图 include ∈ BT 层无地图依赖相互对齐）。
- [x] §3.4 主流与旁路分工贯穿全文：BT 直连 TaskHandle（§6.2），NavigationCallbacks 仅给 ROS 扩展/可视化/回退（§7.5、§3.4）。
- [x] behavior→recovery 改名一致性：核心类、Action、Execution、模块目录、json name、BT 节点、recovery_id 端口全部 `recovery`；仅保留 BT 语境用词。
- [x] §10 已知坑清单逐项有规避。
- [x] 配置目录统一 `config/`，GlobalConfig 路径解析兜底 `configs/` 兼容。
- [x] initialize*Plugin 改 virtual + 用户确认 + AbstractMapContext 基类收口。
- [x] gridmap 本轮不动（§5.5 明示）。
- [x] extension 改为通用 dlopen `create_extension_module`（§4.6）；rviz_viewer 也走这套，删除手工特判。
- 下一阶段：调用 writing-plans skill 出可执行实施计划，计划里每个 task 都带 verifiable success criteria（构建命令、测试命令、grep 检查）。

---

## 附：参考项目对应表

| ayanami 抽象 | glim 等价 | MBF 等价 | nav2 等价 |
|---|---|---|---|
| `GlobalConfig` | `glim::GlobalConfig` | (无, 用 ROS param) | (无, 用 rclcpp 参数) |
| `Config` 子配置 | `glim::Config` | (无) | (无) |
| `CallbackSlot` | `glim::CallbackSlot` | (无) | (无) |
| `NavigationCallbacks` | `*Callbacks` 聚合 | (无) | (无) |
| `load_module_from_so` | `glim::load_module_from_so` | `pluginlib::ClassLoader` | `pluginlib::ClassLoader` |
| `ExtensionModule` | `glim::ExtensionModule` | (无) | (无, 生命周期节点替代) |
| `AbstractPlanner` + `CostmapPlanner` | (无) | `mbf_abstract_core::AbstractPlanner` + `mbf_costmap_core::CostmapPlanner` | `nav2_core::GlobalPlanner` |
| `AbstractController` + `CostmapController` | (无) | `AbstractController` + `CostmapController` | `nav2_core::Controller` |
| `AbstractSmoother` | (无) | (无) | `nav2_core::Smoother` |
| `AbstractRecovery` (不叫 Behavior) | (无) | `AbstractRecovery` + `CostmapRecovery` | `nav2_core::Behavior` |
| `AbstractMapContext` | (无) | `Costmap2DROS*` (注入对象) | (无, costmap 节点持有) |
| `NavigationServerBase` | (无) | `AbstractNavigationServer` | (无, BT 替代) |
| `CostmapNavigationServer` | (无) | `CostmapNavigationServer` | `controller_server + planner_server` 等节点 |
| `CostmapNavigation` 宿主 | (无) | `MoveBaseAction`(状态机) — ayanami 用 BT | `bt_navigator` |
| `BehaviorTreePilot` | (无) | (无, 状态机替代) | `BtActionServer +Navigator` |
| nav2 风格 BT XML | (无) | (无) | `navigate_w_replanning_and_recovery.xml` |