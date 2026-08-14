# ayanami 核心重写 Plan 1 (M1: 核心可独立验证)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 重写 ayanami 纯 C++ 导航核心库到可被宿主插件 `libayanami_costmap_navigation.so` 完成"goal→planner→smoother→controller→cmd_vel"全链路的程度，全程零 ROS 依赖。

**Architecture:** 对标 glim/MBF/nav2 的核心层：GlobalConfig 参数加载、CallbackSlot 旁路、dlopen 插件 + extension、AbstractNavigation 抽象 + virtual initialize*Plugin 注入 AbstractMapContext、BehaviorTreePilot 常驻树 + TaskHandle 直连、costmap 全栈落地为 CostmapNavigation 宿主插件。

**Tech Stack:** C++17、CMake 3.16+、catkin（building）、Eigen、BT.CPP v4（vendored）、fast_tf（vendored）、nlohmann::json（vendored）、spdlog、gtest。

**Spec:** `docs/superpowers/specs/2026-08-14-ayanami-design.md`（本 plan argues from spec；executor 同时读 spec 与 plan 两份）。

## Global Constraints

（取自 spec 的项目级硬约束，每个 task 的要求隐式包含本节）

- 纯 C++：`include/ayanami/core/**` 与 `include/ayanami/components/**` 与 `include/ayanami/behavior_tree/**` 与 `include/ayanami/utility/**` 与 `include/ayanami/interfaces/**` 与 `include/ayanami/messages/**` 严禁 include `ros/ros.h`、`tf2_ros/*`、`sensor_msgs/*`、`nav_msgs/*` 等任何 ROS 头。
- 命名：恢复类一律 `recovery`（`AbstractRecovery`、`RecoveryAction`、`RecoveryExecution`、`costmap_*_recovery`、json `recovery_id`、BT 节点 `Recovery`），不叫 `behavior`。仅保留 BT 语境用词（`BehaviorTree.CPP`、XML 根标签 `<BehaviorTree>`、`BehaviorTreePilot`）。
- 配置目录统一 `config/`（不叫 `configs/`）；GlobalConfig 路径解析兜底兼容旧 `configs/`。
- 库类型：`libayanami` 默认 SHARED；各插件 `*.so` 由 `ayanami_add_module()` 宏生成。
- BT.CPP 子库 build 时必须加 `-Wl,-Bsymbolic`（修 vendored tinyxml2 vs 系统 tinyxml2 符号冲突）。
- dlopen 句柄从不 dlclose；析构纪律：先 reset action → `clearPlugins()` → 卸载宿主 → 句柄随进程退出。
- 所有 `geometry_msgs::TwistStamped` 类消息使用前必须 `cmd_vel = {}` 强制清零（修脏内存历史 bug）。
- 频率约定：`*_frequency == -1` 表示单次后缓存 SUCCESS；`> 0` 表示 Hz；BT RateController 首次 tick 立即执行。
- 帧名单一来源：`costmap_navigation.json::robot_frame`，由 CostmapNavigation 写到 `RobotStateProvider` 与 `CostmapContext`。
- 当前会话/版本：`session_id` / `plan_version` 用 uint64_t 链上 `Goal` 与 `Result`，BT 节点 onRunning 校验。
- 提交：本仓库 git 分支 `redesign`；commit message 风格 `<type>: <summary>`，type∈{feat/test/docs/build/refactor/fix/chore}。Plan 1 结束时一次性把所有模块加进并 commit/push。
- 编译命令：`catkin build ayanami`（M1 只构 ayanami，不构 ayanami_ros）。
- 测试命令：`ctest --test-dir <build_dir> --output-on-failure`。

---

## File Structure (本 Plan 涉及文件总览)

完整目录见 spec §1.2。本 Plan 1 涉及：

```
src/ayanami/
  CMakeLists.txt                         # 顶层 (修改, 含 ayanami_add_module 宏)
  cmake/ayanami-config.cmake.in           # 新增
  package.xml                            # 修改 (catkin dep: roscpp? 否—M1 不依赖 ros)
  docs/superpowers/plans/2026-08-14-ayanami-core-redesign.md  # 本文件
  include/ayanami/
    utility/
      config.hpp config_impl.hpp
      callback_slot.hpp
      load_module.hpp
      extension_module.hpp
      logging.hpp                          # spdlog wrapper
    interfaces/
      callbacks.hpp transform_store.hpp
      impl/callbacks.cpp                    # 槽对象静态成员定义编入 lib (关键)
    messages/
      nav_msgs/geometry_msgs/sensor_msgs/
      actions/
    core/
      abstract_planner.hpp abstract_controller.hpp
      abstract_smoother.hpp abstract_recovery.hpp
      abstract_navigation.hpp
      map_context.hpp                       # 抽象 MapContext 基类
    components/
      execution_base.hpp
      planner_execution.hpp smoother_execution.hpp
      controller_execution.hpp recovery_execution.hpp
      action_base.hpp action_server.hpp
      planner_action.hpp smoother_action.hpp
      controller_action.hpp recovery_action.hpp
      plugin_manager.hpp robot_state_provider.hpp
      navigation_server_base.hpp
    behavior_tree/
      pilot.hpp action_nodes.hpp condition_nodes.hpp
      pipeline_sequence.hpp rate_controller.hpp recovery_node.hpp
      navigation.hpp blackboard_protocol.hpp
  packages/
    costmap_2d/ (从 _legacy_*/thirdparty 迁回; namespace costmap_2d)
    costmap_layers/
    costmap_server/
  include/ayanami/costmap/
    components/costmap_context.hpp
    core/costmap_planner.hpp costmap_smoother.hpp
       costmap_controller.hpp costmap_recovery.hpp
    components/
      costmap_navigation_server.hpp
      costmap_wrapper.hpp
      costmap_planner_execution.hpp costmap_controller_execution.hpp
      costmap_recovery_execution.hpp
      costmap_planner_action.hpp costmap_controller_action.hpp
      costmap_recovery_action.hpp
  modules/
    costmap_navigation/      CostmapNavigation + module.cpp
    costmap_smac_planner/     CostmapSmacPlanner + module.cpp
    costmap_savitzky_smoother/CostmapSavitzkySmoother + module.cpp
    costmap_pid_controller/  CostmapPidController + module.cpp
    costmap_spin_recovery/   CostmapSpinRecovery + module.cpp
    costmap_back_up_recovery/CostmapBackUpRecovery + module.cpp
    costmap_wait_recovery/   CostmapWaitRecovery + module.cpp
    costmap_clear_costmap_recovery/CostmapClearCostmapRecovery + module.cpp
  config/
    config.json costmap_navigation.json costmap.json logging.json
    costmap_smac_planner.json costmap_savitzky_smoother.json
    costmap_pid_controller.json
    costmap_spin_recovery.json costmap_back_up_recovery.json
    costmap_wait_recovery.json costmap_clear_costmap_recovery.json
    BT/navigation.xml
  tests/
    config_test.cpp callback_slot_test.cpp load_module_test.cpp
    extension_module_test.cpp transform_store_test.cpp
    execution_test.cpp action_test.cpp plugin_manager_test.cpp
    navigation_server_test.cpp behavior_tree_test.cpp
    costmap_2d_test.cpp costmap_layers_test.cpp costmap_server_test.cpp
    costmap_navigation_server_test.cpp
    action_protocol_test.cpp cancel_responsiveness_test.cpp
    continuous_goal_test.cpp controller_session_test.cpp
    latest_wins_test.cpp recovery_exhaustion_test.cpp   # 复刻 legacy 6 契约
    headless_e2e_test.cpp                                # M1 终止验证
  thirdparty/  (BehaviorTree.CPP / fast_tf / nlohmann / ankerl, 复用)
```

**职责边界**：
- 一个文件一个职责；> 500 行的拆分。
- abstract 层与具体地图层在子目录隔离（`include/ayanami/core/**` 与 `include/ayanami/costmap/**`）。
- 槽对象实体定义只在 `interfaces/impl/callbacks.cpp` 一份。

---

## Tasks

### Task 0: 工程归档与骨架 + CMake 顶层

**Files:**
- Create: `src/_legacy_ayanami/`（整体移动现 `src/ayanami/{include,modules,packages,config,assets,tests,CMakeLists.txt,package.xml,.clang-format}` 到此处）
- Modify: `src/ayanami/CMakeLists.txt`（重写顶层）
- Create: `src/ayanami/cmake/ayanami-config.cmake.in`
- Modify: `src/ayanami/package.xml`
- 保留: `src/ayanami/thirdparty/**`、`src/ayanami/.git`、`src/ayanami/docs/superpowers/**`

**Interfaces:**
- Consumes: 现有 `thirdparty/BehaviorTree.CPP/`、`thirdparty/fast_tf/`、`thirdparty/nlohmann/`、`thirdparty/ankerl/`
- Produces: `ayanami` catkin 目标 + `ayanami_add_module(name src...)` 宏 + 第三方接口库别名（`BT::behaviortree_cpp`、`fast_tf::fast_tf`、`nlohmann_json::nlohmann_json`、`ankerl::unordered_dense`）

- [ ] **Step 1: 切换分支**

```bash
cd /share/nav_ws/src/ayanami
git checkout -b redesign
git status   # 现有 untracked: include modules packages config assets tests CMakeLists.txt package.xml .clang-format README.md
```

- [ ] **Step 2: 归档旧代码到 `_legacy_*`（在 git 工作树内但不再被新 CMake 扫描）**

```bash
mkdir -p /share/nav_ws/src/_legacy_ayanami
git mv include modules packages config assets tests CMakeLists.txt package.xml .clang-format README.md /share/nav_ws/src/_legacy_ayanami/ 2>/dev/null || \
  mv include modules packages config assets tests CMakeLists.txt package.xml .clang-format README.md /share/nav_ws/src/_legacy_ayanami/  # 这些是 untracked, git mv 会失败, 用 mv
# thirdparty, .git, docs 保留在 src/ayanami/
ls /share/nav_ws/src/ayanami   # 应只看到 thirdparty docs .git
```

同理 ayanami_ros：
```bash
cd /share/nav_ws/src/ayanami_ros
git checkout -b redesign
mkdir -p /share/nav_ws/src/_legacy_ayanami_ros
mv apps include launch src CMakeLists.txt package.xml .clang-format /share/nav_ws/src/_legacy_ayanami_ros/
```

- [ ] **Step 3: 重写 `src/ayanami/CMakeLists.txt` 顶层骨架**

```cmake
cmake_minimum_required(VERSION 3.16)
project(ayanami LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
option(BUILD_SHARED "Build libayanami as shared library" ON)
option(BUILD_TESTING "Build tests" ON)
option(BUILD_WITH_BT "Build behavior tree component" ON)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)  # 或 vendored

# 第三方接口库 (新增 cmake/thirdparty.cmake 引入)
include(${CMAKE_SOURCE_DIR}/cmake/thirdparty.cmake)  # 定义 BT::behaviortree_cpp / fast_tf::fast_tf / nlohmann_json / ankerl

# ayanami_add_module(name src...) 宏: 生成 SHARED 模块 .so, 链接 libayanami
add_library(ayanami ${AYANAMI_BUILD_SHARED_BOOL} src/dummy.cpp)  # 占位; 后续 task 替换为真实源
target_include_directories(ayanami PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(ayanami PUBLIC Eigen3::Eigen spdlog::spdlog)
if(BUILD_WITH_BT)
  target_link_libraries(ayanami PUBLIC BT::behaviortree_cpp fast_tf::fast_tf nlohmann_json::nlohmann_json)
endif()
install(TARGETS ayanami EXPORT ayanamiTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")

# 模块宏
function(ayanami_add_module name)
  add_library(${name} SHARED ${ARGN})
  target_link_libraries(${name} PRIVATE ayanami)
  set_target_properties(${name} PROPERTIES PREFIX "libayanami_" OUTPUT_NAME ${name})
  install(TARGETS ${name} LIBRARY DESTINATION lib)
  list(APPEND AYANAMI_INSTALL_TARGETS ${name})
  set(AYANAMI_INSTALL_TARGETS ${AYANAMI_INSTALL_TARGETS} PARENT_SCOPE)
endfunction()

# 包导出
install(EXPORT ayanamiTargets NAMESPACE ayanami:: FILE ayanamiTargets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ayanami)
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_SOURCE_DIR}/cmake/ayanami-config.cmake.in
  ${CMAKE_BINARY_DIR}/ayanami-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ayanami)
write_basic_package_version_file(ayanami-configVersion.cmake COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_BINARY_DIR}/ayanami-config.cmake ${CMAKE_BINARY_DIR}/ayanami-configVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ayanami)
```

> 设计要点：
> - `ayanami` 是 SHARED 库，依赖 Eigen、spdlog、BT.CPP v4、fast_tf、nlohmann、ankerl。
> - `ayanami_add_module` 生成 `libayanami_<name>.so`，链接 PRIVATE `ayanami`（插件只暴露 module.cpp 的 C 符号）。
> - `find_package(ayanami CONFIG)` 在下游（ayanami_ros）拿到 `ayanami::ayanami` 与 targets。

- [ ] **Step 4: 写 `cmake/ayanami-config.cmake.in`**

```cmake
@PACKAGE_INIT@
include("${CMAKE_CURRENT_LIST_DIR}/ayanamiTargets.cmake")
check_required_components(ayanami)
```

并写 `cmake/thirdparty.cmake`：定义 `BT::behaviortree_cpp`（指向 `thirdparty/BehaviorTree.CPP` 的构建产物）、`fast_tf::fast_tf`（header-only interface 到 `thirdparty/fast_tf`）、`nlohmann_json::nlohmann_json`（interface 到 `thirdparty/nlohmann`）、`ankerl::unordered_dense`（interface 到 `thirdparty/ankerl`）。BT.CPP 必须加 `-Wl,-Bsymbolic`。

- [ ] **Step 5: 修改 `package.xml`**

```xml
<package format="2">
  <name>ayanami</name>
  <version>1.0.0</version>
  <description>ayanami: pure C++ navigation framework</description>
  <maintainer email="dev@example.com">ayanami</maintainer>
  <license>MIT</license>
  <buildtool_depend>cmake</buildtool_depend>
  <depend>eigen</depend>
  <depend>spdlog</depend>
</package>
```

> M1 不依赖 roscpp/rclcpp。catkin 用作构建系统，但不 require ROS 本体。

- [ ] **Step 6: 验证空骨架可配置与构建**

```bash
cd /share/nav_ws/src/ayanami
cmake -S . -B build_m1 && cmake --build build_m1
ls build_m1/libayanami.so  # 或 .a, 取决于 BUILD_SHARED
```

Expected: `libayanami.so` 生成（虽然只有 dummy.cpp 内容）。

- [ ] **Step 7: Commit**

```bash
git add -A && git commit -m "build: archive legacy code, scaffold redesign top-level CMake"
```

`src/_legacy_*` 也加入 `src/ayanami/.gitignore`? 否——它们是 untracked 且属于上 git 仓库之外的 cache，但因为我们 mv 了文件，应在 `/share/nav_ws/src/.gitignore`（顶层非 git）或直接不 add 到本仓库。**重要**：`git add -A` 会 add `_legacy_ayanami` 内的旧文件，这会污染 commit。改为：

```bash
echo "/_legacy_ayanami/" > /share/nav_ws/src/.prune_note   # 仅在 README 说明
# 在 ayanami 仓库内：
cd /share/nav_ws/src/ayanami
git add CMakeLists.txt cmake package.xml
git commit -m "build: scaffold redesign top-level CMake"
# 旧代码留在 src/_legacy_ayanami 但不纳入 ayanami 仓库内 (它会留在/.git)
```

> `_legacy_ayanami` 位于 `/share/nav_ws/src/` 而 ayanami 仓库在 `/share/nav_ws/src/ayanami/.git`——它的 untracked 文件只能影响 `src/ayanami/`，不会污染仓库。`mv` 把内容移到 `/share/nav_ws/src/_legacy_ayanami/`，对 ayanami 仓库来说相当于"那些 untracked 文件消失了"，clean。

---

### Task 1: utility/Config + GlobalConfig

**Files:**
- Create: `include/ayanami/utility/config.hpp`
- Create: `include/ayanami/utility/config_impl.hpp`
- Create: `src/utility/config.cpp`（含 GlobalConfig 单例 + Config::param 模板显式实例化 + traits）
- Create: `tests/config_test.cpp`

**Interfaces:**
- Consumes: `nlohmann::json`（vendored），`spdlog`，`Eigen`（traits 适配）
- Produces:
  - `class Config { Config(const std::string& filename); template<typename T> T param(module, name, default); template<typename T> T param_cast<T>(module, name); template<typename T> void override_param(module, name, val); }`
  - `class GlobalConfig : public Config { static GlobalConfig* instance(const std::string& config_path="", bool override_path=false); static std::string get_config_path(const std::string& config_name); }`
  - 用 `std::any` 隐藏 json；公开头不 include `nlohmann/json.hpp`

- [ ] **Step 1: 写 failing test `tests/config_test.cpp`**

```cpp
#include <gtest/gtest.h>
#include <ayanami/utility/config.hpp>
#include <Eigen/Dense>
TEST(ConfigTest, LoadAndParamScalar) {
  ayanami::Config cfg(/* path to a temp json: { "m": { "k": 42, "v": "hello" } } */);
  EXPECT_EQ(cfg.param<int>("m", "k", 0), 42);
  EXPECT_EQ(cfg.param<std::string>("m", "v", ""), "hello");
  EXPECT_EQ(cfg.param<int>("m", "missing", 7), 7);  // default
}
TEST(ConfigTest, EigenTrait) {
  ayanami::Config cfg(/* json: { "p": { "t": [1,2,3,0,0,0,1] } } (TUM 7 元) */);
  auto t = cfg.param<Eigen::Isometry3d>("p", "t", Eigen::Isometry3d::Identity());
  EXPECT_NEAR(t.translation().x(), 1.0, 1e-6);
}
TEST(GlobalConfigTest, InstanceOnce) {
  ayanami::GlobalConfig* a = ayanami::GlobalConfig::instance(/* temp dir containing config.json */);
  ayanami::GlobalConfig* b = ayanami::GlobalConfig::instance();
  EXPECT_EQ(a, b);
  EXPECT_FALSE(ayanami::GlobalConfig::get_config_path("config_xyz").empty());
}
TEST(GlobalConfigTest, PathResolution) {
  // config_path 是文件 -> 取父目录; 是目录 -> 目录或目录/configs/
  ayanami::GlobalConfig* g = ayanami::GlobalConfig::instance(/* a path to config.json file */, true);
  // 检查 override_param("global","config_path",...) 生效
}
```

- [ ] **Step 2: 验证 fail**

```bash
cd /share/nav_ws/src/ayanami
cmake -S . -B build_m1 -DBUILD_TESTING=ON && cmake --build build_m1 --target config_test
ctest --test-dir build_m1 -R ConfigTest --output-on-failure
```

Expected: 编译失败（`ayanami/config.hpp` 不存在）。

- [ ] **Step 3: 实现 `config.hpp` / `config_impl.hpp` / `src/utility/config.cpp`**

按 spec §2.1 实现。关键点：
- `Config` 内部 `std::any config_` 持有 nlohmann::json。
- 构造失败（文件打不开）`spdlog::error` 但 config 仍设为空 json（不 abort），让 `param` 返回 default。
- `override_param` 在内存 JSON 中 set 值（不保存到磁盘）。
- `traits<T>` 支持 `int/long/double/bool/std::string/Eigen::Vector2d/3d/4d/Eigen::Quaterniond/Eigen::Isometry3d/std::vector<Eigen::Isometry3d>`。Isometry3d 走 TUM `[x,y,z,qx,qy,qz,qw]`。
- JSON 解析 `nlohmann::json::parse(ifs, nullptr, true, true)`（ignore_comments=true）。
- `GlobalConfig::instance`: 用 `std::once_flag + std::call_once`；
- `instance(config_path, override_path=true)` 重建（删旧实例）；`override_path=false` 且已有实例时直接返回。
- `get_config_path(name)` = `param("global","config_path",".") + "/" + param("global", name, name+".json")`。
- 路径解析按 spec §2.2：文件→父目录；目录→查 `config.json` 或 `config/config.json` 或 `configs/config.json`。

- [ ] **Step 4: 验证 pass**

```bash
cmake --build build_m1 --target config_test && ctest --test-dir build_m1 -R ConfigTest --output-on-failure
```

Expected: 4 个用例全 PASS。

- [ ] **Step 5: 集成到 ayanami 库**

更新 `CMakeLists.txt`：把 `src/utility/config.cpp` 加入 ayanami；`BUILD_TESTING=ON` 时构建 `config_test` 并链 gtest。

- [ ] **Step 6: Commit**

```bash
git add include/ayanami/utility/config.hpp include/ayanami/utility/config_impl.hpp src/utility/config.cpp tests/config_test.cpp CMakeLists.txt
git commit -m "feat: utility Config + GlobalConfig (once_flag 线程安全, 路径二义解析)"
```

---

### Task 2: utility/CallbackSlot

**Files:**
- Create: `include/ayanami/utility/callback_slot.hpp`
- Create: `tests/callback_slot_test.cpp`

**Interfaces:**
- Consumes: `<functional> <shared_mutex> <vector>`
- Produces: `template<typename Func> class CallbackSlot { int add(cb); void remove(int); void call(Args...) const; void operator()(Args...) const; explicit operator bool() const; }`

- [ ] **Step 1: 写 failing test**

```cpp
TEST(CallbackSlotTest, AddCallRemove) {
  ayanami::CallbackSlot<void(int)> slot;
  int sum = 0;
  int id1 = slot.add([&](int v){ sum += v; });
  int id2 = slot.add([&](int v){ sum += v*10; });
  slot(5);  // sum = 5 + 50 = 55
  EXPECT_EQ(sum, 55);
  slot.remove(id1);
  slot(5);  // sum = 55 + 50 = 105
  EXPECT_EQ(sum, 105);
}
TEST(CallbackSlotTest, ThreadSafe) {
  ayanami::CallbackSlot<void(int)> slot;
  std::atomic<int> hits{0};
  slot.add([&](int){ hits++; });
  std::vector<std::thread> ts;
  for (int i=0;i<8;++i) ts.emplace_back([&]{ for (int j=0;j<1000;++j) slot(j); });
  for (auto& t: ts) t.join();
  EXPECT_EQ(hits.load(), 8000);
}
TEST(CallbackSlotTest, EmptySlotIsFalse) {
  ayanami::CallbackSlot<void()> slot;
  EXPECT_FALSE(bool(slot));
  slot.add([]{});
  EXPECT_TRUE(bool(slot));
}
```

- [ ] **Step 2: 验证 fail**

```bash
cmake --build build_m1 --target callback_slot_test && ctest --test-dir build_m1 -R CallbackSlotTest
```

- [ ] **Step 3: 实现 `callback_slot.hpp`**

按 spec §3.1：vector<std::function> + shared_mutex；add 返回 vector size-1 作 id；remove[id]=nullptr；call/operator() 遍历跳过 nullptr；用 `std::shared_lock` for read（call），`std::unique_lock` for write（add/remove）。

- [ ] **Step 4: 验证 pass**

- [ ] **Step 5: Commit**

```bash
git add include/ayanami/utility/callback_slot.hpp tests/callback_slot_test.cpp CMakeLists.txt
git commit -m "feat: utility CallbackSlot (shared_mutex, 跨 .so 可靠)"
```

---

### Task 3: utility/load_module + utility/extension_module

**Files:**
- Create: `include/ayanami/utility/load_module.hpp`
- Create: `src/utility/load_module.cpp`
- Create: `include/ayanami/utility/extension_module.hpp`
- Create: `src/utility/extension_module.cpp`
- Create: `tests/load_module_test.cpp`
- Create: `tests/plugins/dummy_planner_module.cpp`（被 dlopen 的 .so）

**Interfaces:**
- Consumes: `<dlfcn.h>`
- Produces:
  - `void* open_so(const std::string& so_name);`
  - `void* load_symbol(void* h, const std::string& sym);`
  - `template<typename T> std::shared_ptr<T> load_module_from_so(so_name, sym);`
  - `class ExtensionModule { public: using Ptr = std::shared_ptr<ExtensionModule>; static Ptr load_module(const std::string& so_name); virtual ~ExtensionModule()=default; };`

- [ ] **Step 1: 写 dummy module**

```cpp
// tests/plugins/dummy_planner_module.cpp
extern "C" int create_dummy_module() { return 42; }
```

- [ ] **Step 2: 写 failing test**

```cpp
TEST(LoadModuleTest, OpenAndCallSymbol) {
  auto factory = ayanami::load_module_from_so<int(*)()>(
    /* path to libdummy_planner_module.so */ , "create_dummy_module");
  ASSERT_NE(factory, nullptr);
  EXPECT_EQ(factory(), 42);
}
TEST(LoadModuleTest, MissingReturnsNull) {
  auto x = ayanami::load_module_from_so<int(*)()>("libnonexistent_xyz.so", "create_xxx");
  EXPECT_EQ(x, nullptr);
}
TEST(ExtensionModuleTest, LoadViaCreateExtensionModule) {
  // tests/plugins/dummy_extension_module.cpp: extern "C" ayanami::ExtensionModule* create_extension_module() { return new DummyExtension; }
  auto ext = ayanami::ExtensionModule::load_module(/* so path */);
  ASSERT_NE(ext, nullptr);
}
```

- [ ] **Step 3: 实现 `load_module.cpp`**：`dlopen(RTLD_LAZY|RTLD_GLOBAL)`、`dlsym`，失败 `spdlog::error` 返回 nullptr；句柄**不 dlclose**（spec §4.1）。
- [ ] **Step 4: 实现 `extension_module.cpp`**：`ExtensionModule::load_module(so)` 调用 `load_module_from_so<ExtensionModule*(*)()>(so, "create_extension_module")` 返回 shared_ptr（不带自定义 deleter；与进程同寿命）。

- [ ] **Step 5: 验证 pass + Commit**

```bash
git add include/ayanami/utility/load_module.hpp src/utility/load_module.cpp \
  include/ayanami/utility/extension_module.hpp src/utility/extension_module.cpp \
  tests/load_module_test.cpp tests/plugins/ CMakeLists.txt
git commit -m "feat: utility load_module + ExtensionModule (dlopen, 句柄不 dlclose)"
```

---

### Task 4: messages (Eigen 自研消息)

**Files:**
- Create: `include/ayanami/messages/geometry_msgs.hpp`（`PoseStamped`、`PointStamped`、`TwistStamped`、`Quaternion`、`Pose`、`Vector3` 全用 Eigen）
- Create: `include/ayanami/messages/nav_msgs.hpp`（`Path`、`OccupancyGrid`、`Odometry`、`MapMetaData`）
- Create: `include/ayanami/messages/sensor_msgs.hpp`（`PointCloud2`、`LaserScan`、`PointField`）
- Create: `include/ayanami/messages/actions.hpp`（`GetPathGoal/Result/Feedback`、`SmoothPathGoal/Result/Feedback`、`ExePathGoal/Result/Feedback`、`RecoveryGoal/Result/Feedback`，含 `outcome` 错误码常量、`session_id`/`plan_version`）
- Create: `tests/messages_test.cpp`

**Interfaces:**
- Consumes: `Eigen`
- Produces: 上述类型 + 每类 `using Ptr = std::shared_ptr<...>`

- [ ] **Step 1: 写 failing test**：检查 `PoseStamped` 有 `header.frame_id`、`pose.translation().x() = 1.0`；`Path.poses` 是 `vector<PoseStamped>`；`RecoveryResult::SUCCESS == 0`、`RecoveryResult::INTERNAL_ERROR >= 300`；`ExePathGoal` 有 `path`、`session_id`、`plan_version`。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现各 hpp**：消息结构用 Eigen，不 include ROS。错误码常量见 spec §5.1。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: messages Eigen self-contained types`。

---

### Task 5: interfaces/TransformStore + FastTF

**Files:**
- Create: `include/ayanami/interfaces/transform_store.hpp`
- Create: `tests/transform_store_test.cpp`

**Interfaces:**
- Consumes: `thirdparty/fast_tf/fast_tf.hpp`
- Produces:
  - `class TransformStore { static TransformStore& instance(); void setTransform(parent, child, stamp, T, is_static=false); bool lookupTransform(target, source, stamp, out, timeout=0.0) const; static TimePoint fromSeconds(double); }`
  - 内部持 `fast_tf::detail::transform_buffer`

- [ ] **Step 1: 写 failing test**：`setTransform("map","odom", fromSeconds(0), Isometry3d(Translation(1,0,0)*Quaternion::Identity()))` 后 `lookupTransform("map","odom", fromSeconds(0), out, 0)` 成功且 `out.translation().x() == 1.0`；不存在边 lookup 返回 false。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现 `transform_store.hpp`**：单例包装 fast_tf，按 spec §8；用 `std::chrono::time_point` 作 TimePoint。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: interfaces TransformStore (fast_tf wrapper)`。

---

### Task 6: interfaces/NavigationCallbacks (静态聚合对象定义在 lib.cpp 内)

**Files:**
- Create: `include/ayanami/interfaces/callbacks.hpp`
- Create: `src/interfaces/callbacks.cpp`（实体定义编入 libayanami）
- Create: `tests/callbacks_test.cpp`

**Interfaces:**
- Consumes: `utility/callback_slot.hpp`、`messages/actions.hpp`
- Produces: `struct NavigationCallbacks { static CallbackSlot<...> on_get_path_goal; ... on_recovery_result; ... on_robot_state_updated; ... on_velocity_command; ... on_costmap_visualization; ... on_obstacle_pointcloud; ... on_obstacle_laserscan; }`（共 16 个槽，spec §3.3）

- [ ] **Step 1: 写 failing test**

```cpp
TEST(NavigationCallbacksTest, AddAndCallAcrossCompilationUnits) {
  // 在 tests 内 add 回调到 NavigationCallbacks::on_get_path_result
  // 跨编译单元验证槽对象一致 (callbacks.cpp 定义在 lib, test 使用 include)
  int received = 0;
  NavigationCallbacks::on_get_path_result.add([&](const auto& r){ received++; });
  // 在测试内 emit 槽
  NavigationCallbacks::on_get_path_result(/* fake GoalResult<GetPathResult> */);
  EXPECT_EQ(received, 1);
}
TEST(NavigationCallbacksTest, ThreadSafeAcrossAddRemoveCall) {
  // 同 Task 2 的并发测试, 但挂在 NavigationCallbacks 上
}
```

- [ ] **Step 2: 验证 fail**。

- [ ] **Step 3: 实现 `callbacks.hpp`**：声明 `static CallbackSlot<Func> on_xxx;`（**不**用 inline static）。实现 `callbacks.cpp`：每个静态成员显式定义 `CallbackSlot<...> NavigationCallbacks::on_xxx;`。编入 libayanami——这是跨 .so 可靠性的根本（spec §3.2）。

```cpp
// callbacks.hpp 声明
struct NavigationCallbacks {
  static CallbackSlot<void(const GetPathGoal&)>      on_get_path_goal;
  static CallbackSlot<void(const GoalResult<GetPathResult>&)> on_get_path_result;
  // ... 其余 14 个
};
// callbacks.cpp 定义 (只在 libayanami 内一份)
CallbackSlot<void(const GetPathGoal&)> NavigationCallbacks::on_get_path_goal;
CallbackSlot<void(const GoalResult<GetPathResult>&)> NavigationCallbacks::on_get_path_result;
// ...
```

- [ ] **Step 4: 验证 pass**（含 dlopen 一个测试 .so add 回调验证跨 .so 可见，对应 §3.2 关键纪律）：
  - 额外加一个 `tests/plugins/echo_extension.cpp` 也 add 到 `on_get_path_result`，dlopen 后再触发 callback，预期 echo extension 内的计数也增加。

- [ ] **Step 5: Commit**

```bash
git add include/ayanami/interfaces/callbacks.hpp src/interfaces/callbacks.cpp \
  tests/callbacks_test.cpp tests/plugins/echo_extension.cpp CMakeLists.txt
git commit -m "feat: interfaces NavigationCallbacks (槽对象实体在 lib, 跨 .so 可靠)"
```

---

### Task 7: core 抽象接口 + AbstractMapContext

**Files:**
- Create: `include/ayanami/core/abstract_planner.hpp`
- Create: `include/ayanami/core/abstract_controller.hpp`
- Create: `include/ayanami/core/abstract_smoother.hpp`
- Create: `include/ayanami/core/abstract_recovery.hpp`
- Create: `include/ayanami/core/abstract_navigation.hpp`
- Create: `include/ayanami/core/map_context.hpp`
- Create: `tests/core_interfaces_test.cpp`

**Interfaces:**
- Consumes: `messages/**`
- Produces: spec §5.1 + §5.2 中列出的全部接口（按签名 verbatim）

- [ ] **Step 1: 写 failing test**：定义一个 TestPlanner : public AbstractPlanner，验证可实例化且 `makePlan` 可被调用、cancel 默认返回 false；同理 TestController/TestSmoother/TestRecovery/TestNavigation；以及 TestMapContext : public AbstractMapContext 验证默认 `isCurrent()==true`、`lock/unlock` 空实现。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现各 hpp**：纯虚接口（spec §5.1）+ `using Ptr = std::shared_ptr<...>` 惯用法。`MapContext` 提供默认空 `lock/unlock/isCurrent`。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: core abstract plugin interfaces + AbstractMapContext`。

---

### Task 8: components/ExecutionBase + 4 *_execution

**Files:**
- Create: `include/ayanami/components/execution_base.hpp`
- Create: `include/ayanami/components/planner_execution.hpp`
- Create: `include/ayanami/components/smoother_execution.hpp`
- Create: `include/ayanami/components/controller_execution.hpp`
- Create: `include/ayanami/components/recovery_execution.hpp`
- Create: `include/ayanami/components/impl/execution_base.cpp`（内含线程管理、condition_variable）
- Create: `tests/execution_test.cpp`

**Interfaces:**
- Consumes: `core/abstract_*`
- Produces:
  - `class ExecutionBase { void start(); void stop(); void cancel(); void join(); bool waitForStateUpdate(timeout); State getState(); uint32_t getOutcome(); }`
  - `PlannerExecution`，`SmootherExecution`，`ControllerExecution`，`RecoveryExecution` 各带独立状态枚举（spec §5 与参考 MBF §2.2）

- [ ] **Step 1: 写 failing test**：用 TestPlanner 起一个 PlannerExecution，`start(goal)` 后 poll `waitForStateUpdate(100ms)` 直至 `FOUND_PLAN`，验 `getOutcome()==0`；用 TestController 起 ControllerExecution，验证按 controller_frequency 循环调 `computeVelocityCommands`，`isGoalReached` 时 ARRIVED_GOAL 退出循环；RecoveryExecution 调一次 `runRecovery` 状态进 RECOVERY_DONE；所有 cmd_vel/TwistStamped 发布前先 `cmd_vel = {}` 强制清零（spec Global Constraints）。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现**：
  - `ExecutionBase`：`std::thread`、`std::condition_variable condition_`、`std::mutex mutex_`、`should_exit_/cancel_`、`start()` 开线程跑虚 `run()`、`waitForStateUpdate(dur)` wait on cv。
  - `PlannerExecution::run()`：while 循环 `makePlan`，FOUND_PLAN 退出；支持 `max_retries_/patience`（参考 MBF）。
  - `ControllerExecution::run()`：while 循环 `computeVelocityCommands` + `isGoalReached`；按 controller_frequency sleep。
  - `RecoveryExecution::run()`：单次 `runRecovery` + 状态机。
  - `SmootherExecution::run()`：单次 `smoothPath` + 状态机。
  - `virtual void preRun(){}` / `virtual void postRun(){}` / `virtual bool safetyCheck(){ return true; }` 钩子（map 层覆写）。
  - `virtual void publishStopCommand()`：所有 `TwistStamped cmd = {}` 全字段清零。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: components ExecutionBase + 4 executions`。

---

### Task 9: components/ActionBase + ActionServer + 4 *_action

**Files:**
- Create: `include/ayanami/components/action_base.hpp`
- Create: `include/ayanami/components/action_server.hpp`
- Create: `include/ayanami/components/planner_action.hpp`
- Create: `include/ayanami/components/smoother_action.hpp`
- Create: `include/ayanami/components/controller_action.hpp`
- Create: `include/ayanami/components/recovery_action.hpp`
- Create: `tests/action_test.cpp`

**Interfaces:**
- Consumes: `components/execution_base + *_execution`、`messages/actions.hpp`、`interfaces/callbacks.hpp`
- Produces:
  - `template<typename Goal, typename Execution> class ActionBase { ConcurrencySlot 结构; void start(goal_handle, execution, slot=0); void cancel(slot); void cancelAll(); TaskHandle acceptGoal(goal) -> TaskHandle; }`
  - `ActionServer<Goal, Result, Feedback>`（线程安全状态机 Pending→Accepted→Executing→Succeeded/Aborted/Canceled）
  - `PlannerAction`、`SmootherAction`、`ControllerAction`、`RecoveryAction` 模板特化
  - `class TaskHandle { bool ready() const; Result result() const; void onComplete(cb); void cancel(); }`

- [ ] **Step 1: 写 failing test**：
  - 创建 PlannerAction，`acceptGoal(GetPathGoal{...})` 拿 TaskHandle；在另一线程 await `task.ready()` 阻塞直到 result 到；验证 status==SUCCEEDED、result.path 非空。
  - `cancel(slot=0)` 让 RUNNING exec 中断，TaskHandle 收到 CANCELED。
  - 同名 goal 抢占：再次 `acceptGoal` 同槽位时旧 goal 先 preempt → join → 新 goal 启动（参考 MBF 抢占）。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现**：
  - `ActionServer` 内部 `std::map<uint64_t, GoalState>`，goal_id 自增。
  - `ActionBase::start(goal_handle, execution, slot)`：同槽已有则 `preempt()`+`join()` 旧线程再起新线程。
  - `run()` 调 `execution.preRun() → runImpl(goal_handle, execution) → execution.join() → postRun()`。
  - `acceptGoal(goal)` 返回 `TaskHandle`（含 future/promise），完成时通过 future set value。
  - `runImpl` 在子类里 poll `execution.waitForStateUpdate(50ms)` + state 分派（参考 MBF §2.4）。
  - 关键：**Action 触发 `NavigationCallbacks::on_*_result` 作旁路**（spec §3.4），但 TaskHandle 直连给调用者（BT 节点）。
- [ ] **Step 4: 验证 pass**（含复刻 legacy 的 `action_protocol_test`）。
- [ ] **Step 5: Commit** `feat: components ActionBase + ActionServer + 4 actions + TaskHandle 旁路 callback`。

---

### Task 10: components/PluginManager + RobotStateProvider

**Files:**
- Create: `include/ayanami/components/plugin_manager.hpp`
- Create: `include/ayanami/components/robot_state_provider.hpp`
- Create: `include/ayanami/components/impl/plugin_manager.cpp`（dlopen 加载）
- Create: `tests/plugin_manager_test.cpp`

**Interfaces:**
- Consumes: `core/abstract_*`、`utility/load_module`、`utility/config`
- Produces: spec §4.4 + `class RobotStateProvider { void setFrames(global, odom, robot); void setRobotState(PoseStamped, TwistStamped); PoseStamped getRobotPose(); TwistStamped getRobotVelocity(); }`

- [ ] **Step 1: 写 failing test**：
  - PluginManager<AbstractPlanner> 加载一个 dummy planner .so（Task 3 的 dummy_planner_module）+ init_fn（验证被调用）；`hasPlugin/getPlugin/getLoadedNames` 工作。
  - RobotStateProvider 设帧名后 setRobotState/getRobotPose 一致；未 set 时 getRobotPose 返回 identity。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现**：
  - `PluginManager::loadPlugin(spec, init_fn)` dlopen + dlsym 返回 Base*，调 init_fn；success 入 map。
  - `loadSingleFromConfig(cfg, module_name, default_name, init_fn)`：读 `name/so_name/create_symbol`。
  - `clearPlugins()` 释放所有 → 析构纪律（spec §4.5）。
  - `RobotStateProvider` 用 `std::mutex` 保护 pose/velocity 缓存。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: components PluginManager + RobotStateProvider`。

---

### Task 11: components/NavigationServerBase

**Files:**
- Create: `include/ayanami/components/navigation_server_base.hpp`
- Create: `src/components/navigation_server_base.cpp`
- Create: `src/components/impl/navigation_server_base_impl.cpp`（核心 implementation）
- Create: `tests/navigation_server_test.cpp`

**Interfaces:**
- Consumes: `components/plugin_manager + *_action + *_execution + robot_state_provider + core/map_context + interfaces/callbacks + utility/config`
- Produces: spec §5.3 中 `NavigationServerBase` 全部公共与虚接口；含 `requestPath/requestSmoothing/requestExecution/requestRecovery` 返回 TaskHandle、`registerPlannerFallback` 自动触发默认 recovery action。

- [ ] **Step 1: 写 failing test**：
  - 实例化 NavServer（用 Test* plugin + TestMapContext），`loadPlugins(temp config_dir)` 成功；调 `requestPath(GetPathGoal{...})` 拿 TaskHandle，wait 直到 ready，验 result 正常。
  - `registerPlannerFallback`：让 TestPlanner 故意返回失败 outcome，`on_get_path_result` 旁路触发后，`recovery_action_` 收到 goal（验证旁路链）。
  - `requestExecution/controller_frequency` 循环；`cancelAll` 杀掉所有 RUNNING action。
  - `sessionIsCurrent/planVersionIsCurrent` 生效。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现 `navigation_server_base.hpp/cpp`**：
  - 持 4 个 PluginManager + 4 个 Action + AbstractMapContext。
  - **`virtual initialize*Plugin(name, ptr)`（默认空实现 return true）** → 派生覆写注入地图（spec §5.3 关键）。
  - **`virtual new*Execution` 工厂**（默认返回 abstract 版本）。
  - `loadPlugins(config_path)`：读 `config.json` 的 global 段，逐子配置 dlopen + 调虚 `initialize*Plugin`；成功后 `createActions()`。
  - `requestPath(g)`：从 planner_action_ 调 acceptGoal 返回 TaskHandle；goal 带 `session_id_/plan_version_`。
  - `registerPlannerFallback()`：add `NavigationCallbacks::on_get_path_result` callback，失败时调 default recovery_action 的 acceptGoal（spec §3.4 旁路用途）。
  - 析构：按 spec §4.5 顺序 reset action → clearPlugins → 卸载宿主。
- [ ] **Step 4: 验证 pass**（包括 6 个 legacy 契约中 `navigation_test` 的等价内容）。
- [ ] **Step 5: Commit** `feat: components NavigationServerBase (virtual initialize*Plugin + AbstractMapContext + 旁路 fallback)`。

---

### Task 12: behavior_tree/blackboard + 控制节点 + 条件节点

**Files:**
- Create: `include/ayanami/behavior_tree/blackboard_protocol.hpp`（端口名常量 + validateRequiredBlackboardEntries）
- Create: `include/ayanami/behavior_tree/pipeline_sequence.hpp`
- Create: `include/ayanami/behavior_tree/rate_controller.hpp`
- Create: `include/ayanami/behavior_tree/recovery_node.hpp`
- Create: `include/ayanami/behavior_tree/condition_nodes.hpp`（IsPathValid、IsGoalReached、_goalUpdated、IsRobotStuck、IsFarFromGoal）
- Create: `tests/behavior_tree_control_test.cpp`

**Interfaces:**
- Consumes: BT.CPP v4（vendored）
- Produces: BT.CPP 自定义 node 类型，注册名 `PipelineSequence`、`RateController`、`RecoveryNode`、`GoalUpdated`、`IsPathValid` 等

- [ ] **Step 1: 写 failing test**：
  - `RecoveryNode retries=2`：child0 必失败 3 次后整体 FAILURE；child1 必失败立即 FAILURE。
  - `RateController hz=10`：发 100Hz tick 100ms 预期 child 被执行 1 次；hz=-1 单次后缓存 SUCCESS。
  - `PipelineSequence`：child0 RUNNING 时下次 tick 依然执行 child0；child0 SUCCESS 后下次 tick 从 child0 开始转 child1 继续；任一 child FAILURE → 整树 FAILURE。
  - `IsPathValid cmd=空 path → FAILURE`；`GoalUpdated` 在黑板 goal 变化时返回 SUCCESS。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现**：
  - `PipelineSequence`、`RateController`、`RecoveryNode` 对齐 nav2（参考 nav2_behavior_tree §2.4），但用 BT.CPP v4 API（`halt()`、`executeStatus()`、`providedPorts()`）。
  - `IsRobotStuck` 用 **黑板令牌 reset_stuck_token 复位**（避免 static 状态泄漏，spec §6.5）。
  - `blackboard_protocol.hpp` 集中常量：`kGoal`、`kRawPath`、`kSmoothedPath`、`kPlannerFrequency`、`kControllerFrequency`、`kRecoveryId`、`kClearCostmapId`、`kSpinId`、`kWaitId`、`kBackupId`、`kSessionId`、`kPlanVersion`、`kResetStuckToken`。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: behavior_tree control+condition nodes (nav2 风格 + 板令牌复位)`。

---

### Task 13: behavior_tree/action_nodes (TaskHandle 直连) + 注册 + 默认 XML

**Files:**
- Create: `include/ayanami/behavior_tree/action_nodes.hpp`（ComputePathToPoseNode、SmoothPathNode、FollowPathNode、RecoveryActionNode）
- Create: `include/ayanami/behavior_tree/navigation.hpp`（`registerNavigationBehaviorTreeNodes(BT::BehaviorTreeFactory&)` + 内置默认 XML 字符串）
- Create: `config/BT/navigation.xml`（与字符串同构，文件可覆盖）
- Create: `tests/behavior_tree_action_test.cpp`

**Interfaces:**
- Consumes: `components/navigation_server_base`、`behavior_tree/blackboard_protocol`、BT.CPP v4
- Produces: BT.CPP `StatefulActionNode` 子类，节点名 `ComputePathToPose`、`SmoothPath`、`FollowPath`、`Recovery`

- [ ] **Step 1: 写 failing test**：
  - 用 MockNavigationServer（实现 `requestPath/...` 返回固定 TaskHandle）+ BT.CPP 树跑完 `PipelineSequence`，验证 onCompleted 时 `raw_path/smoothed_path` 写入黑板、`plan_version` 自增。
  - 故意让 `requestPath` 返回 session_id 不匹配的 result：验证 `onRunning` 拒收并返回 FAILURE（spec §6.2 关键纪律）。
  - `onHalted` 触发时 `task.cancel()` 被调用。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现 `action_nodes.hpp`**：每节点持 `NavigationServerBase* nav_server_`（构造期注入）；`onStart` 组 Goal 调 request*，注册 `task.onComplete(emitWakeUpSignal)`；`onRunning` 校验 session/version 后写端口；`onHalted` cancel task。代码骨架见 spec §6.2。
- [ ] **Step 4: 实现 `navigation.hpp`** + `config/BT/navigation.xml`：默认 XML 按 spec §6.3 提供。XML 与内嵌字符串同构；`loadFromFile` 后 `validateRequiredBlackboardEntries` 必须校验通过。
- [ ] **Step 5: 验证 pass**（注册树 + 加载 XML + 黑板校验 + 简单 tick 路径）。
- [ ] **Step 6: Commit** `feat: behavior_tree action_nodes (TaskHandle 直连 + session/version 校验 + 默认 XML)`。

---

### Task 14: behavior_tree/Pilot (常驻树 + Mock 集成)

**Files:**
- Create: `include/ayanami/behavior_tree/pilot.hpp`
- Create: `src/behavior_tree/pilot.cpp`
- Create: `tests/pilot_test.cpp`

**Interfaces:**
- Consumes: `components/navigation_server_base`、`behavior_tree/navigation`、`behavior_tree/blackboard_protocol`、BT.CPP v4
- Produces: `class BehaviorTreePilot { void configure(config_path); void setNavigationServer(NavigationServerBase*); void loadFromFile(xml_path, tree_id); void beginNavigation(PoseStamped goal); void run(); void halt(); void stop(); }`

- [ ] **Step 1: 写 failing test**：
  - MockNavigationServer 给定成功路径与 cmd_vel；`BehaviorTreePilot::configure(tmp/config)` + `loadFromFile(default xml)` + `beginNavigation(goal)` + 启 `run()` 线程。
  - 等待 100ms 后黑板 raw_path/smoothed_path 应非空、`session_id_` 增加、`plan_version_` 改变、`on_velocity_command` 旁路被触发。
  - **新 goal 抢占**：再 `beginNavigation(new_goal)`，验证旧 task 被 cancel、cmd_vel 旁路先发零速度、`tree_` 不重建（同一 tree 实例）、`session_id_` 再次自增。
  - `halt()` 触发后所有 RUNNING action 被取消。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现 `pilot.cpp`**：
  - 持 `BT::Tree tree_; BT::Blackboard::Ptr blackboard_; NavigationServerBase* nav_server_;`
  - **树常驻不重建**（spec §6.1）；`beginNavigation` 只 set pending_goal_ + emitWakeUpSignal。
  - `run()` 线程：`WallRate(tick_frequency_=100Hz)` + `tree_.tickOnce()` + `sleep` 可被 `emitWakeUpSignal` 唤醒；每 tick 前 `applyPendingGoal` 重写黑板 goal、自增 session_id、reset `reset_stuck_token`、清零 plan_version。
  - 新 goal 抢占流程（spec §6.1）：通过 `nav_server_->controllerAction()->cancelAll()`（旁路触发零速度）+ `tree_.halt()` + join 旧 navigation_thread_，再 beginNavigation。
  - `configure(config_path)` 读 `costmap_navigation.json` 的 `navigation` 段（xml/id/频率/ids），调 `loadFromFile` 与 `setNavigationServer`。
- [ ] **Step 3.1: 写默认 XML 通过 `navigation.hpp` 的内嵌字符串**（Task 13 已写）。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: behavior_tree BehaviorTreePilot (常驻树 + pending_goal 黑板 + 抢占发零速度)`。

---

### Task 15: packages/costmap_2d (纯 C++ 子库)

**Files:**
- Create: `packages/costmap_2d/include/costmap_2d/cost_values.hpp`
- Create: `packages/costmap_2d/include/costmap_2d/costmap_2d.hpp`
- Create: `packages/costmap_2d/include/costmap_2d/layer.hpp`
- Create: `packages/costmap_2d/include/costmap_2d/layered_costmap.hpp`
- Create: `packages/costmap_2d/CMakeLists.txt`（生成 `costmap_2d::costmap_2d` interface 或 shared）
- Create: `tests/costmap_2d_test.cpp`

**Interfaces:**
- Consumes: `Eigen`、`<vector>`、`thirdparty` 中已有
- Produces: `costmap_2d::Costmap2D`、`costmap_2d::Layer`、`costmap_2d::LayeredCostmap`、`costmap_2d::FREE_SPACE/INSCRIBED_INFLATED_OBSTACLE/LETHAL_OBSTACLE/NO_INFORMATION`

- [ ] **Step 1: 写 failing test**：
  - `Costmap2D.getCost/setCost`、`worldToMap/mapToWorld`、`resizeMap` 后 `size_x_*size_y_*resolution_*origin_*default_value` 落到成员（spec Status 修复历史遮蔽 bug）。
  - `LayeredCostmap` 添加自定义 TestLayer，`updateBounds/updateCosts` 被调用且 `getCharMap()` 反映更新。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现**：从 `_legacy_*/thirdparty/costmap_2d/` 提升 → `packages/costmap_2d/`，统一 namespace `costmap_2d`；**修复历史 `resizeMap` 参数遮蔽 bug**（`this->size_x = size_x` 等显式 this）。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: packages/costmap_2d (pure C++, namespace costmap_2d, 修 resizeMap 遮蔽)`。

---

### Task 16: packages/costmap_layers

**Files:**
- Create: `packages/costmap_layers/include/costmap_2d_layers/static_layer.hpp/.cpp`
- Create: `packages/costmap_layers/include/costmap_2d_layers/inflation_layer.hpp/.cpp`
- Create: `packages/costmap_layers/include/costmap_2d_layers/simple_obstacle_layer.hpp/.cpp`
- Create: `packages/costmap_layers/CMakeLists.txt`
- Create: `tests/costmap_layers_test.cpp`

**Interfaces:**
- Consumes: `costmap_2d::Layer`、`CostmapContext::Ptr`（后续 task 引入；本 task 用 Costmap2D*)
- Produces: 三个具体 `Layer` 子类

- [ ] **Step 1: 写 failing test**：
  - StaticLayer 加载 nav.pgm → `updateBounds` 覆盖全图、`updateCosts` 写入 `FREE/LETHAL/NO_INFORMATION`。
  - SimpleObstacleLayer：mark 一些 obstacles → `updateCosts` 在落点置 lethal。
  - InflationLayer：给定 inflation_radius，obstacle cells 周围 `INSCRIBED_INFLATED_OBSTACLE` 衰减代价（验证嵌套 inflation_radius 与 resolution clamp）。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现**：从 `_legacy_*/{costmap_layers, include/ayanami/costmap/layers}` 提升；逻辑对齐 navigation1 costmap_2d layers 但纯 C++（去 ROS topic publisher、去 dynamic_reconfigure）。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: packages/costmap_layers (Static/Inflation/SimpleObstacle, pure C++)`。

---

### Task 17: packages/costmap_server (CostmapWrapper + lifetime)

**Files:**
- Create: `packages/costmap_server/include/costmap_server/costmap_wrapper.hpp`
- Create: `packages/costmap_server/CMakeLists.txt`
- Create: `tests/costmap_server_test.cpp`

**Interfaces:**
- Consumes: `costmap_2d::LayeredCostmap`
- Produces: `class CostmapWrapper { bool checkActivate(); bool checkDeactivate(); bool isCurrent(); std::recursive_mutex& mutex(); void updateMap(x,y,yaw); LayeredCostmap& get(); const std::string& getGlobalFrame(); }`

- [ ] **Step 1: 写 failing test**：`checkActivate` 多次幂等；`mutex()` 可递归 lock；`updateMap(rolling_window=true)` 移动 origin 并保留重叠数据。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现**：参考 `_legacy_*` 中 `costmap/components/costmap_wrapper.hpp`，但去掉 ROS topic。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: packages/costmap_server (CostmapWrapper lifecycle)`。

---

### Task 18: costmap/core + CostmapContext

**Files:**
- Create: `include/ayanami/costmap/core/costmap_planner.hpp`
- Create: `include/ayanami/costmap/core/costmap_smoother.hpp`
- Create: `include/ayanami/costmap/core/costmap_controller.hpp`
- Create: `include/ayanami/costmap/core/costmap_recovery.hpp`
- Create: `include/ayanami/costmap/components/costmap_context.hpp`
- Create: `tests/costmap_context_test.cpp`

**Interfaces:**
- Consumes: `core/abstract_*`、`packages/costmap_server`、`packages/costmap_2d`
- Produces:
  - `costmap::CostmapPlanner : abstract::AbstractPlanner` 加 `virtual void initialize(name, CostmapContext::Ptr)=0`
  - 同理 CostmapSmoother/CostmapController/CostmapRecovery
  - `costmap::CostmapContext : public abstract::AbstractMapContext`，内部持 global/local `LayeredCostmap::Ptr` + `CostmapWrapper`

- [ ] **Step 1: 写 failing test**：
  - TestCostmapContext 设全局/局部 LayeredCostmap + 帧名后：`getGlobalFrame()` 返回 "map"、`isCurrent()` 转发到 wrapper、`lock/unlock` 调用 `recursive_mutex`、`getRobotPose` 通过 TransformStore 取。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现**：CostmapContext 聚合两个 `CostmapWrapper` 与一个 `TransformStore&`；override `AbstractMapContext` 全部虚方法。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: costmap core + CostmapContext (AbstractMapContext override)`。

---

### Task 19: costmap/components (Costmap*Execution + Costmap*Action)

**Files:**
- Create: `include/ayanami/costmap/components/costmap_planner_execution.hpp`
- Create: `include/ayanami/costmap/components/costmap_controller_execution.hpp`
- Create: `include/ayanami/costmap/components/costmap_recovery_execution.hpp`
- Create: `include/ayanami/costmap/components/costmap_planner_action.hpp`
- Create: `include/ayanami/costmap/components/costmap_controller_action.hpp`
- Create: `include/ayanami/costmap/components/costmap_recovery_action.hpp`
- Create: `tests/costmap_components_test.cpp`

**Interfaces:**
- Consumes: `components/{*_execution,*_action}`、`packages/costmap_server`
- Produces: 在父类基础上覆写 `preRun/postRun` 注入 `checkActivate/checkDeactivate`、`safetyCheck` 调用 `isCurrent`、`run` 加 `recursive_mutex::scoped_lock`；Action 覆写 `createExecution` 工厂返回 costmap 版本（spec §5.4）

- [ ] **Step 1: 写 failing test**：
  - 创建 CostmapPlannerExecution 并 start → 验证执行前 `checkActivate` 被调、执行中 costmap mutex 被持锁、执行后 `checkDeactivate` 被调。
  - `safetyCheck` 当 `isCurrent()==false` 返回 false → controller 不发速度。
- [ ] **Step 3: 实现**：在 costmap_execution.hpp::preRun `{ costmap_wrapper_.checkActivate(); }`、postRun `{ costmap_wrapper_.checkDeactivate(); }`、safetyCheck `{ return costmap_wrapper_.isCurrent(); }`、run 锁加 `std::lock_guard<recursive_mutex>(costmap_wrapper_.mutex())` 再调父。Action 覆写 `createExecution`（按 MBF 风格）返回 costmap execution。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: costmap components Costmap*{Execution,Action} (Activate/Deactivate/锁/safetyCheck)`。

---

### Task 20: costmap/components/CostmapNavigationServer (Server 特化, virtual 注入地图)

**Files:**
- Create: `include/ayanami/costmap/components/costmap_navigation_server.hpp`
- Create: `src/costmap/components/costmap_navigation_server.cpp`
- Create: `tests/costmap_navigation_server_test.cpp`

**Interfaces:**
- Consumes: `components/navigation_server_base`、`costmap/core/**`、`costmap/components/**`、`packages/costmap_server`、`utility/config`
- Produces: `costmap::CostmapNavigationServer : public NavigationServerBase`（spec §5.4）

- [ ] **Step 1: 写 failing test**：
  - 实例化 CostmapNavigationServer，`configureCoreMaps(config_path)` 建 global/local CostmapContext + StaticLayer/InflationLayer/SimpleObstacleLayer。
  - `loadPlugins(config_path)` 触发虚 `initializePlannerPlugin` → `dynamic_pointer_cast<CostmapPlanner>(ptr) -> initialize(name, costmap_context_)` 被调（验证 conversion 成功且 context 注入正确）。
  - `handleObstaclePointCloud(PointCloud2{frame=lidar,...})` 下推到 SimpleObstacleLayer（用 TransformStore mock 提供 lidar→map）。
  - `fail case`：caster 指向一个非 CostmapPlanner 的抽象插件时 `dynamic_pointer_cast` 失败 → initializePlannerPlugin 返回 false 且 loadPlugins 报错。

- [ ] **Step 3: 实现 `costmap_navigation_server.hpp/cpp`**：
  - 持 `CostmapContext::Ptr costmap_context_`
  - 覆写 `initializePlannerPlugin/Smoother/Controller/Recovery`：dynamic_pointer_cast 到 costmap 族接口 → initialize(name, context)
  - 覆写 `newPlannerExecution/ControllerExecution/RecoveryExecution` 返回 Costmap 版（传入 costmap_wrapper）
  - `configureCoreMaps` 读 `config/costmap.json` 构 StaticLayer/InflationLayer/SimpleObstacleLayer
  - 构造期注册 `NavigationCallbacks::on_obstacle_pointcloud` 与 `on_obstacle_laserscan`，回调内同步 `handleObstacle*`
  - 处理点云时用 TransformStore 把 `cloud.header.frame_id` → `costmap_context_->getGlobalFrame()` 再写 obstacle layer
- [ ] **Step 4: 验证 pass**。

- [ ] **Step 5: Commit** `feat: costmap CostmapNavigationServer (virtual initialize*Plugin 注入 + layer 配置 + pointcloud 落图)`。

---

### Task 21: modules/costmap_smac_planner

**Files:**
- Create: `modules/costmap_smac_planner/costmap_smac_planner.hpp/.cpp`
- Create: `modules/costmap_smac_planner/module.cpp`（导出 `create_costmap_planner_module`）
- Create: `config/costmap_smac_planner.json`
- Create: `tests/costmap_smac_planner_test.cpp`

**Interfaces:**
- Consumes: `costmap/core/costmap_planner.hpp`、`A*` 算法（参考 _legacy SmacPlannerAStar）
- Produces: `costmap::CostmapSmacPlanner : public costmap::CostmapPlanner`；module.cpp 导出 `extern "C" CostmapPlanner* create_costmap_planner_module()`

- [ ] **Step 1: 写 failing test**：给定静态 map（障碍）+ start/goal → makePlan 返回 SUCCESS、path 非空、cost>0；起点 OUT_OF_MAP 失败；cancel() 中途打断。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现**：A* on costmap_2d.getCharMap()，按 4/8 连通，`neutral_cost + cost_factor*cell_cost`；lethal_cost=254 阻断。读 config json 的参数（use_diagonal/tolerance/cost_factor/lethal_cost）。参考 `_legacy_*` 中的实现但要纯 C++。
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: costmap_smac_planner module`。

---

### Task 22: modules/costmap_savitzky_smoother

**Files:**
- Create: `modules/costmap_savitzky_smoother/costmap_savitzky_smoother.hpp/.cpp`
- Create: `modules/costmap_savitzky_smoother/module.cpp`（导出 `create_costmap_smoother_module`）
- Create: `config/costmap_savitzky_smoother.json`
- Create: `tests/costmap_savitzky_smoother_test.cpp`

**Interfaces:**
- Consumes: `costmap/core/costmap_smoother.hpp`
- Produces: `CostmapSavitzkySmoother`；7 点 SG 滤波 `[-2,3,6,7,6,3,-2]/21`（参考 nav2_smoother §2）

- [ ] **Step 1: 写 failing test**：给一条 L 形 input path 平滑后输出比输入平滑（一阶差分极差减小）；max_duration=0.001s 立即返回 SUCCESS 但部分处理；空 path 直接 SUCCESS 空。
- [ ] **Step 2-5: 实现+验证+commit** `feat: costmap_savitzky_smoother module`。

---

### Task 23: modules/costmap_pid_controller (加速度限制 + Twist 清零)

**Files:**
- Create: `modules/costmap_pid_controller/costmap_pid_controller.hpp/.cpp`
- Create: `modules/costmap_pid_controller/module.cpp`（导出 `create_controller_module`）
- Create: `config/costmap_pid_controller.json`
- Create: `tests/costmap_pid_controller_test.cpp`

**Interfaces:**
- Consumes: `core/abstract_controller.hpp`（不是 costmap/core，控制器与地图无关——只读 robot pose + path）
- Produces: `CostmapPidController : public abstract::AbstractController`（三轴 PID + 前馈 + 加速度限制）

- [ ] **Step 1: 写 failing test**：
  - 给一条沿 X 的 path + robot pose 从 0 开始；首帧 `linear.x` 应 < `max_angular_accel+max_linear_accel` 控制下的 ramp，不应该是脏内存值（即必须是 `TwistStamped{};` 后填充）。
  - 角速度 ramp：setPlan 后第一帧 `angular.z` 应为小值（受 `max_angular_accel` 限制），不是大尖峰。
  - 同一目标 repeat setPlan **不重置** PID 状态（spec §修复导航原地旋转.md:230，避免顿挫）；换不同目标才 reset。
  - `isGoalReached` 在 dist<tolerance || 角度<tolerance 时返回 true → cmd_vel 停止。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现**：参考 `_legacy_*` 与 `test/pid_controller`；关键修复：
  - `computeVelocityCommands` 入口 `cmd_vel = geometry_msgs::TwistStamped{}; cmd_vel.twist = {};`（spec Global Constraints）
  - 差速模式角速度方向参考 pid_controller 实现取"最近车体方向角"
  - 同 `goal_frame` 调用 setPlan 不 reset tracking state（用一个 `last_plan_signature_` 判断）
  - linear 按 max_acc/max_decel 分开限幅；angular 受 max_angular_accel 限幅
  - distanceDecay (临近目标减速)、limitAcceleration
- [ ] **Step 4: 验证 pass**。
- [ ] **Step 5: Commit** `feat: costmap_pid_controller (Twist 清零 + 角速度加速度 ramp + 同 goal 不重置)`。

---

### Task 24: modules/costmap_*_recovery (4 个：spin/back_up/wait/clear_costmap)

**Files:**
- Create: `modules/costmap_spin_recovery/costmap_spin_recovery.hpp/.cpp` + `module.cpp`
- Create: `modules/costmap_back_up_recovery/costmap_back_up_recovery.hpp/.cpp` + `module.cpp`
- Create: `modules/costmap_wait_recovery/costmap_wait_recovery.hpp/.cpp` + `module.cpp`
- Create: `modules/costmap_clear_costmap_recovery/costmap_clear_costmap_recovery.hpp/.cpp` + `module.cpp`
- Create: `modules/costmap_spin_recovery/config/costmap_spin_recovery.json`（其余 3 同）
- Create: `tests/costmap_recovery_modules_test.cpp`

**Interfaces:**
- Consumes: `costmap/core/costmap_recovery.hpp`、`packages/costmap_server`
- Produces: 4 个 `Costmap*Recovery : public costmap::CostmapRecovery`，每个支持 `runRecovery`/`cancel`，每个 module.cpp 导出 `extern "C" CostmapRecovery* create_costmap_recovery_module()`

> **命名关键**：4 个插件的 `name` 字段分别为 `costmap_spin_recovery`、`costmap_back_up_recovery`、`costmap_wait_recovery`、`costmap_clear_costmap_recovery`，与 `costmap_navigation.json` 中的 `spin_id/wait_id/backup_id/clear_costmap_id` 一一对应（修现状 `recovery_id` 不匹配 bug，spec §6.6）。

- [ ] **Step 1: 写 failing test**：
  - `CostmapSpinRecovery::runRecovery` 在 `dt=10ms` 配置下完成所设 `spin_dist` 角度返回 SUCCESS；中途 `cancel()` 返回 CANCELED。
  - `CostmapBackUpRecovery` 倒车指定距离，用 costmap footprint 检测碰撞，碰 → FAILED。
  - `CostmapWaitRecovery` 等待 N 秒返回 SUCCESS（用 loop + cancel 检查）。
  - `CostmapClearCostmapRecovery` 调用 `CostmapContext::Ptr` 清除指定 map（global/local/both）一定 radius 范围。
- [ ] **Step 2-5: 实现+验证+commit** `feat: costmap recovery modules (spin/back_up/wait/clear_costmap)`。

> spin/back_up 两个 recovery 不需要 cmd_vel（M1 集成测试不连真底盘），只要 algorithmically 模拟提议 v/w。但为了 headless e2e 跑得过，可以让 spin/back_up/wait 都不阻塞真正底盘；clear_costmap 直接修 costmap。

---

### Task 25: modules/costmap_navigation (AbstractNavigation 宿主)

**Files:**
- Create: `modules/costmap_navigation/costmap_navigation.hpp`
- Create: `modules/costmap_navigation/costmap_navigation.cpp`
- Create: `modules/costmap_navigation/module.cpp`（导出 `extern "C" AbstractNavigation* create_navigation_module()`）
- Create: `config/costmap_navigation.json`（已在 Task 26 提前；如未在则这里创建）
- Create: `tests/costmap_navigation_test.cpp`

**Interfaces:**
- Consumes: `core/abstract_navigation.hpp`、`costmap/components/costmap_navigation_server`、`behavior_tree/pilot`、`utility/config`、`interfaces/callbacks`（停指令旁路）
- Produces: `CostmapNavigation : public AbstractNavigation`（spec §5.6）+ module.cpp

- [ ] **Step 1: 写 failing test**：
  - 模拟一个 fake `config/` 目录（含 `config.json` + `costmap_navigation.json` + `costmap.json` + 各插件 json + `BT/navigation.xml`）。
  - `CostmapNavigation::initialize(config_path)` 触发：CostmapNavigationServer 构造 + configureCoreMaps + loadPlugins（dlopen 各 .so） + 注册 on_obstacle_pointcloud → 落图；Pilot 构造 + loadFromFile + setNavigationServer。
  - `start()` 后 `spinOnce()` 周期触发 `on_costmap_visualization`（旁路被 fake 观察者捕获）。
  - `setGoal(PoseStamped{map frame})`：启 navigation_thread_ 跑 BT → 验证 BT 主流程推进，raw_path/smoothed_path 写入黑板，cmd_vel 通过 `on_velocity_command` 发出（fake goal 已 close，isGoalReached 后停）。
  - 新 goal 抢占：再调 `setGoal(new_goal)`，验证 cmd_vel 先发零速度、`session_id` 自增、旧 task 被 cancel，BT 树不重建（同一 tree 实例 reuse）。
- [ ] **Step 2: 验证 fail**。
- [ ] **Step 3: 实现 `costmap_navigation.cpp`**：
  - 持 `CostmapNavigationServer navigation_server_` + `BehaviorTreePilot pilot_` + `MapCallback map_callback_` + `std::unique_ptr<std::thread> navigation_thread_` + `std::atomic<uint64_t> session_id_/plan_version_` + 配置变量
  - `initialize(config_path)`：`navigation_server_.loadPlugins(config_path)` + `navigation_server_.createActions()`；`pilot_.configure(config_path)` + `pilot_.setNavigationServer(&navigation_server_)`
  - `start()`：`navigation_server_.start()` + `publishMapsForVisualization()`（首发一次）
  - `spinOnce()`：限频（`costmap_visualization_rate`）触发 `publishMapsForVisualization()`
  - `setGoal(goal)`：
    ```
    { std::lock_guard lk(goal_mutex_);
      if (navigation_thread_ && navigation_thread_->joinable()) {
        // 抢占流程 (spec §6.1)
        TwistStamped stop{};                                          // 全字段清零
        NavigationCallbacks::on_velocity_command(stop);               // 先发零速度
        navigation_server_.controllerAction()->cancelAll();
        pilot_.halt();
        auto old = std::move(navigation_thread_);
        navigation_thread_.reset();
        // move 旧 join 出锁外
        { /* unlock */ old->join(); }
      }
      session_id_++;
      plan_version_ = 0;
      pilot_.beginNavigation(goal);
      navigation_thread_ = std::make_unique<std::thread>([this]{ pilot_.run(); });
    }
    ```
  - `stop()`：`pilot_.stop()` → `navigation_server_.stop()` → reset navigation_thread_
  - `setMapCallback(cb)`：保存到 `map_callback_`
  - `publishMapsForVisualization()`：转 costmap → `OccupancyGrid` 调 `map_callback_("global_costmap", grid)`（如果有 callback），同时 emit `NavigationCallbacks::on_costmap_visualization`
  - module.cpp 导出 `create_navigation_module` 返回 `new CostmapNavigation`
- [ ] **Step 4: 验证 pass**（关键：M1 headless 接通 path→cmd_vel）。
- [ ] **Step 5: Commit** `feat: costmap_navigation 宿主插件 (AbstractNavigation + BT 抢占零速度 + 旁路可视化)`。

---

### Task 26: config/*.json + BT/navigation.xml

**Files:**
- Create: `config/config.json`
- Create: `config/logging.json`
- Create: `config/costmap_navigation.json`
- Create: `config/costmap.json`
- Create: `config/BT/navigation.xml`
- Create: `config/costmap_smac_planner.json`、`costmap_savitzky_smoother.json`、`costmap_pid_controller.json`、`costmap_spin_recovery.json`、`costmap_back_up_recovery.json`、`costmap_wait_recovery.json`、`costmap_clear_costmap_recovery.json`
- Create: `assets/`（留 nav.json/nav.pgm + rmuc26.json/pgm 现有）

**Interfaces:** 纯配置文件，不 consumes / 不 produces 代码。

- [ ] **Step 1: 写 `config.json`**：严格按 spec §2.3，global 段映射 + `config_logging`；**不写死 logging_dir 绝对路径，用相对 `config_path/logs`**。
- [ ] **Step 2: 写 `costmap_navigation.json`** 按 spec §9.3：
  ```jsonc
  { "navigation": {
      "behavior_tree_xml": "BT/navigation.xml",
      "behavior_tree_id": "NavigateWithReplanningAndSmoothing",
      "planner_id":"costmap_smac", "smoother_id":"costmap_savitzky",
      "controller_id":"costmap_pid", "recovery_id":"costmap_clear_costmap",
      "clear_costmap_id":"costmap_clear_costmap",
      "spin_id":"costmap_spin", "wait_id":"costmap_wait", "backup_id":"costmap_back_up",
      "planner_frequency":-1.0, "smoother_frequency":-1.0,
      "controller_frequency":20.0,
      "goal_tolerance":0.25, "angle_tolerance":0.20,
      "robot_frame":"base_footprint", "costmap_visualization_rate":1.0 } }
  ```
  注意：`_id` 是插件`name` 段缩写（如 `costmap_smac`），调用方查 `name=costmap_smac` 找 `costmap_smac_planner.json`（**修复**现状 `recovery_id="costmap_cleanup"`而插件 name 是 `costmap_cleanup_recovery` 不匹配的坑——本 task 确保 `_id` 全部对应插件 `name`）。
- [ ] **Step 3: 写 `costmap.json`**：global/local 尺寸/分辨率/origin/rolling/static_layer(map_path)/inflation_layer/inflation_radius/clamp_cells/obstacle_layer(obstacle_range/raytrace_range/observation_sources)/footprint。
- [ ] **Step 4: 写各模块 json**：每个含 `name/so_name/create_symbol` + 算法参数；so_name 形如 `libayanami_costmap_smac_planner.so`、`libayanami_costmap_spin_recovery.so` 等。
- [ ] **Step 5: 写 `config/BT/navigation.xml`**：严格按 spec §6.3 的 XML。
- [ ] **Step 6: 写一个 `pytest`/脚本 `tests/check_config_consistency.cpp`**：加载 `costmap_navigation.json` + 各插件 json，断言所有 `_id` 都能在对应 plugin manager 的 loaded plugins 里找到 name（避免再次出现 id 不匹配）。
- [ ] **Step 7: 验证**（编译运行 `check_config_consistency`）
- [ ] **Step 8: Commit** `feat(config): 全套 config json + BT/navigation.xml + id↔name 一致性检查`。

---

### Task 27: 复刻 6 个 legacy gtest 契约

**Files:**
- Create: `tests/action_protocol_test.cpp`
- Create: `tests/cancel_responsiveness_test.cpp`
- Create: `tests/continuous_goal_test.cpp`
- Create: `tests/controller_session_test.cpp`
- Create: `tests/latest_wins_test.cpp`
- Create: `tests/recovery_exhaustion_test.cpp`
- Modify: `CMakeLists.txt` 注册这些为 gtest target

**Interfaces:** consumes 各 `components/*`，每个 test 用各 task 写的 Test* 测试桩插件 + mock map context；行为断言复刻 `_legacy_ayanami/tests/` 中同名文件的关键场景。

- [ ] **Step 1: 通读 `_legacy_ayanami/tests/` 六个文件**，提炼每个的核心断言（不直接复制 legacy 代码，因为接口已收尾成新版本）：
  - `action_protocol_test`：goal lifecycle（Pending→Accepted→Executing→Succeeded/Aborted/Canceled）的法定转换。
  - `cancel_responsiveness_test`：cancel RUNNING action 在指定时间内回调并改 status 为 CANCELED；recovery action 中途 cancel 也响应。
  - `continuous_goal_test`：连续发同 goal 等价语义：第二次 call 不无限重 spin（避免抖动）。
  - `controller_session_test`：controller 一次 setPlan + 多帧 `computeVelocityCommands` 直至 ARRIVED_GOAL。
  - `latest_wins_test`：同槽位抢占：旧 goal 应被 preempt 并立即返回 CANCELED/Aborted；新 goal 拿到 RUNNING。
  - `recovery_exhaustion_test`：恢复行为列表耗尽后整次导航 FAILED（参考 MBF `actionRecoveryResult` §2.7；对应到 ayanami 是 BT 外层 RecoveryNode retries 耗尽）。
- [ ] **Step 2-5: 每个测试改写为基于新接口的 pytest/gtest**，独立 commit。
- [ ] **Last step: 串跑测试**

```bash
cmake -S . -B build_m1 -DBUILD_TESTING=ON && cmake --build build_m1
ctest --test-dir build_m1 --output-on-failure
```

Expected：6 契约 + 之前 task 的组件测试全 PASS。
- [ ] **Commit** `test: 复刻 legacy 6 个 gtest 契约为新接口`。

---

### Task 28: headless E2E 测试 (M1 终止验证) + catkin build ayanami 通过 + push

**Files:**
- Create: `tests/headless_e2e_test.cpp`（M1 终止里程碑）
- Modify: `CMakeLists.txt`
- Create: `tests/assets/headless_map.json/.pgm`（小尺寸 grid 用于 e2e）

**Interfaces:** consumes 完整的 dlopen `libayanami_costmap_navigation.so`（宿主）；不连真底盘，模拟 odom/pointcloud/goal，验证 cmd_vel 通过 `NavigationCallbacks::on_velocity_command` 发出且 navigation_session 最终 SUCCEEDED。

- [ ] **Step 1: 写 failing test**

```cpp
TEST(HeadlessE2E, FullNavigationPipeline) {
  // 1. GlobalConfig::instance(/* temp config dir with full config suite */);
  // 2. dlopen libayanami_costmap_navigation.so, create_navigation_module
  AbstractNavigation::Ptr nav = load_module_from_so<...>(...);
  nav->initialize(config_path);
  // 3. 收集 cmd_vel
  TwistStamped last_cmd;
  int cmd_count = 0;
  NavigationCallbacks::on_velocity_command.add([&](auto& v){ last_cmd=v; cmd_count++; });
  // 4. 收集 map 可视化 (验证 spinOnce 触发)
  int map_count = 0;
  nav->setMapCallback([&](auto& name, auto& g){ map_count++; });
  // 5. 启动
  nav->start();
  // 6. 模拟 robot pose: 每帧 setRobotState (经 NavigationCallbacks::on_robot_state_updated, robot 从 0,0 沿 path 移动)
  //    + 喂虚拟 pointcloud (mark 一个虚拟障碍物场景空)
  // 7. nav->setGoal(PoseStamped{frame=map, x=2,y=0, yaw=0})（virtual goal）
  // 8. 等待 cmd_count 达到预期 / navigation SessionId 推进
  // 9. 断言: cmd_count > 阈值, last_cmd.twist.linear.x 已初始化（!==脏内存），last_cmd.twist 是 limit_accel 控制下的小值
  // 10. nav->stop();
  EXPECT_GT(cmd_count, 1);
  EXPECT_TRUE(std::isfinite(last_cmd.twist.linear.x));
  EXPECT_FALSE(std::isnan(last_cmd.twist.angular.z));
}
```

- [ ] **Step 2: 验证 fail**（M1 全面跑通前的最后一个红灯）。
- [ ] **Step 3: 跑通测试**

按依赖保证：先在单独子任务里解决任何遗漏的小问题（start/stop 顺序、stop 命令清零、bt tree tick 频率、goal 抢占）；如果时序 resemble nav2/PipelineSequence 频率分层（spec §6.4），goal 推进时 controller 一直在自己循环里发速度。让 cmd_count 大于阈值证明整条链已通。

- [ ] **Step 4: 跑整库测试 + catkin build ayanami**

```bash
cd /share/nav_ws
catkin build ayanami          # 仅 build ayanami，不构 ayanami_ros
ctest --output-on-failure    # 在 build_ayanami 测试目录跑所有 gtest
```

Expected：全部 PASS；`libayanami_costmap_navigation.so` 与所有模块 `libayanami_*.so` 生成在 `devel/lib/`。

- [ ] **Step 5: 静态红线检查脚本**

```bash
cd /share/nav_ws/src/ayanami
# 核心层零 ROS include 检查
! grep -r "ros/ros.h\|tf2_ros/\|sensor_msgs/\|nav_msgs/" include/ayanami/{core,components,behavior_tree,utility,interfaces,messages}/
# abstract 层零地图 include 检查
! grep -r "costmap_2d/\|gridmap_2d/" include/ayanami/core/ include/ayanami/components/ include/ayanami/behavior_tree/
# 命名检查: 禁止 AbstractBehavior / BehaviorAction / behavior_execution
! grep -rwi "AbstractBehavior\|BehaviorAction\|behavior_execution" include/ayanami/ modules/
```

Expected：3 个 grep 全部退出 0（passes match = 0 个匹配），spec Global Constraints 满足。

- [ ] **Step 6: Commit + Push**

```bash
cd /share/nav_ws/src/ayanami
git add -A
git commit -m "test: headless E2E 打通 (M1 终止验证) + 静态红线脚本

M1 milestone: ayanami 纯 C++ 导航核心重写完成
- libayanami + 8 模块 .so (costmap_navigation 宿主 + 1 planner + 1 smoother + 1 controller + 4 recovery) 全部能 catkin build 通过
- headless e2e 测试 dlopen libayanami_costmap_navigation.so 模拟 goal→path→cmd_vel 全链路, 零 ROS 依赖
- 6 个 legacy 接口契约 gtest 复刻通过
- 静态红线: 核心零 ROS include, abstract 零地图 include, 命名全部 recovery"
git push -u origin redesign
```

- [ ] **Step 7: 把一切提交到 ayanami_ros 仓库（空 README + master 占位，避免仓库长期裸着）**

```bash
cd /share/nav_ws/src/ayanami_ros
# 仍是 plan mode 后的空目录 (旧文件已 mv 到 _legacy_ayanami_ros)
# 写一个占位 README 提交:
echo "# ayanami_ros (ROS1 边界层)" > README.md
echo "见 ../ayanami/docs/superpowers/specs 与 plans 描述本包职责（M2 实施将填 src/）" >> README.md
git checkout redesign
git add README.md
git commit -m "docs: placeholder, M2 实施将注入 ROS 边界代码"
git push -u origin redesign
```

---

## Self-Review (Plan 与 Spec 对照)

- [x] **Spec coverage**: 逐节核对：
  - §1 顶层架构 → Task 0
  - §2 参数加载 → Task 1
  - §3 全局回调 → Task 2 + Task 6
  - §4 插件 + extension → Task 3 + Task 6 (echo extension 跨 .so 测试) + 后续 Plan 2 (ROS extension)
  - §5 抽象 + virtual + MapContext → Task 7 + Task 18 + Task 20
  - §6 BT 编排 → Task 12 + 13 + 14
  - §7 ayanami_ros 边界 → Plan 2 (本轮不动)
  - §8 TransformStore → Task 5
  - §9 配置 → Task 26
  - §10 已知坑规避 → 散落各 Task 的 Step 3 + Step 5/6 检查脚本（Task 28）
  - §11 阶段 A 全覆盖；§11 阶段 B 留 Plan 2。
- [x] **Placeholder scan**: 28 个 Task 全部含具体 step + 测试代码/断言 + commit 命令；无 "TBD/TODO/Later"。
- [x] **Type consistency**: 抽查 `AbstractMapContext` (Task 7 定义) → Task 18 CostmapContext override、Task 20 注入：参数与返回类型一致；`TaskHandle.ready()/result()/onComplete()/cancel()` (Task 9 定义) → Task 13/14 use：方法名一致；`NavigationServerBase::requestPath/Smoothing/Execution/Recovery` (Task 11 定义) → Task 13/14/25 调用：签名一致；`create_costmap_planner_module / create_navigation_module` 符号名贯穿 Task 3/15-25；`config.json` 的 `global` 段 mapping 与各 `*_id`（Task 26）一致性由 Task 26 Step 6 的 consistency test 保证。

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-08-14-ayanami-core-redesign.md`. Two execution options:**

**1. Subagent-Driven (recommended)** - 每个任务派发一个 fresh subagent 实施，主会话在 task 之间 review；快速迭代；适合 28 任务这样的大工程。

**2. Inline Execution** - 在当前会话里批处理执行（executing-plans），遇 checkpoint 暂停 review；上下文连续。

**选择哪种？**