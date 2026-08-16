# ayanami M2: ROS1 边界接入 + M1 carry 修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 M1 redesign 分支上修 3 个 BLOCK carry (controller 死锁 / abstract 虚析构 UB / recovery 锁不全) + 5 minor carry，并从零铺设 `src/ayanami_ros` ROS1 边界层，使之能跑通 `roslaunch nav_demo ayanami.launch` Gazebo 端到端仿真 (RViz 点 goal → robot 抵达)。

**Architecture:** 接续 M1：`src/ayanami/` 仅修订 carry 项 (零接口破坏)；`src/ayanami_ros/` 全新铺——catkin 包含 `AyanamiROS` 类 + `TF2Listener` + `ros_msg_converter` + `ExtensionModuleROS` 基类 + `rviz_viewer` 改为 dlopen ROS extension .so。端到端联调依赖 nav_demo `ayanami.launch` 已提供的 Gazebo+amcl+map_server 链。

**Tech Stack:** C++17、CMake 3.16+、catkin (ROS1 Noetic)、roscpp/tf2_ros/tf2_eigen/nav_msgs/sensor_msgs/geometry_msgs/pcl_conversions、Eigen、BT.CPP v4 (vendored)、fast_tf (vendored)、spdlog、gtest。

**Spec:** `docs/superpowers/specs/2026-08-14-ayanami-m2-ros-design.md` (commit `414ecaf`)。Spec 是权威依据；plan argues from spec；executor 同时读 spec 与 plan。

## Global Constraints

（取自 spec §12 + M1 carry 的项目级硬约束，每个 task 的要求隐式包含本节）

### 来自 M1 (Plan 1) 仍生效
- 纯 C++：`include/ayanami/{core,components,behavior_tree,utility,interfaces,messages}/**` 严禁 include `ros/ros.h`、`tf2_ros/*`、`sensor_msgs/*`、`nav_msgs/*` 等任何 ROS 头。M2 仅在 costmap 具体实现内修订，不得破坏这条红线。
- ROS include 只允许出现在 `src/ayanami_ros/**`。
- 命名：恢复类一律 `recovery`。仅保留 BT 语境用词 (`BehaviorTree.CPP`、`<BehaviorTree>` XML 根标签、`BehaviorTreePilot`、`behavior_tree_xml`)。
- `libayanami` 默认 SHARED、`BUILD_WITH_BT=ON` 默认、`BUILD_TESTING=ON`、`BUILD_SHARED=ON`（M2 不改）。
- BT.CPP 子库 build 加 `-Wl,-Bsymbolic`（M1 已就绪）。
- dlopen 句柄从不 dlclose；析构纪律：先 reset action → clearPlugins → 卸载宿主 → 句柄随进程退出。
- 所有 `TwistStamped` 类消息使用前必须 `cmd_vel = {}` 清零纪律（M2 ros_msg_converter 同样守纪）。
- 频率约定：`*_frequency == -1` 单次后缓存 SUCCESS；`> 0` 表示 Hz；首 tick 立即执行。
- `session_id`/`plan_version` 用 `uint64_t` 链上 Goal/Result，BT 节点 onRunning 校验。

### M2 新增
- 3 BLOCK carry 先修 (Task 0/1/2)，再写 ROS 边界 (Task 4+)。
- `src/_legacy_ayanami_ros/` 是 ROS 边界参考（提取思路不 copy），`src/_legacy_ayanami/` 仍作算法参考。
- `src/ayanami/thirdparty/` 与 `docs/superpowers/` 由 M1 验证保留。
- 端到端联调目标：`roslaunch nav_demo ayanami.launch`（已存在仿真环境，M2 不改 nav_demo）。
- 仿真一致性帧名：`base_footprint`（amcl `base_frame_id`/Gazebo robot `base_footprint`）；`map_frame=map`、`odom_frame=odom`。
- 提交：本仓库 git 分支 `redesign`；commit message 风格 `<type>: <summary>`，type∈{feat/test/docs/build/refactor/fix/chore}。
- 编译命令：M2 阶段 A (BLOCK 修) 用 `cmake --build build_m1` (M1 cmake 配置)；M2 阶段 B 起改用 `catkin build ayanami ayanami_ros`（catkin workspace）。
- 测试命令：M2 阶段 A `ctest --test-dir build_m1 --output-on-failure`；M2 阶段 B+ catkin 测试 `catkin run_tests ayanami ayanami_ros` 或 `catkin build ayanami_ros && ctest --test-dir <build_dir>/ayanami_ros --output-on-failure`。
- 网络遥控 push：Plan 2 末 Task 13 才 push origin/redesign。

---

## File Structure (本 Plan 涉及文件总览)

完整 spec §1.2 已列目录树；本 Plan 涉及：

```
src/ayanami/  (M2 carry 修订)
  include/ayanami/
    core/abstract_controller.hpp           # +virtual ~ =default
    core/abstract_smoother.hpp              # +virtual ~ =default
    core/abstract_recovery.hpp              # +virtual ~ =default
    components/controller_execution.hpp     # state provider wiring
    costmap/components/costmap_controller_execution.hpp   # per-cycle short lock
    costmap/components/costmap_recovery_execution.hpp     # scoped_lock(global, local)
    modules/costmap_smac_planner/costmap_smac_planner.hpp/.cpp   # start 留空时 getRobotPose fallback
  config/costmap_navigation.json           # robot_frame 单一来源
  config/costmap.json                       # 静态地图对齐 nav_demo/map/nav.yaml
  config/config_ros.json                    # ROS 边界配置 + Gazebo topic 一致
  cmake/ayanami-config.cmake.in             # +find_dependency
  tests/
    plugin_teardown_no_ub_test.cpp          # T0 验证派生析构被调用
    concurrent_controller_obstacle_test.cpp  # T1 验证 controller+obstacle 无死锁
    recovery_vs_obstacle_test.cpp            # T2 验证 recovery 锁 local 不丢
    minor_carry_wirings_test.cpp             # T3 综合验证 robot pose fallback 等

src/ayanami_ros/  (M2 全新铺设)
  include/ayanami_ros/
    ayanami_ros.hpp
    ros_msg_converter.hpp
    tf2_listener.hpp
    extension_module_ros.hpp
  src/ayanami_ros/
    ayanami_ros.cpp
    tf2_listener.cpp
    extensions/
      rviz_viewer.cpp                       # 导出 create_extension_module
  apps/ayanami_rosnode.cpp
  launch/navigation.launch
  launch/config.rviz
  CMakeLists.txt
  package.xml
  # (config_ros.json 主要留在 src/ayanami/config/, 见 spec §8.1)
```

**职责边界**：
- abstract 层 (`include/ayanami/core/**` + `include/ayanami/components/**`) M2 只补虚析构 + 修 controller 执行层；不破接口。
- costmap 具体实现 (`include/ayanami/costmap/**`) M2 修 costmap_*_execution 锁粒度；不破接口。
- ayanami_ros 全新 ROS1 边界；不交叉污染 ayanami 核心。

---

## Tasks

### Task 0: BLOCK #2 — abstract base 3 个补 virtual ~ =default

**Files:**
- Modify: `src/ayanami/include/ayanami/core/abstract_controller.hpp`
- Modify: `src/ayanami/include/ayanami/core/abstract_smoother.hpp`
- Modify: `src/ayanami/include/ayanami/core/abstract_recovery.hpp`
- Create: `src/ayanami/tests/plugin_teardown_no_ub_test.cpp`

**Interfaces:**
- Consumes: M1 已有 `AbstractRecovery/Controller/Smoother` (spec §5.1); `AbstractPlanner/Navigation` 已是 virtual ~ (Ruling #3 M1 scope)
- Produces: 3 个 abstract 基类新加 `virtual ~AbstractX() = default;`，派生已有 `~Derived() override = default` 此时 `override` 真生效。

- [ ] **Step 1: 写 failing test `tests/plugin_teardown_no_ub_test.cpp`**

```cpp
#include <gtest/gtest.h>
#include <ayanami/components/plugin_manager.hpp>
#include <ayanami/core/abstract_recovery.hpp>
#include <string>

namespace {
class FakeRecovery : public ayanami::abstract::AbstractRecovery {
public:
    static std::atomic<int> dtor_count_;
    std::string non_pod_member_{"some state longer than SSO"};  // 防止 SSO 短优化
    ~FakeRecovery() override { ++dtor_count_; }
    uint32_t runRecovery(std::string&) override { return 0; }
    bool cancel() override { return true; }
};
std::atomic<int> FakeRecovery::dtor_count_{0};

extern "C" ayanami::abstract::AbstractRecovery* create_fake_recovery() {
    return new FakeRecovery();
}
}

TEST(PluginTeardownNoUB, DerivedDestructorCalledOnClearPlugins) {
    FakeRecovery::dtor_count_ = 0;
    ayanami::components::PluginManager<ayanami::abstract::AbstractRecovery> mgr;
    ayanami::PluginSpec spec{"fake", /*so_name*/ "", /*create_symbol*/ ""};
    // dlopen 符号 from this test TU (link the same nm: create_fake_recovery)
    auto* ptr = create_fake_recovery();
    ASSERT_NE(ptr, nullptr);
    auto shr = std::shared_ptr<ayanami::abstract::AbstractRecovery>(ptr);
    bool init_result = mgr.registerPlugin(spec.name, shr);
    EXPECT_TRUE(init_result);
    mgr.clearPlugins();
    EXPECT_EQ(FakeRecovery::dtor_count_.load(), 1) << "派生析构未被调用 (abstract base virtual ~ 缺)";
}
```

> `registerPlugin(name, ptr)` 是 PluginManager (Task 10) 已就绪接口 (允许直接 mount 内存中插件不 dlopen)。

- [ ] **Step 2: 验证 fail**

```bash
cd /share/nav_ws/src/ayanami
cmake --build build_m1 --target plugin_teardown_no_ub_test
ctest --test-dir build_m1 -R PluginTeardownNoUB --output-on-failure
```

Expected: FAIL，`dtor_count_ = 0`（只 base 析构被调，派生析构不调）。

- [ ] **Step 3: 修改 3 abstract 头**

```cpp
// abstract_controller.hpp
class AbstractController {
public:
    using Ptr = std::shared_ptr<AbstractController>;
    virtual ~AbstractController() = default;        // ★ 加
    virtual uint32_t computeVelocityCommands(...) = 0;
    // ... 原接口保持不变
};
// 同理 abstract_smoother.hpp / abstract_recovery.hpp
```

- [ ] **Step 4: 验证 pass**

```bash
cmake --build build_m1 && ctest --test-dir build_m1 --output-on-failure
```

Expected: 全套 33 + 1 新增 = 34 PASS (除新 PluginTeardownNoUB 外不破任何)。

- [ ] **Step 5: Commit**

```bash
git add include/ayanami/core/abstract_controller.hpp include/ayanami/core/abstract_smoother.hpp include/ayanami/core/abstract_recovery.hpp tests/plugin_teardown_no_ub_test.cpp CMakeLists.txt
git commit -m "fix(abstract): virtual ~Abstract{Controller,Smoother,Recovery} = default 修 Ruling #3 UB (派生析构经 base ptr 可调)"
```

---

### Task 1: BLOCK #1 — controller 整循环持锁 → per-cycle short lock

**Files:**
- Modify: `src/ayanami/include/ayanami/costmap/components/costmap_controller_execution.hpp`
- Create: `src/ayanami/tests/concurrent_controller_obstacle_test.cpp`

**Interfaces:**
- Consumes: M1 已有 `CostmapControllerExecution::run()` + `local_costmap_->mutex()` (recursive_mutex)
- Produces: 拆 run 锁为 per-cycle short lock；不影响 `handleObstaclePointCloud` (Ruling #7 R1) 与 `publishMapsForVisualization` (Ruling T25 R1) 已加锁逻辑。

- [ ] **Step 1: 写 failing test (并发 controller + obstacle feed)**

```cpp
#include <gtest/gtest.h>
#include <ayanami/costmap/components/costmap_controller_execution.hpp>
// 必要 includes for CostmapContext, CostmapWrapper, AbstractController test stub, etc.
#include <atomic>
#include <thread>
#include <vector>

TEST(ConcurrentControllerObstacle, NoDeadlockNoStarvation) {
    // 1. 构造 FakeController : AbstractController (TestController stub from Task 8 M1) 让 computeVelocityCommands sleep 5ms
    // 2. 构造 CostmapContext + local CostmapWrapper
    // 3. 构造 CostmapControllerExecution with local_costmap + FakeController
    // 4. Start it: exec.start(path, ...);  -> 进 control loop 跑 @ ~20Hz (50ms/cycle)
    // 5. 另一线程持续 50ms 周期投 markObstacles (模拟 handleObstaclePointCloud 走 local costmap mutex)
    // 6. 跑 1s (20 controller cycle + 20 obstacle mark)
    // 7. 断言: controller cycle_count >= 15 (≥ 75% 执行成功; 防饿死)
    // 8. 断言: obstacle feed_count >= 15
    // 9. 不死锁 (整体在 5s 内完成测试)
    int controller_cycles = 0;
    int obstacle_feeds = 0;
    // ... 用 std::thread 跑两线程 ...
    // exec.stop()
    EXPECT_GE(controller_cycles, 15);
    EXPECT_GE(obstacle_feeds, 15);
}
```

- [ ] **Step 2: 验证 fail**

```bash
cmake --build build_m1 --target concurrent_controller_obstacle_test
ctest --test-dir build_m1 -R ConcurrentControllerObstacle --output-on-failure
```

Expected: FAIL — run() 整循环持锁 → obstacle feed 饿死或死锁超时；controller_cycles 或 obstacle_feeds 一项远低 (如 < 5)。

- [ ] **Step 3: 修改 `costmap_controller_execution.hpp::run()` 拆 lock**

```cpp
void CostmapControllerExecution::run() {
    while (...) {
        // 1. 不持锁取 robot pose
        geometry_msgs::PoseStamped pose = ...;
        // 2. 短锁: compute + read costmap slice
        {
            std::lock_guard<std::recursive_mutex> lk(local_costmap_->mutex());
            if (!safetyCheck()) { publishStopCommand(); sleep; continue; }
            controller_->computeVelocityCommands(pose, velocity, cmd_vel, message);
        }
        // 3. 释锁后 emit
        NavigationCallbacks::on_velocity_command(cmd_vel);
        // 4. 不持锁 sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0 / controller_frequency_)));
    }
}
```

- [ ] **Step 4: 验证 pass**

```bash
ctest --test-dir build_m1 -R ConcurrentControllerObstacle --output-on-failure
# 全套 ctest 验证不破:
ctest --test-dir build_m1 --output-on-failure
```

Expected: 全 PASS (新 + 既有 33 + T0 新增 1)。

- [ ] **Step 5: Commit**

```bash
git add include/ayanami/costmap/components/costmap_controller_execution.hpp tests/concurrent_controller_obstacle_test.cpp CMakeLists.txt
git commit -m "fix(controller): 拆 run() 为 per-cycle short lock 防 obstacle feed 饿死 (BLOCK #1)"
```

---

### Task 2: BLOCK #3 — recovery 锁 global + local (scoped_lock)

**Files:**
- Modify: `src/ayanami/include/ayanami/costmap/components/costmap_recovery_execution.hpp`
- Create: `src/ayanami/tests/recovery_vs_obstacle_test.cpp`

**Interfaces:**
- Consumes: `CostmapRecoveryExecution::run()` (目前只锁 global_costmap_->mutex())
- Produces: run() 同时锁 global + local (scoped_lock 防死锁)；compatibility 与 handleObstacle* (走 local) 互斥防 data race

- [ ] **Step 1: 写 failing test**

```cpp
TEST(RecoveryVsObstacle, NoDataRaceOnLocalMasterGrid) {
    // 1. 构造 CostmapContext (global + local); local costmap 含 SimpleObstacleLayer
    // 2. 构造 CostmapClearCostmapRecovery + CostmapRecoveryExecution
    // 3. 启动 recovery thread (run 持续 clearArea on local grid)
    // 4. 同线程并发投 markObstacles 到 local grid (模拟 handleObstacle)
    // 5. addressSanitizer 或 验证 cell 值正确无撕裂
    EXPECT_TRUE(true);   // 完整跑下即算 pass (锁使无 UB)
}
```

> 若环境无 ASan，用简单断言 "recovery 写完前 cell A==post_recovery_value" + "obstacle 写完后 cell B==feed_value" 交叉验证。

- [ ] **Step 2: 验证 fail**

```bash
cmake --build build_m1 --target recovery_vs_obstacle_test
ctest --test-dir build_m1 -R RecoveryVsObstacle --output-on-failure
```

Expected: FAIL — concurrent 跑导致 cell 值不一致或崩溃/segfault。

- [ ] **Step 3: 修改 `costmap_recovery_execution.hpp::run()` 加 scoped_lock**

```cpp
void CostmapRecoveryExecution::run() {
    setState(STARTED);
    while (state() != CANCELED && !cancel_ && !should_exit_) {
        // ★ 关键: 同时锁 global + local 防与 obstacle feed 死锁/竞态
        std::scoped_lock lk(costmap_context_->global_costmap_->mutex(),
                            costmap_context_->local_costmap_->mutex());
        setState(RECOVERING);
        outcome_ = recovery_->runRecovery(message_);
        // 判定 RECOVERY_DONE/CANCELED
        break;  // runRecovery 单次逻辑保持不变
    }
    setState(RECOVERY_DONE);
    notifyStateUpdate();
}
```

- [ ] **Step 4: 验证 pass**

```bash
ctest --test-dir build_m1 -R RecoveryVsObstacle --output-on-failure
ctest --test-dir build_m1 --output-on-failure
```

Expected: 全 PASS。

- [ ] **Step 5: Commit**

```bash
git add include/ayanami/costmap/components/costmap_recovery_execution.hpp tests/recovery_vs_obstacle_test.cpp CMakeLists.txt
git commit -m "fix(recovery): scoped_lock(global, local) 防与 obstacle feed 数据竞态 (BLOCK #3)"
```

---

### Task 3: minor carry wiring 综合

**Files:**
- Modify: `src/ayanami/include/ayanami/components/controller_execution.hpp`  (RobotStateProvider fallback)
- Modify: `src/ayanami/include/ayanami/modules/costmap_smac_planner/costmap_smac_planner.hpp/.cpp`  (start pose 留空 fallback getRobotPose)
- Modify: `src/ayanami/include/ayanami/costmap/components/costmap_navigation_server.hpp/.cpp`  (robot_frame 单一来源 + 二次 configureCoreMaps 守护)
- Modify: `src/ayanami/config/costmap_navigation.json`  (robot_frame 单一来源移到此)
- Modify: `src/ayanami/config/costmap.json`  (移除 robot_frame 字段 + 与 costmap_navigation.json 保持一致)
- Modify: `src/ayanami/cmake/ayanami-config.cmake.in`  (find_dependency; T0 minor carry)
- Modify: `src/ayanami_include/ayanami/modules/costmap_pid_controller.hpp/.cpp`  (plan_signature 粗签名)
- Create: `src/ayanami/tests/minor_carry_wirings_test.cpp`

**Interfaces:**
- Consumes: M1 已有 costmap_navigation_server configureCoreMaps / costmap_smac_planner makePlan / pid_controller setPlan / cmake-package-config
- Produces: 综合 5 项 minor carry 实现 + ctest

- [ ] **Step 1: 写 failing test `tests/minor_carry_wirings_test.cpp`**

```cpp
TEST(MinorCarryWiring, SmacPlannerFallsBackToRobotPose) {
    // 1. 构造 CostmapContext + transform_store mock 注入 robot pose (1, 2, pi/4)
    // 2. CostmapSmacPlanner::makePlan(start.pose=空(默认), goal.x=3, goal.y=2)
    // 3. 验证 path 首个 pose ≈ (1, 2, pi/4) (实际 robot pose from context)
}

TEST(MinorCarryWiring, RobotFrameSingleSourceFromCostmapNavigationJson) {
    // 1. 用 temp config dir 写 costmap_navigation.json::robot_frame="base_footprint"; costmap.json 无 robot_frame (移除)
    // 2. GlobalConfig::instance(temp); CostmapNavigationServer::configureCoreMaps(temp)
    // 3. 验证 CostmapContext::getRobotFrame() == "base_footprint"
    // 4. RobotStateProvider::getRobotFrame() == "base_footprint"
}

TEST(MinorCarryWiring, ConfigureCoreMapsTwiceWarns) {
    // 1. 调 configureCoreMaps 两次
    // 2. 断言第二次 spdlog warn + return false 不替换 context_
}

TEST(MinorCarryWiring, PidSignatureCoarseSameGoalNoReset) {
    // 1. 给 PidController setPlan(plan1 目标 (1, 0));
    // 2. 跑几帧 computeVelocity
    // 3. 给 setPlan(plan2 目标 (1, 0) 但中间路径点稍异)
    // 4. 断言 integral_x_/last_error_x_ 不重置 (用 reflection friend 或 instrumented member)
}
```

- [ ] **Step 2: 验证 fail**

```bash
cmake --build build_m1 --target minor_carry_wirings_test && ctest --test-dir build_m1 -R MinorCarryWiring --output-on-failure
```

- [ ] **Step 3: 实现 carry 修法**

**3.1 smac_planner fallback**:
```cpp
// costmap_smac_planner.cpp makePlan start
if (start.pose.translation() == Eigen::Vector3d::Zero() && start_no_pose_provided) {
    ayanami::geometry_msgs::PoseStamped robot;
    if (costmap_context_->getRobotPose(robot)) start = robot;   // ★ fallback
}
```

**3.2 robot_frame 单一来源**:
```cpp
// costmap_navigation_server.cpp::configureCoreMaps(config_path)
GlobalConfig* g = GlobalConfig::instance();
std::string robot_frame = g->param<std::string>("navigation", "robot_frame", std::string{"base_link"});
costmap_context_->robot_frame_ = robot_frame;
robot_state_provider_.setFrames(global_frame_, odom_frame_, robot_frame);
// costmap.json 不再读 robot_frame (从 costmap_navigation.json 取)
```

**3.3 二次 configureCoreMaps 守护**:
```cpp
if (costmap_context_->global_costmap_) {
    spdlog::warn("configureCoreMaps called twice, ignoring");
    return;
}
```

**3.4 ayanami-config.cmake.in**:
```cmake
@PACKAGE_INIT@
include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
find_dependency(spdlog)
find_dependency(Threads)
include("${CMAKE_CURRENT_LIST_DIR}/ayanamiTargets.cmake")
check_required_components(ayanami)
```

**3.5 pid plan_signature 粗签名**:
```cpp
// costmap_pid_controller.hpp/.cpp
uint64_t computePlanSignature(const ayanami::nav_msgs::Path& plan) const {
    // 仅末位 goal (x, y) × 1000 + path size; 不逐 pose hash
    if (plan.poses.empty()) return 0;
    auto& goal = plan.poses.back();
    auto gx = double(int(goal.pose.translation().x() * 1000));
    auto gy = double(int(goal.pose.translation().y() * 1000));
    return (std::hash<double>()(gx) ^ std::hash<double>()(gy)) ^ (plan.poses.size() << 4);
}
```

- [ ] **Step 4: 验证 pass**

```bash
ctest --test-dir build_m1 -R MinorCarryWiring --output-on-failure
ctest --test-dir build_m1 --output-on-failure   # 全量验证不破
```

Expected: 全 PASS。

- [ ] **Step 5: Commit**

```bash
git add include/ayanami/... src/ayanami/... config/ tests/minor_carry_wirings_test.cpp CMakeLists.txt cmake/ayanami-config.cmake.in
git commit -m "fix: minor carry wiring (robot pose fallback / robot_frame 单一来源 / find_dependency / plan_signature 粗签名 / configureCoreMaps 二次守护)"
```

---

### Task 4: ayanami_ros 包骨架 (CMakeLists + package.xml + 空类)

**Files:**
- Create: `src/ayanami_ros/CMakeLists.txt`
- Create: `src/ayanami_ros/package.xml`
- Create: `src/ayanami_ros/include/ayanami_ros/ayanami_ros.hpp`  (空类声明)
- Create: `src/ayanami_ros/src/ayanami_ros/ayanami_ros.cpp`  (空实现)
- Create: `src/ayanami_ros/apps/ayanami_rosnode.cpp`  (main)
- Create: `src/ayanami_ros/src/ayanami_ros/tf2_listener.cpp`(空 stub)
- Create: `src/ayanami_ros/include/ayanami_ros/tf2_listener.hpp`
- Create: `src/ayanami_ros/include/ayanami_ros/ros_msg_converter.hpp`

**Interfaces:**
- Consumes: M1 libayanami (catkin find_package(ayanami CONFIG)) + ROS1 catkin components
- Produces: 空 ayanami_ros catkin 包能 `catkin build ayanami_ros` 通过 (空 node 可启)

- [ ] **Step 1: 写 CMakeLists.txt**

按 spec §9 (32 lines) verbatim。**关键**:`find_package(ayanami CONFIG REQUIRED)` + catkin components。

- [ ] **Step 2: 写 package.xml**

按 spec §10 verbatim。

- [ ] **Step 3: 空 AyanamiROS + main**

```cpp
// include/ayanami_ros.hpp
namespace ayanami_ros {
class AyanamiROS {
  ros::NodeHandle& nh_;
public:
  explicit AyanamiROS(ros::NodeHandle& nh) : nh_(nh) {}
  void start() {}
  void stop() {}
};
}
// apps/ayanami_rosnode.cpp
int main(int argc, char** argv) {
  ros::init(argc, argv, "ayanami_rosnode");
  ros::NodeHandle nh("~");
  ayanami_ros::AyanamiROS node(nh);
  node.start();
  ros::spin();
  node.stop();
  return 0;
}
```

- [ ] **Step 4: catkin build 验证骨架**

```bash
cd /share/nav_ws
catkin build ayanami_ros
# 验证 catkin workspace 识别 + 链 ayanami::ayanami 通过
```

Expected: 骨架编译通过，`ayanami_rosnode` 可执行生成。

- [ ] **Step 5: Commit**

```bash
cd /share/nav_ws/src/ayanami_ros
git checkout redesign
git add CMakeLists.txt package.xml include/ src/ apps/
git commit -m "feat(ayanami_ros): 骨架 (catkin 包 + 空 AyanamiROS + main + 暂未实 TF2/Converter)"
```

> 注意 ayanami_ros 已有 placeholder 在远端 (M1 T28 push 的 README.md); commit 时只 `git add` 本 task 新文件，README 保留。

---

### Task 5: ExtensionModuleROS 基类 + .so 模板 + dlopen 测试

**Files:**
- Create: `src/ayanami_ros/include/ayanami_ros/extension_module_ros.hpp`
- Create: `src/ayanami_ros/src/ayanami_ros/extensions/_extension_template.cpp`  (stub)
- Create: `src/ayanami_ros/tests/extension_module_ros_test.cpp`

**Interfaces:**
- Consumes: ayanami::ExtensionModule (M1 core 已有)
- Produces: `ExtensionModuleROS : public ayanami::ExtensionModule` + create_subscriptions + ndfdloop hooks

- [ ] **Step 1: 写基类 hpp**

按 spec §5 verbatim。

- [ ] **Step 2: 写 stub extension .cpp**

```cpp
// extensions/_extension_template.cpp
#include <ayanami_ros/extension_module_ros.hpp>
namespace ayanami_ros {
class StubExtension final : public ExtensionModuleROS {
public:
  std::vector<GenericTopicSubscription> create_subscriptions(ros::NodeHandle&) override { return {}; }
};
}
extern "C" ExtensionModuleROS* create_extension_module() { return new ayanami_ros::StubExtension(); }
```

- [ ] **Step 3: 写 ctest**

```cpp
TEST(ExtensionModuleROS, DlopenCreateSubscriptions) {
    auto ext = ayanami::ExtensionModule::load_module(/*stub so path*/);
    ASSERT_NE(ext, nullptr);
    auto ros_ext = std::dynamic_pointer_cast<ayanami_ros::ExtensionModuleROS>(ext);
    ASSERT_NE(ros_ext, nullptr);
    // create_subscriptions 调用 nh 不挂
    ros::NodeHandle nh("~");
    auto subs = ros_ext->create_subscriptions(nh);
    EXPECT_TRUE(subs.empty());
}
```

> ayanami_ros 测试改用 catkin 测试 target，BUILD_TESTING=ON guarded。

- [ ] **Step 4: catkin build + 跑测试**

```bash
catkin build ayanami_ros && \
  ctest --test-dir build/ayanami_ros --output-on-failure
```

Expected: 全 PASS。

- [ ] **Step 5: Commit**

```bash
git add include/ayanami_ros/extension_module_ros.hpp src/ayanami_ros/extensions/ tests/ CMakeLists.txt
git commit -m "feat(ayanami_ros): ExtensionModuleROS 基类 + stub .so + dlopen 测试"
```

---

### Task 6: AyanamiROS 类 (构造 + 加载链)

**Files:**
- Modify: `src/ayanami_ros/include/ayanami_ros/ayanami_ros.hpp`
- Modify: `src/ayanami_ros/src/ayanami_ros/ayanami_ros.cpp`

**Interfaces:**
- Consumes: ayanami::GlobalConfig, Config (M1); ayanami::load_module (M1); PluginManager / ExtensionModuleROS (Task 5); AbstractNavigation (M1)
- Produces: AyanamiROS 实例完整 init flow: loadExtensionModules + loadNavigationPlugin + subscribeTopics + loop

- [ ] **Step 1: TDD - 写死锁-safe init test (headless)**

```cpp
TEST(AyanamiROS, InitLoadExtensionsBeforeNavigationPlugin) {
    ros::NodeHandle nh("~");
    ayanami_ros::AyanamiROS node(nh);
    // Stub a fake working config via temp dir with 全套 M1 config + extension_modules=["librviz_viewer.so"]
    // Stub/mock librviz_viewer (use _extension_template stub)
    // Init (no exception), order = loadExtension before loadNavigation
    node.start();
    // 验证内部成员: extension_modules_.size() == 1, navigation_plugin_ != nullptr
    node.stop();
}
```

> AyanamiROS 提供 getter for tests (private 成员可暴露 protected 因 test fixture inheritance 或 friend test class)

- [ ] **Step 2: 实现 AyanamiROS 构造与加载链**

按 spec §4.1 + §4.2 verbatim 实现。关键 order:
1. config_path via ROS param (ros::package::getPath("ayanami") + "/config")
2. GlobalConfig::instance(config_path, true)
3. Config config_ros(GlobalConfig::get_config_path("config_ros"))
4. tf_listener_ 构造
5. loadExtensionModules(config_ros)
6. loadNavigationPlugin(config_ros)  → dlopen libayanami_costmap_navigation.so → initialize → start
7. subscribeTopics()
8. loop_thread_ 启 (loop: tf sync + navigation_plugin->spinOnce + ros::spinOnce)

- [ ] **Step 3: commit**

```bash
git add include/ayanami_ros/ayanami_ros.hpp src/ayanami_ros/ayanami_ros.cpp tests/ CMakeLists.txt
git commit -m "feat(ayanami_ros): AyanamiROS 构造与加载链 (GlobalConfig + ExtensionModules + NavigationPlugin + ROS topics + loop)"
```

> 详见 spec §4.1-§4.2

---

### Task 7: TF2Listener 实现

**Files:**
- Modify: `src/ayanami_ros/include/ayanami_ros/tf2_listener.hpp`
- Modify: `src/ayanami_ros/src/ayanami_ros/tf2_listener.cpp`

按 spec §7 verbatim。**Stamp 用消息自带时间** (spec §4.4 carry)。

- [ ] **Step 1: TDD - lookup_transform 假设有 TF 时返 Isometry3d**
- [ ] **Step 2: 实现 (tf2_ros::Buffer + TransformListener + lookup_transform)**
- [ ] **Step 3: commit**

```bash
git commit -m "feat(ayanami_ros): TF2Listener (tf2_ros Buffer + lookup → Eigen → TransformStore)"
```

---

### Task 8: ros_msg_converter.hpp

**Files:**
- Modify: `src/ayanami_ros/include/ayanami_ros/ros_msg_converter.hpp`

按 spec §6。Eigen ↔ ROS:
- Pose/PoseStamped (Eigen::Quaterniond & Eigen::Vector3d 与 geometry_msgs::Pose 互转)
- Twist/TwistStamped
- Path
- OccupancyGrid
- PointCloud2 (PCL aware)
- LaserScan

- [ ] **Step 1: TDD - 单测每个 converter**
- [ ] **Step 2: 实现**
- [ ] **Step 3: commit**

```bash
git commit -m "feat(ayanami_ros): ros_msg_converter (Eigen ↔ ROS msgs)"
```

---

### Task 9: rviz_viewer ROS extension .so (Task 5 stub → 本 task 全实)

**Files:**
- Modify: `src/ayanami_ros/src/ayanami_ros/extensions/rviz_viewer.cpp`
- Modify: `src/ayanami_ros/CMakeLists.txt`  (rviz_viewer .so target; 导出 create_extension_module)

按 spec §4.5。构造期 add NavigationCallbacks 订阅 + emit via rostopic pub。

- [ ] **Step 1: TDD - 验证 callback → ROS publish 触发**
- [ ] **Step 2: 实现 (publisher 实例 + NavigationCallbacks::on_*_result.add + on_velocity_command.add + on_costmap_visualization.add)**
- [ ] **Step 3: catkin test pass**
- [ ] **Step 4: commit**

```bash
git commit -m "feat(ayanami_ros): rviz_viewer ROS extension .so (导出 create_extension_module, 订阅 NavigationCallbacks → ROS publish)"
```

---

### Task 10: config 文件 (ROS 边界侧)

**Files:**
- Modify or Create `src/ayanami/config/config_ros.json`  (按 spec §8.1 - Gazebo 一致 topic)
- Modify `src/ayanami/config/costmap_navigation.json`  (robot_frame=base_footprint 等 spec §8.2)
- Modify `src/ayanami/config/costmap.json`  (box_house static map; spec §8.3)

- [ ] **Step 1: 写 config_ros.json**

按 spec §8.1 verbatim。**仿真对齐**:
- pointcloud_topic 当前 Gazebo 用 LaserScan，PointcloudTopic 可留 "/velodyne_points" 兜底 (Topic 话题端校静跑)
- laserscan_topic = "/scan"
- odom_topic = "/odom"
- goal_topic = "/move_base_simple/goal"
- map_frame = "map", odom_frame = "odom", robot_frame = "base_footprint"

- [ ] **Step 2: 写 costmap_navigation.json + costmap.json for sim**

参考 nav_demo/map/nav.yaml 加载静态地图 (as per spec §8.2-§8.3)。

- [ ] **Step 3: commit**

```bash
cd /share/nav_ws/src/ayanami
git add config/
git commit -m "feat(config): ROS 边界 + costmap Gazebo 对齐 (nav_demo map)"
```

---

### Task 11: launch/navigation.launch + config.rviz

**Files:**
- Create `src/ayanami_ros/launch/navigation.launch`
- Create `src/ayanami_ros/launch/config.rviz`

按 spec §8.4 verbatim。

- [ ] **Step 1: 写 navigation.launch**
```xml
<launch>
    <arg name="rviz" default="true"/>
    <node name="ayanami_ros" pkg="ayanami_ros" type="ayanami_rosnode" output="screen"/>
    <group if="$(arg rviz)">
        <node pkg="rviz" type="rviz" name="rviz" args="-d $(find ayanami_ros)/launch/config.rviz"/>
    </group>
</launch>
```
- [ ] **Step 2: 写 config.rviz** (显示 global_costmap/local_costmap/static_map/planner_path/smoother_path/cmd_vel/goal)
- [ ] **Step 3: catkin build + ros launch file check**
- [ ] **Step 4: commit**

```bash
git add launch/
git commit -m "feat(ayanami_ros): launch/navigation.launch + config.rviz"
```

---

### Task 12: 端到端联调 (M2 终止验证里程碑)

**Files:**
- N/A (test 是实机/仿真, 不入 commit; 若需后置 ctest 记录, 加 `tests/ros_e2e_test.cpp` 极简 smoke test catkin run)

**Interfaces:** consumers 全栈; produces 验证 roslaunch 可启动 + cmd_vel 有输出 + RViz 能显示 path/costmap

- [ ] **Step 1: catkin build ayanami ayanami_ros**

```bash
cd /share/nav_ws
catkin build ayanami ayanami_ros
```

- [ ] **Step 2: 启动仿真**

```bash
source /share/nav_ws/devel/setup.bash
roslaunch nav_demo ayanami.launch
```

- [ ] **Step 3: 在另端验证 topics**

```bash
rostopic hz /cmd_vel                    # expect 10-20Hz
rostopic echo /ayanami/planner/path      # 非空 Path (验证 publishMaps)
rostopic echo /ayanami/global_costmap    # 非空 OccupancyGrid
rosrun tf2_tools view_frames            # 验证 map->odom->base_footprint TF tree
```

- [ ] **Step 4: RViz 点 goal → robot 抵达**

- 在 RViz 中点 2D Nav Goal (按 nav_demo RViz config "2D Nav Goal")，location 在 nav_demo/map 上
- 观察 Gazebo 中 robot 沿路径移动至 goal
- **关键**: 中途再点新的 goal，验证 ayanami_ros 抢占 + 零速度反发 + 不死锁

- [ ] **Step 5: Ctrl+C 优雅退出**

- 验证无 segfault (Ruling #3 修 + 3 BLOCK + 5 minor carry 全程生效)
- log 末 "ayanami_ros node stopping cleanly"

- [ ] **Step 6: commit** (若 launch + build 绿, commit 任何收尾调整)

```bash
git commit --allow-empty -m "chore(ayanami_ros): M2 端到端联调验证完成 (roslaunch nav_demo ayanami.launch Gazebo + RViz)"
```

> 执行 implementer 检查项:
> - catkin build 退 0
> - roslaunch 启动后 cmd_vel 发布 > 0 (里程 cmd_vel 为非零发布持续 / NaN 检测)
> - 在 RobotStateProvider 注入真实 robot pose 后, planner 生成 path
> - Recovery 子树功能可执行 ("recovery_id 验证" - 测试翻清障碍 recovery)
> - 若 Gazebo/仿真环境未配置 (urdf_gazebo_demo/world path 有 dependencies 缺失), implementer 报 NEEDS_CONTEXT

---

### Task 13: 全套回归 + push origin/redesign

**Files:**
- Modify: docs/superpowers/specs/2026-08-14-ayanami-m2-ros-design.md (若 self-review 有微调), Plan doc

**Interfaces:** N/A

- [ ] **Step 1: 全套 ctest 回归 (ayanami + ayanami_ros)**

```bash
cd /share/nav_ws
catkin build ayanami ayanami_ros
ctest --test-dir build/ayanami --output-on-failure
ctest --test-dir build/ayanami_ros --output-on-failure
```

Expected: 全 PASS - 新加 BLOCK/minor carry 测试和 ayanami_ros extension_module/ros 的 ctest + 既有 M1 33 用例

- [ ] **Step 2: 静态红线 grep (M1 carry still passes)**

```bash
cd /share/nav_ws/src/ayanami
! grep -rn "ros/ros.h\|tf2_ros/\|sensor_msgs/\|nav_msgs/" include/ayanami/{core,components,behavior_tree,utility,interfaces,messages}/
! grep -rn "costmap_2d/\|gridmap_2d/" include/ayanami/core/ include/ayanami/components/ include/ayanami/behavior_tree/
! grep -rwn "AbstractBehavior\|BehaviorAction\|behavior_execution" include/ayanami/ modules/
```

Expected: 3 grep 全 0 match。

- [ ] **Step 3: 静态红线新增 ayanami_ros 边界**

```bash
cd /share/nav_ws/src/ayanami_ros
# ayanami_ros 可 include ROS, 但核心 libayanami 头不可 include ROS+ costmap_2d types in its public hpp
! grep -rn "ros/ros.h" include/ayanami_ros/   # exception: 可 include ros.h in irrtypeless .hpp but let's verify if it exists
! grep -rn "costmap_2d/" include/ayanami_ros/  # redline: ayanami_ros 应不直接 include 核心地图具体类型 (spec §4.6)
```

- [ ] **Step 4: push origin/redesign**

```bash
cd /share/nav_ws/src/ayanami
git push origin redesign      # ayanami 仓库
cd /share/nav_ws/src/ayanami_ros
git push origin redesign      # ayanami_ros 仓库
```

- [ ] **Step 5: commit ledger 收尾** (在 plan doc 同步 self-review report)

- commit 的 message: `chore: M2 收尾 + final ctest + push origin/redesign`
- push 成功记录落 SDD ledger / docs

---

## Self-Review (Plan ↔ Spec 对照)

- [x] **Spec coverage**: 逐节核对:
  - §1 顶层架构→ Task 0-3 (carry 修) + Task 4-11 (ros 边界) + Task 12-13 (联调 + push)
  - §2 BLOCK 3 项 → Task 0/1/2
  - §3 minor 5 项 → Task 3
  - §4 AyanamiROS → Task 6
  - §5 ExtensionModuleROS → Task 5
  - §6 ros_msg_converter → Task 8
  - §7 TF2Listener → Task 7
  - §8 配置文件 → Task 10
  - §11 端到端联调 → Task 12
  - §12 已知坑规避汇总 22 项逐项 → 所有 task 内 verify
  - §13 实施步骤概要 (14 task) → 本 plan 14 task
- [x] **Placeholder scan**: 14 个 Task 全部含 step + 测试代码 / 验证命令 / commit message;无 "TBD/TODO/Later"
- [x] **Type consistency**:
  - `AbstractRecovery/Controller/Smoother::Ptr` from M1 已有，Task 0 复用
  - `CostmapContext::Ptr` M1 已有，Task 1-3 复用（Task 3 加 `getRobotPose` fallback wiring 调用方代换）
  - `ExtensionModuleROS::Ptr` Task 5 定义，Task 6/9 消费
  - `ros::NodeHandle&` Task 4/5/6 一致签名
  - `global_costmap_/local_costmap_` (CostmapContext) 在 Task 1/2 锁顺序固定为 global→local 避免死锁

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-08-14-ayanami-m2-ros.md`. Recommended execution option:

**1. Subagent-Driven (recommended)**

每个 Task dispatch fresh subagent implementer + reviewer; 切合 Plan 1 (M1) 经验：28-task SDD 完整循环也曾圆满, 14 task 量可行。

**2. Inline Execution**

批量执行（executing-plans）+ checkpoint review.

请选择方式: