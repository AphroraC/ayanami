# 模块开发指南

## 模块机制总览

所有 dlopen 对象（算法模块、FSM 宿主、可视化扩展）都继承
`ayanami::ExtensionModule`，以**同一个零参工厂符号** `create_extension_module`
暴露。

- **加载**：`dlopen(RTLD_LAZY|RTLD_GLOBAL|RTLD_NODELETE)` + `dlsym`；句柄按
  名字去重、**从不 dlclose**，模块生命周期 = 进程生命周期。
- **接口版本**：每个族基类带 `kApiVersion`，模块把自己的编译期常量经虚函数
  `apiVersion()` 上报。RTTI 只能证明类名对得上；用旧头文件编出来的 `.so` 能
  通过 `dynamic_pointer_cast` 却在虚表布局上与本 build 不一致——版本不符即
  拒绝加载，且失败原因是独立的（与"类型不符"区分开，便于定位）。
- **LoadFailure 七值分类**（`ModuleManager::LoadFailure`）：None /
  InvalidSpec（json 缺 name/so_name）/ OpenOrSymbolFailed（dlopen 失败或缺
  工厂符号）/ TypeMismatch（非本族接口）/ ApiVersionMismatch / 
  IdentityMismatch（name/段名/so_name 不一致）/ InitRejected（族 init 钩子
  拒绝）。加载顺序是刻意的：先建对象、再类型转换、再接口版本校验、最后才
  注入参数——任何一步失败都不会去读参数。
- **名字的唯一性**：一个模块只有一个名字，且它是其它一切所键的对象——参数
  段名（`ModuleContext::name`）、家族 json 里的声明、被加载的 `.so`。
  注意 `.so` 基名**不参与**该校验：同一个 `.so` 挂多个逻辑名（各自一套
  参数）是合法的别名用法。
- **参数**：加载成功后注入，见下节。
- **失败即失败**：配置里明确写出的模块加载失败 → critical 日志 + 抛异常，
  启动即中止。静默起一个没有 planner 的栈，只会在下发目标时以莫名的
  `INVALID_INPUT` 暴露。
- **卸载能力保留但未实现**：`LoadedModule::unloadable()` /
  `ExtensionModule::canUnload()` 原样保留、当前恒为 false。真要实现卸载，
  先解决三个前置条件：(1) `load_module.cpp` 用的 `RTLD_NODELETE` 必须去掉；
  (2) 去重表按名持有引用计数，卸载时需能断言引用归零；(3) 模块对象由
  **无自定义 deleter** 的 `shared_ptr` 持有，卸载必须先把所有对象释放在
  dlopen 句柄仍存活的窗口内。

## 模块清单（modules/，11 个）

| 模块 | 族 | 依赖地图 | 职责 |
|---|---|---|---|
| `costmap_astar_planner` | planner | costmap（读者） | costmap A* 全局规划；`isPathValid` 覆写为 FusedMapView（global+local 逐点 MAX）稀疏探针——FSM 的 path validator 来源 |
| `esdfmap_minco_smoother` | smoother | esdfmap（**唯一读者**） | MINCO 轨迹优化 + ESDF 碰撞代价；整次优化持一份不可变 EsdfSnapshot（零锁） |
| `costmap_minsnap_smoother` | smoother | costmap | 最小 snap 路径平滑（走廊约束 + 碰撞修复环）；纯 costmap 栈路由使用 |
| `abstract_pid_controller` | controller | 无（**纯跟踪**） | 三轴 PID 路径跟踪（diff/omni 双模式，含五项历史控制修复；继承自 costmap_pid_controller 去掉地图壳） |
| `costmap_pid_controller` | controller | costmap | costmap 族 PID 控制器（纯 costmap 栈路由使用） |
| `costmap_mppi_controller` | controller | costmap | MPPI 控制器（4D swerve 采样空间） |
| `abstract_nothing_recovery` | recovery | 无 | **占位**恢复行为：跑完整 start/tick/finish 协议、一拍即成功；恢复编排由 FSM 预算/游标驱动并可观测，换真行为是换插件 |
| `simple_goal_checker` | goal_checker | 无 | 位置/朝向容差到达判定（容差唯一权威，经执行层下推给控制器） |
| `simple_progress_checker` | progress_checker | 无 | 无进展判定 |
| `rose_navigation_fsm` | 宿主（AbstractNavigation） | costmap + esdfmap | rose 路由宿主：CostmapNavigationServer + EsdfWriter 派生场 + 混合族模块 + FsmDriver |
| `costmap_navigation_fsm` | 宿主（AbstractNavigation） | costmap | 纯 costmap 路由宿主：CostmapNavigationServer + costmap 族模块 + FsmDriver |

宿主不是"族模块"：它实现 `AbstractNavigation`（initialize/start/stop/
setGoal/setPointcloud/setLaserScan），由 ayanami_ros 经路由 json 的
`navigation.so_name` 选择加载。

## 族接口（要实现什么）

抽象层（`abstract/core/`，零地图类型）：

```cpp
class AbstractPlanner : public virtual ExtensionModule {
  virtual PlannerResultT makePlan(const PoseStamped& start,
                                  const PoseStamped& goal,
                                  double tolerance) = 0;
  virtual bool isPathValid(const nav_msgs::Path& path) const;  // 默认=非空检查
  virtual bool cancel();          // 协作式中断，默认 false
  virtual bool setNewGoal(...);   // 可选 latest-wins seam
  virtual void activate()/deactivate()/cleanup();
};
```

| 接口 | 纯虚 | 说明 |
|---|---|---|
| AbstractPlanner | makePlan(start, goal, tolerance) | 失败立即返回 ErrorCode，**不做内部重试**（重试归 FSM 预算）；**start 永远有效**（空 start 由 PlannerExecution 经 RobotStateController 解析）。`isPathValid` 是 FSM 的路径校验探针——地图族规划器覆写为真实地图检查 |
| AbstractSmoother | smoothPath(Path&, max_duration) | **in-place** 修改传入路径；平滑产物的合法性由 FSM 的 path validator 在规划视图上判定（DegradeToRawPath 兜底） |
| AbstractController | computeVelocityCommand(pose, velocity, cmd) + setPlan(plan) | 单周期速度命令；到达判定不归模块。`setGoalTolerances` 由执行层每轮从 goal_checker 下推——模块必须把它当作**上界**：停得比验收区更远，checker 永不触发 |
| AbstractRecovery | start() / tick() / finish()（+cancel/setBehaviorParams/setCyclePeriod） | 框架固定周期驱动；tick 返回状态+optional 速度。`setCyclePeriod` 由执行层在 start 前下发节拍——模块算周期相关量时用它，**不要**去读自己配置文件里的框架参数 |
| AbstractGoalChecker | isGoalReached + getTolerances() | 到达容差的**唯一权威**：执行层把它下推给控制器 |
| AbstractProgressChecker | check | 无进展判定，由 ControllerExecution 消费 |

地图族接口只增加注入点：

```cpp
// costmap 族（costmap/core/）
class CostmapPlanner : public abstract::AbstractPlanner {
  virtual void configure(const ModuleContext& context,
                         CostmapMapContext::ConstPtr map) = 0;
};
// CostmapSmoother / CostmapController 同构

// esdfmap 族（esdfmap/core/）
class EsdfmapSmoother : public abstract::AbstractSmoother {
  virtual void configure(const ModuleContext& context,
                         ::esdfmap::EsdfMap::Ptr map) = 0;
};
```

宿主在 `initialize*Module` 虚钩子里 `dynamic_pointer_cast` 到本族接口，
非本族模块直接拒绝（spdlog error）——rose 宿主要求 planner 是 CostmapPlanner、
smoother 是 EsdfmapSmoother、controller/recovery 无地图。

## 标准模块模板

`modules/abstract_nothing_recovery/` 是最简范例：

```
modules/abstract_nothing_recovery/
├── abstract_nothing_recovery.hpp  # 类（+ 复杂模块再有 XxxParam 结构体）
├── abstract_nothing_recovery.cpp  # configureParameters + start/tick/finish
└── module.cpp                     # create_extension_module 工厂
```

参数装载时机与归属：

```cpp
void XxxModule::configureParameters(
    const ayanami::ModuleContext& context) {   // 加载成功后由 loader 调用
  // 段名 = 模块 json 的 name 字段（= ModuleContext::name）
  param.wait_duration =
      context.config.param<double>(context.name, "wait_duration",
                                   param.wait_duration);
}
```

- **工厂 `create_extension_module()` 保持零参**；参数在**模块成功加载之后**
  由 loader 通过 `ExtensionModule::configureParameters(context)` 注入，落到
  模块自己的 `XxxParam param` 成员。
- 参数**默认值写在结构体里**，加载只做覆盖——保证模块零配置可用。
- **段名 = 模块名**：loader 在加载时校验「模块 json 里存在以模块名命名的
  参数段」，缺失即拒绝加载——否则模块会一个参数都读不到而静默跑默认值。
  读不到参数的模块也要在 json 里写空段（`"abstract_nothing_recovery": {}`）
  显式声明"读 no 参数"。
- `ModuleContext{name, config}`：name 为逻辑 id，config 为该模块自己的
  模块 json（已解析）。
- **未消费键审计**：加载后 loader 比对该段「声明了但没人读」的键并告警
  （打错一个字与"有意省略"在静默默认值下无法区分）。
  `global.strict_params = true` 可把它变成启动失败。
- `setBehaviorParams`（按次覆盖）：nullopt 字段 = 保留 configure 默认值，
  **禁止解释为 0**。FSM 出厂路径下发的是空 `RecoveryBehaviorParams{}`。
- 恢复模块需要清图时由 FSM 直接调用 server 的 `executeClearCommand()`（
  绝不直接触碰 wrapper/layer。

module.cpp：

```cpp
#include "abstract_nothing_recovery.hpp"
extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::NothingRecovery();
}
```

## 接入配置

1. 新建 `config/modules/my_module.json`（family 段 + 参数段）。
2. 在路由 json（`config/routes/fsm_*.json`）的 `navigation` 段把对应
   `config_<family>` 指针指向它（planner/smoother/controller/checker 单串；
   recovery 追加到 `config_recovery` 数组——**数组顺序即恢复轮转序**）。
3. 模块的地图族归属必须与宿主匹配：rose 宿主要求 costmap 族 planner +
   esdfmap 族 smoother + 无地图 controller/recovery；costmap 宿主全
   costmap 族。族不匹配在 `initialize*Module` 钩子被拒绝（启动即失败）。
4. CMake：加入 `ayanami/CMakeLists.txt` 的 `AYANAMI_MODULES` 列表（见下）。

## CMake 集成（ayanami/CMakeLists.txt）

```cmake
# AYANAMI_MODULES 列表追加模块名，模块自带 modules/<name>/CMakeLists.txt：
ayanami_add_module(costmap_my_module
  modules/costmap_my_module/costmap_my_module.cpp
  modules/costmap_my_module/module.cpp)
# 产物：libayanami_costmap_my_module.so，链接 libayanami
```

- `-mcx16` 全树启用（shared_ptr 原子自由函数无锁路径，跨 .so 一致性要求）。
- 模块间不相互链接；只依赖 libayanami 与 packages 目标
  （`costmap::costmap` / `esdfmap::esdfmap`）。
- 每模块有缓存开关 `AYANAMI_BUILD_<NAME>`（默认 ON；
  `BUILD_COSTMAP_MODULES=OFF` 关掉全部 costmap 族）。
- 测试模块放 `tests/modules/`（`ayanami_add_test_plugin` 目标，不 install）。

## 宿主模块（AbstractNavigation）

新的导航栈宿主实现两个接口：`NavigationServerBase`（模块装载/Execution
装配）+ `AbstractNavigation`（ayanami_ros 的加载边界）。最小宿主：

```cpp
class MyNavigationFsm : public components::NavigationServerBase,
                        public abstract::AbstractNavigation {
  void initialize(const std::string& config_path) override;  // 地图 + loadModules
  void start() / stop() override;                            // 写者线程/模块 activate/FsmDriver
  void setGoal(const PoseStamped&) override;                 // fsm_->beginNavigation（邮箱）
  void setPointcloud/setLaserScan(...) override;             // 直递地图观测管线
  // initialize*Module 钩子：注入地图句柄
  // createActions()：装配三族 Execution + recovery 列表 + FsmDriver
};
```

参考实现：`modules/rose_navigation_fsm`（带 ESDF 派生）与
`modules/costmap_navigation_fsm`（纯 costmap）。两者对 FsmDriver 完全同构。

## planner tolerance 语义（一致性说明）

`GetPathGoal.tolerance` 是 goal 级字段，但 FSM 侧固定下发
`k_plan_tolerance = 0.1`，而 astar planner **不消费它**（`makePlan` 形参
`(void)tolerance`）：邻域降级的半径由模块 config 自持
（`costmap_astar_planner.json::tolerance`，出厂 0.35）。这是有意的家族
约定——goal 级字段若未来透传，需要 planner 族 kApiVersion 再次 bump 并
同步模块；在那之前，调整容差半径请改模块 json 而不是期望 goal 字段生效。
