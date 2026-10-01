# 模块开发指南

## 模块机制总览

所有 dlopen 对象（算法模块、宿主、BT 扩展、可视化扩展）都继承
`ayanami::ExtensionModule`，以**同一个零参工厂符号** `create_extension_module`
暴露。

- **加载**：`dlopen(RTLD_LAZY|RTLD_GLOBAL|RTLD_NODELETE)` + `dlsym`；句柄按名字去重、
  **从不 dlclose**，模块生命周期 = 进程生命周期。
- **接口版本**：每个族基类带 `kApiVersion`，模块把自己的编译期常量经虚函数
  `apiVersion()` 上报。RTTI 只能证明类名对得上；用旧头文件编出来的 `.so` 能通过
  `dynamic_pointer_cast` 却在虚表布局上与本 build 不一致——版本不符即拒绝加载，
  且失败原因是独立的（与"类型不符"区分开，便于定位）。
- **名字的唯一性**：一个模块只有一个名字，且它是其它一切所键的对象——参数段名
  （`ModuleContext::name`）、家族 json 里的声明、被加载的 `.so`。不一致即拒绝加载。
  注意 `.so` 基名**不参与**该校验：同一个 `.so` 挂多个逻辑名（各自一套参数）是
  合法的别名用法。
- **参数**：加载成功后注入，见下节。
- **失败即失败**：配置里明确写出的模块加载失败 → critical 日志 + 抛异常，启动即
  中止。静默起一个没有 planner 的栈，只会在下发目标时以莫名的 `INVALID_INPUT`
  暴露。
- **卸载能力保留但未实现**：`LoadedModule::unloadable()` / `ExtensionModule::canUnload()`
  原样保留、当前恒为 false。真要实现卸载，先解决三个前置条件：
  (1) `load_module.cpp` 用的 `RTLD_NODELETE` 必须去掉；
  (2) 去重表按名持有引用计数，卸载时需能断言引用归零；
  (3) 模块对象由**无自定义 deleter** 的 `shared_ptr` 持有，卸载必须先把所有对象
  释放在 dlopen 句柄仍存活的窗口内。

## 族接口（要实现什么）

抽象层（`abstract/core/`，零地图类型）：

```cpp
class AbstractPlanner : public virtual ExtensionModule {
  virtual PlannerResultT makePlan(const PoseStamped& start,
                                  const PoseStamped& goal,
                                  double tolerance) = 0;
  virtual bool cancel();          // 协作式中断，默认 false
  virtual bool setNewGoal(...);   // 可选 latest-wins seam
  virtual void activate()/deactivate()/cleanup();
};
```

| 接口 | 纯虚 | 说明 |
|---|---|---|
| AbstractPlanner | makePlan(start, goal, tolerance) | 失败立即返回 ErrorCode，**不做内部重试**（重试归 BT）；**start 永远有效**（空 start 由 PlannerExecution 经 RobotStateController 解析，模块不自解释哨兵） |
| AbstractSmoother | smoothPath(Path&, max_duration) | **in-place** 修改传入路径；碰撞检查只读 local（readLocal），路径帧→局部帧变换每次调用解析一次。无 check_collision 参数：平滑产物的合法性由 tree 的 `IsPathValid` 在叠加图上判定（平滑器只看得到 local 图） |
| AbstractController | computeVelocityCommand(pose, velocity) + setPlan(plan) + setGoalTolerances(tol) | 单周期速度命令；到达判定不归模块。`setGoalTolerances` 由执行层每轮从 goal_checker 下推——模块必须把它当作**上界**：停得比验收区更远，checker 永不触发（超时失败） |
| AbstractRecovery | start() / tick() / finish()（+cancel/setBehaviorParams/setCyclePeriod） | 框架固定周期驱动；tick 返回状态+optional 速度。`setCyclePeriod` 由执行层在 start 前下发节拍——模块算周期相关量时用它，**不要**去读自己配置文件里的框架参数（跨段读取已消除） |
| AbstractGoalChecker | getTolerances() | 到达容差的**唯一权威**：执行层把它下推给控制器，critic 也经此拿到（不再有人去打开 checker 的配置文件） |
| AbstractGoalChecker / AbstractProgressChecker | configure/reset/isGoalReached·check | 由 ControllerExecution 消费 |

地图族接口（costmap 族，`costmap/core/`）只增加注入点：

```cpp
class CostmapPlanner : public abstract::AbstractPlanner {
  virtual void configure(const ModuleContext& context,
                         CostmapMapContext::ConstPtr map) = 0;
};
// CostmapSmoother / CostmapController / CostmapRecovery 同构
```

族 server（如 `CostmapNavigationServer::initializePlannerModule`）会
`dynamic_pointer_cast<CostmapPlanner>`，非本族模块直接拒绝（spdlog error）。

## 标准模块模板

`modules/costmap_wait_recovery/` 是最简范例：

```
modules/costmap_wait_recovery/
├── costmap_wait_recovery.hpp   # 类 + XxxParam 结构体
├── costmap_wait_recovery.cpp   # configure（参数装载）+ start/tick/finish
└── module.cpp                  # create_extension_module 工厂
```

hpp 模式：

```cpp
class CostmapWaitRecovery final : public costmap::CostmapRecovery {
 public:
  struct CostmapWaitRecoveryParam {          // ① 参数结构体
    double wait_duration = 1.0;              // 默认值即兜底
    double sim_cycle_dt = 0.01;
  };

  void configure(const ModuleContext& context,
                 CostmapMapContext::ConstPtr map) override {
    param = ...;  // ② 从 context.config 逐键覆盖默认值
    map_context = std::move(map);            // ③ 只存 ConstPtr
    port = std::dynamic_pointer_cast<CostmapCommandPort>(map->commandPort());
  }
  RecoveryResultT tick() override;           // ④ 每周期一拍，返回状态+optional 速度
  void finish() override;                    // 清理

 private:
  CostmapWaitRecoveryParam param;            // ⑤ 模块内参数统一保存在 param
  CostmapMapContext::ConstPtr map_context;
  CostmapCommandPort::Ptr command_port;
};
```

参数装载时机与归属（**改过一轮**）：

```cpp
void CostmapWaitRecovery::configureParameters(
    const ayanami::ModuleContext& context) {   // 加载成功后由 loader 调用
  // 段名 = 模块 json 的 name 字段（= ModuleContext::name）
  param.wait_duration =
      context.config.param<double>(context.name, "wait_duration",
                                   param.wait_duration);
}

void CostmapWaitRecovery::configure(const ModuleContext& context,
                                    CostmapMapContext::ConstPtr map) {
  configureParameters(context);   // 直接构造对象的调用者（测试/宿主）也走这里
  map_context = std::move(map);   // configure 只做地图绑定
}
```

- **工厂 `create_extension_module()` 保持零参**；参数在**模块成功加载之后**由
  loader 通过 `ExtensionModule::configureParameters(context)` 注入，落到模块自己的
  `XxxParam param` 成员。顺序是刻意的：先建对象、再类型转换、再接口版本校验，
  任何一步失败都不会去读参数。
- 参数**默认值写在结构体里**，加载只做覆盖——保证模块零配置可用。
- **段名 = 模块名**：`context.name`。loader 在加载时校验「模块 json 里存在以模块名
  命名的参数段」，缺失即拒绝加载——否则模块会一个参数都读不到而静默跑默认值。
- `ModuleContext{name, config}`：name 为逻辑 id，config 为该模块自己的
  模块 json（已解析）。
- **未消费键审计**：加载后 loader 比对该段「声明了但没人读」的键并告警（打错一个字
  与"有意省略"在静默默认值下无法区分）。`global.strict_params = true` 可把它变成
  启动失败——先审计、后严格的两步落地方式。
- 恢复模块需要写图时经 `map->commandPort()` 提交
  `ClearCostmapCommand`，绝不直接触碰 wrapper/layer。
- `setBehaviorParams`（按次覆盖）：nullopt 字段 = 保留 configure 默认值，
  **禁止解释为 0**。

module.cpp：

```cpp
#include "costmap_wait_recovery.hpp"
extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new ayanami::modules::CostmapWaitRecovery();
}
```

## 接入配置

1. 新建 `config/modules/my_module.json`（family 段 + 参数段）。
2. 在 `costmap_navigation.json::navigation` 增加文件指针
   （planner/smoother/controller/goal_checker/progress_checker 单串或数组；
   recovery 追加到 `config_recovery` 数组）。**数组 = 多模块**（nav2 语义）：
   每个模块一个 action，BT 端口 `planner_id/controller_id/smoother_id`（可经
   PlannerSelector/ControllerSelector/SmootherSelector 写黑板热切换）按 id
   选择，空 id = 第一个配置的模块。生产配置仍为每族单模块。
3. recovery 类模块补充 `role` 字段；**BT 扩展**（不是 pilot）把它映射为黑板键
   `<role>_id`。角色若无人声明，键为空串——**建树期校验会拒绝**该树
   （否则错误只在恢复分支真的被派发时出现，也就是机器人已经卡住的那一刻）
   `spin_id/wait_id/backup_id/clear_costmap_id`（默认树的恢复引用）。

## CMake 集成（ayanami/CMakeLists.txt）

```cmake
ayanami_add_module(costmap_my_module
  modules/costmap_my_module/costmap_my_module.cpp
  modules/costmap_my_module/module.cpp)
# 产物：libayanami_costmap_my_module.so，链接 libayanami，install 到 lib
```

- `-mcx16` 全树启用（shared_ptr 原子自由函数无锁路径，跨 .so 一致性要求）。
- 模块间不相互链接；只依赖 libayanami 与 packages 目标。
- 测试模块放 `tests/modules/`（`tests/modules/` 目标单独声明，不 install）。

## 行为树扩展模块

继承 `BehaviorTreeExtension`（`behavior_tree/extension.hpp`）：

```cpp
class MyTreeExtension final : public BehaviorTreeExtension {
  std::string name() const override;
  void registerNodes(BT::BehaviorTreeFactory&) const override;   // 节点注册
  std::vector<std::string> registerTrees(BT::BehaviorTreeFactory&) const; // 注册树 XML
  std::string defaultTreeId() const override;
};
```

参考实现：`modules/navigation_behavior_tree`（生产默认树的内嵌单一来源 +
`registerNavigationBehaviorTreeNodes`）。扩展经由
`navigation.behavior_tree_extension` 配置加载，可叠加多个。

## planner tolerance 语义（一致性说明）

`GetPathGoal.tolerance` 是 goal 级字段，但当前两个 astar planner 均**不消费它**
（`makePlan` 形参 `(void)tolerance`）：邻域降级的半径由各模块 config 自持
（`costmap_astar_planner.json::tolerance` / `rosemap_astar_planner.json::tolerance`，
costmap 路由现配 0.25）。这是有意的家族约定——goal 级字段若未来透传，需要
planner 族 kApiVersion 再次 bump 并同步两模块；在那之前，调整容差半径请改模块
json 而不是期望 goal 字段生效。
