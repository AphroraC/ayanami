# 插件开发指南

## 插件机制总览

所有 dlopen 对象（算法插件、宿主、BT 扩展、可视化扩展）都继承
`ExtensionModule` 并导出同名 C 工厂：

```cpp
extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new my_ns::MyPlugin();
}
```

- 加载：`dlopen(RTLD_LAZY | RTLD_GLOBAL | RTLD_NODELETE)`，dlsym
  `create_extension_module`，返回不带自定义 deleter 的 shared_ptr
  （**句柄从不 dlclose，插件生命周期 = 进程生命周期**）。
- 进程级模块登记表：同 so_name 去重 + 引用计数
  （`moduleHandleRefs(so_name)` / `loadedModuleNames()` 观测接口）。
  卸载逻辑本身**未实现**；`LoadedModule::unloadable()` 与
  `ExtensionModule::can_unload()` 是预留闸门。卸载前置条件
  （见 `load_module.hpp` 注释）：全部 action 取消并 join → 全部插件实例
  析构（clearPlugins）→ 模块引用归零 → 闸门放行。
- 无 ABI version 检查：类型安全由 `dynamic_pointer_cast` 保证，cast 失败
  即拒绝加载。**因此每次改抽象接口头文件后必须全量重编所有插件**（含
  测试二进制——catkin 的增量判断在这种跨包头文件变更上不可靠）。

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
| AbstractPlanner | makePlan(start, goal, tolerance) | 失败立即返回 ErrorCode，**不做内部重试**（重试归 BT）；**start 永远有效**（空 start 由 PlannerExecution 经 RobotStateController 解析，插件不自解释哨兵） |
| AbstractSmoother | smoothPath(Path&, max_duration, check_collision) | **in-place** 修改传入路径；碰撞检查只读 local（lockLocal），路径帧→局部帧变换每次调用解析一次 |
| AbstractController | computeVelocityCommand(pose, velocity) + setPlan(plan) | 单周期速度命令；到达判定不归插件 |
| AbstractRecovery | start() / tick() / finish()（+cancel/setBehaviorParams） | 框架固定周期驱动；tick 返回状态+optional 速度 |
| AbstractGoalChecker / AbstractProgressChecker | configure/reset/isGoalReached·check | 由 ControllerExecution 消费 |

地图族接口（costmap 族，`costmap/core/`）只增加注入点：

```cpp
class CostmapPlanner : public abstract::AbstractPlanner {
  virtual void configure(const PluginContext& context,
                         CostmapMapContext::ConstPtr map) = 0;
};
// CostmapSmoother / CostmapController / CostmapRecovery 同构
```

族 server（如 `CostmapNavigationServer::initializePlannerPlugin`）会
`dynamic_pointer_cast<CostmapPlanner>`，非本族插件直接拒绝（spdlog error）。

## 标准插件模板

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

  void configure(const PluginContext& context,
                 CostmapMapContext::ConstPtr map) override {
    param = ...;  // ② 从 context.config 逐键覆盖默认值
    map_context = std::move(map);            // ③ 只存 ConstPtr
    port = std::dynamic_pointer_cast<CostmapCommandPort>(map->commandPort());
  }
  RecoveryResultT tick() override;           // ④ 每周期一拍，返回状态+optional 速度
  void finish() override;                    // 清理

 private:
  CostmapWaitRecoveryParam param;            // ⑤ 插件内参数统一保存在 param
  CostmapMapContext::ConstPtr map_context;
  CostmapCommandPort::Ptr command_port;
};
```

cpp 参数装载（family 段 = 参数段名）：

```cpp
param.wait_duration =
    context.config.param<double>("recovery", "wait_duration", param.wait_duration);
```

- 参数**默认值写在结构体里**，`configure` 只做覆盖——保证插件零配置可用。
- `PluginContext{name, config}`：name 为逻辑 id，config 为该插件自己的
  模块 json（已解析）。
- 恢复插件需要写图时经 `map->commandPort()` 提交
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

1. 新建 `config/modules/my_plugin.json`（family 段 + 参数段）。
2. 在 `costmap_navigation.json::navigation` 增加文件指针
   （planner/smoother/controller/goal_checker/progress_checker 单串或数组；
   recovery 追加到 `config_recovery` 数组）。**数组 = 多插件**（nav2 语义）：
   每个插件一个 action，BT 端口 `planner_id/controller_id/smoother_id`（可经
   PlannerSelector/ControllerSelector/SmootherSelector 写黑板热切换）按 id
   选择，空 id = 第一个配置的插件。生产配置仍为每族单插件。
3. recovery 类插件补充 `role` 字段，pilot 会把它映射到黑板
   `spin_id/wait_id/backup_id/clear_costmap_id`（默认树的恢复引用）。

## CMake 集成（ayanami/CMakeLists.txt）

```cmake
ayanami_add_module(costmap_my_plugin
  modules/costmap_my_plugin/costmap_my_plugin.cpp
  modules/costmap_my_plugin/module.cpp)
# 产物：libayanami_costmap_my_plugin.so，链接 libayanami，install 到 lib
```

- `-mcx16` 全树启用（shared_ptr 原子自由函数无锁路径，跨 .so 一致性要求）。
- 模块间不相互链接；只依赖 libayanami 与 packages 目标。
- 测试插件放 `tests/plugins/`（`tests/plugins/` 目标单独声明，不 install）。

## 行为树扩展插件

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
