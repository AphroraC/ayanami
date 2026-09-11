# 总体架构

## 分层总览

ayanami 采用"算法核心与中间件彻底分离"的两包结构：

```
┌──────────────────────────────────────────────────────────────┐
│ ayanami_ros（唯一的 ROS1 边界）                                │
│  AyanamiROS: setTransform / setOdometry / submitSensor /      │
│              submitGoal / drainPendingGoal / start / stop      │
│  TF2Listener / ros_msg_converter / rviz_viewer(扩展)           │
└──────────────┬───────────────────────────────────────────────┘
               │ dlopen(libayanami_costmap_navigation.so)
               │      + AbstractNavigation 接口
┌──────────────▼───────────────────────────────────────────────┐
│ ayanami（纯 C++，零 ROS 头文件）                                │
│                                                               │
│  宿主模块层  CostmapNavigation（modules/costmap_navigation）    │
│    ├─ CostmapNavigationServer（地图族 server，唯一写者）        │
│    └─ BehaviorTreePilot（工厂常驻；每尝试一棵新树）             │
│                                                               │
│  行为树层    ComputePathToPose / SmoothPath / FollowPath /     │
│              Recovery action 节点 + PipelineSequence /         │
│              RecoveryNode / RateController /                   │
│              RecoverySequence(轮换) / 条件节点                  │
│                                                               │
│  执行层      NavigationServerBase(NavigationRuntime)           │
│              ActionBase 并发槽 ─► ExecutionBase 状态机线程      │
│              （Planner/Smoother/Controller/Recovery 四族）      │
│                                                               │
│  模块接口层  AbstractPlanner/Smoother/Controller/Recovery/     │
│              GoalChecker/ProgressChecker（零地图类型）          │
│              + 地图族接口 CostmapPlanner/Smoother/...          │
│                                                               │
│  地图层      AbstractMapContext ◄─ CostmapMapContext           │
│              (global/local CostmapWrapper + LayeredCostmap)    │
│                                                               │
│  基础设施    Config/GlobalConfig、NavigationCallbacks 静态槽、  │
│              RobotStateController(TF 桥+里程计)、              │
│              ModuleManager、WorkerPool(已移除，见下)             │
└───────────────────────────────────────────────────────────────┘
```

## 两个包、一条红线

- `src/ayanami`：**零 ROS 头文件**（红线）。消息全部自研
  （`messages/`，Eigen 存储），TF 按需经注入的 tf2 源查询，行为树使用
  vendored BehaviorTree.CPP v4。
- `src/ayanami_ros`：**唯一允许包含 ROS 头文件的地方**。职责只有三件：
  1. 把 ROS 世界的数据（TF / odom / scan / pointcloud / goal）转成
     ayanami 消息并送入核心（`setTransform / setOdometry / submitSensor /
     submitGoal`）。
  2. 以固定频率（50Hz）驱动核心 `spinOnce()`（可视化发布）与
     `ros::spinOnce()`（回调）。
  3. 承载 ROS 侧扩展（`rviz_viewer`）。

## 目录结构（ayanami）

| 目录 | 内容 |
|---|---|
| `include/ayanami/abstract/core` | 算法模块抽象接口（零地图类型）：Planner/Smoother/Controller/Recovery/GoalChecker/ProgressChecker/Navigation |
| `include/ayanami/abstract/components` | 运行时组件：ExecutionBase、四族 Execution、ActionBase、ActionServer/TaskHandle、ModuleManager、NavigationServerBase(NavigationRuntime，多模块 action map + goal 级 id 派发)、NavigationSessionState、RobotStateController(唯一 TF 查询单例) |
| `include/ayanami/behavior_tree` | BehaviorTreePilot、BT action/condition/selector 节点（CRTP 基类 NavigationActionNode）、控制节点（PipelineSequence/RecoveryNode/RateController/RecoverySequence(轮换)）、黑板协议、BehaviorTreeExtension |
| `include/ayanami/costmap` | costmap 地图族：MapContext、CostmapReader、OverlayPolicy、命令端口、地图族 Execution/Action |
| `include/ayanami/map` | 地图族抽象：AbstractMapContext、MapCommandPort、MapFamilyFactory |
| `include/ayanami/interfaces` | NavigationCallbacks 全局旁路槽（可视化 + 无损 raw 代价图槽） |
| `include/ayanami/messages` | 自研消息（geometry/nav/sensor/actions/failure/recovery_params/timestamp） |
| `include/ayanami/utility` | Config/GlobalConfig、logging、CallbackSlot/CallbackSubscription、loadModule/loaded_module/extension_module |
| `modules/` | 12 个 dlopen 模块（见 [modules.md](modules.md)） |
| `packages/` | costmap_2d（Costmap2D/LayeredCostmap）、costmap_layers（Static/SimpleObstacle/Inflation）、costmap_server（CostmapWrapper） |
| `tests/` | 52 个 gtest（见 [testing.md](testing.md)） |
| `thirdparty/` | BehaviorTree.CPP v4、nlohmann_json、ankerl |

## 线程模型

Ayanami 内部**没有进程级线程池**（WorkerPool 已移除）。线程模型为
"**每个长任务一线程**"，单导航并发上限天然有界：

| 线程 | 数量 | 职责 |
|---|---|---|
| AyanamiROS loop | 1 | 50Hz：TF 同步 → TF 健康诊断 → spinOnce(可视化) → drainPendingGoal → ros::spinOnce |
| BT tick（navigation_thread） | 0/1 | BT tick 循环（默认 100Hz；空闲时在邮箱条件变量上等待） |
| Action 驱动线程 | 每槽 1 | 轮询 execution 状态机并结算 goal（runImpl） |
| Execution 状态机线程 | 每执行 1 | 模块调用循环（planner 单发 / controller 周期 / recovery tick） |

同步纪律：

- 并发槽（ActionBase）：同槽新 goal 必须先 cancel 旧 execution 并等旧驱动
  线程退出（`task_done` + `task_cv`）后才允许提交新任务 → 同族模块绝不重入。
- 执行线程异常有 `k_aborted = 250` 兜底终态，保证驱动任务不会死轮询、
  recovery 互斥不会死锁。
- 模块调用前/发布命令前强制复查 `cancel_requested / should_exit`。

## 核心设计决策（Ruling 摘录）

1. **主流控制流不走全局回调**：BT 节点直接调 `NavigationRuntime::request*`
   拿 `TaskHandle`（直连，future + onComplete）。全局回调槽
   （`NavigationCallbacks`）只承载旁路输出（可视化 / 监控）。
2. **树实例按尝试新建**：工厂（节点 + 树 XML）只建一次；每次导航尝试建一棵新树
   + 新黑板，尝试结束即释放树对象、保留黑板作为记录。树生命期 == 树实例，跨生命期
   状态泄漏按构造不可能（见 [behavior-tree.md](behavior-tree.md)）。
3. **latest-wins 单一目标**：同一时间只有一个导航任务。新 goal 覆写邮箱；
   planner 原位换目标；controller 持续跟随旧路径直到新路径平滑落地后
   in-place `updatePlan`。版本计数合并为**单一单调 plan_version**（规划
   +1、平滑 +1；SmooothPath 按内容签名跳过相同输入）。
4. **地图族抽象**：核心与宿主只认 `MapFamilyFactory` / `AbstractMapContext`
   抽象面；CostmapMapContext 的创建经由工厂；`libayanami` 本体不含任何
   costmap 符号（`map_family_abstraction_test` 守护）。
5. **地图读契约**（三族各取所需，写者永不被算法阻塞）：
   - planner：`snapshotGlobal()` 短锁快照——共享锁只住一次 memcpy，A* 在
     不可变副本上跑（`costmap_snapshot.hpp`）；
   - smoother：订阅 `NavigationCallbacks::on_costmap_raw` 持有自有副本
     （有损的 occupancy 可视化槽不变），平滑全程零活锁图；
   - controller / recovery：`readLocal()` 直读（滚动窗小、调用短）。
   全局图 = static + (可选) obstacle + inflation（nav2 形状，
   `costmap_global.observation_sources` 或层 json 启用）。local costmap
   建在 odom 系（定位跳变免疫）。视图禁止存为长生命周期成员。
6. **恢复调度归 BT**：执行层失败只返回结构化 `ErrorCode`，不做重试决策；
   恢复动作的编排全部在行为树（RecoveryNode / RecoverySequence）。

## 运行时组件交叉引用

| 组件 | 头文件 | 说明 |
|---|---|---|
| NavigationRuntime | `abstract/components/navigation_server_base.hpp` | BT → 执行层的抽象接口（request*/session/plan_version） |
| NavigationSessionState | `abstract/components/navigation_session_state.hpp` | session_id / plan_version 单一来源（pilot 与 server 共享同一对象） |
| RobotStateController | `abstract/components/robot_state_controller.hpp` | 跨 DSO 单例：最新位姿(TF)+最新速度(odom)，帧名一次性缓存 |
| RobotStateController | `abstract/components/robot_state_controller.hpp` | 精简桥接器：TF 查询面与 tf2 对齐（注入 ayanami_ros 的 tf2 源）+ boost::circular_buffer 里程计缓存（跨 DSO 单例） || NavigationCallbacks | `interfaces/callbacks.hpp` | 旁路输出槽（rviz_viewer / 监控订阅），静态成员只在 libayanami 内定义一份 |
| ModuleManager | `abstract/components/module_manager.hpp` | dlopen + dynamic_pointer_cast + init_fn + 名称表 |
