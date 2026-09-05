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
│  宿主插件层  CostmapNavigation（modules/costmap_navigation）    │
│    ├─ CostmapNavigationServer（地图族 server，唯一写者）        │
│    └─ BehaviorTreePilot（常驻树 tick 循环）                    │
│                                                               │
│  行为树层    ComputePathToPose / SmoothPath / FollowPath /     │
│              Recovery action 节点 + PipelineSequence /         │
│              RecoveryNode / RateController / RoundRobin /      │
│              RecoverySequence / 条件节点                        │
│                                                               │
│  执行层      NavigationServerBase(NavigationRuntime)           │
│              ActionBase 并发槽 ─► ExecutionBase 状态机线程      │
│              （Planner/Smoother/Controller/Recovery 四族）      │
│                                                               │
│  插件接口层  AbstractPlanner/Smoother/Controller/Recovery/     │
│              GoalChecker/ProgressChecker（零地图类型）          │
│              + 地图族接口 CostmapPlanner/Smoother/...          │
│                                                               │
│  地图层      AbstractMapContext ◄─ CostmapMapContext           │
│              (global/local CostmapWrapper + LayeredCostmap)    │
│                                                               │
│  基础设施    Config/GlobalConfig、NavigationCallbacks 静态槽、  │
│              RobotStateController、TransformStore(fast_tf)、    │
│              PluginManager、WorkerPool(已移除，见下)             │
└───────────────────────────────────────────────────────────────┘
```

## 两个包、一条红线

- `src/ayanami`：**零 ROS 头文件**（红线）。消息全部自研
  （`messages/`，Eigen 存储），TF 使用 vendored `fast_tf`，行为树使用
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
| `include/ayanami/abstract/core` | 算法插件抽象接口（零地图类型）：Planner/Smoother/Controller/Recovery/GoalChecker/ProgressChecker/Navigation |
| `include/ayanami/abstract/components` | 运行时组件：ExecutionBase、四族 Execution、ActionBase、ActionServer/TaskHandle、PluginManager、NavigationServerBase(NavigationRuntime)、NavigationSessionState、RobotStateController |
| `include/ayanami/behavior_tree` | BehaviorTreePilot、BT action/condition 节点、控制节点（PipelineSequence/RecoveryNode/RateController/RoundRobin/RecoverySequence）、黑板协议、BehaviorTreeExtension |
| `include/ayanami/costmap` | costmap 地图族：MapContext、CostmapView、命令端口、地图族 Execution/Action |
| `include/ayanami/map` | 地图族抽象：AbstractMapContext、MapCommandPort、MapCapability、MapFamilyFactory |
| `include/ayanami/interfaces` | NavigationCallbacks 全局旁路槽、TransformStore |
| `include/ayanami/messages` | 自研消息（geometry/nav/sensor/actions/failure/recovery_params/timestamp） |
| `include/ayanami/utility` | Config/GlobalConfig、logging、CallbackSlot/CallbackSubscription、load_module/loaded_module/extension_module |
| `modules/` | 11 个 dlopen 插件（见 [plugins.md](plugins.md)） |
| `packages/` | costmap_2d（Costmap2D/LayeredCostmap）、costmap_layers（Static/SimpleObstacle/Inflation）、costmap_server（CostmapWrapper） |
| `tests/` | 52 个 gtest（见 [testing.md](testing.md)） |
| `thirdparty/` | BehaviorTree.CPP v4、fast_tf、nlohmann_json、ankerl |

## 线程模型

Ayanami 内部**没有进程级线程池**（WorkerPool 已移除）。线程模型为
"**每个长任务一线程**"，单导航并发上限天然有界：

| 线程 | 数量 | 职责 |
|---|---|---|
| AyanamiROS loop | 1 | 50Hz：TF 同步 → TF 健康诊断 → spinOnce(可视化) → drainPendingGoal → ros::spinOnce |
| BT tick（navigation_thread） | 0/1 | 常驻树 tick 循环（默认 100Hz，`bt_tree.sleep` 可被唤醒） |
| Action 驱动线程 | 每槽 1 | 轮询 execution 状态机并结算 goal（runImpl） |
| Execution 状态机线程 | 每执行 1 | 插件调用循环（planner 单发 / controller 周期 / recovery tick） |

同步纪律：

- 并发槽（ActionBase）：同槽新 goal 必须先 cancel 旧 execution 并等旧驱动
  线程退出（`task_done` + `slots_cv`）后才允许提交新任务 → 同槽插件绝不重入。
- 执行线程异常有 `k_aborted = 250` 兜底终态，保证驱动任务不会死轮询、
  recovery 互斥不会死锁。
- 插件调用前/发布命令前强制复查 `cancel_requested / should_exit`。

## 核心设计决策（Ruling 摘录）

1. **主流控制流不走全局回调**：BT 节点直接调 `NavigationRuntime::request*`
   拿 `TaskHandle`（直连，future + onComplete）。全局回调槽
   （`NavigationCallbacks`）只承载旁路输出（可视化 / 监控）。
2. **常驻树**：树只建一次；新 goal 只重写黑板 + 推进 revision，绝不跨线程
   halt；树返回终端状态时由 pilot 调 `resetNavigationTreeState()` 全量重置
   内部状态（BT.CPP v4 halt 语义适配，见 [behavior-tree.md](behavior-tree.md)）。
3. **latest-wins 单一目标**：同一时间只有一个导航任务。新 goal 覆写邮箱；
   planner 原位换目标；controller 持续跟随旧路径直到新路径平滑落地后
   in-place `updatePlan`。
4. **地图族抽象**：核心与宿主只认 `MapFamilyFactory` / `AbstractMapContext`
   抽象面；CostmapMapContext 的创建经由工厂；`libayanami` 本体不含任何
   costmap 符号（`map_family_abstraction_test` 守护）。
5. **CostmapView 零拷贝读**：算法侧 `lockLocal()/lockGlobal()` 得到持有
   shared 锁的视图（RAII），单次调用 (<10ms) 持锁；视图禁止存为长生命周期
   成员（历史 bug，见 [troubleshooting.md](troubleshooting.md)）。
6. **恢复调度归 BT**：执行层失败只返回结构化 `ErrorCode`，不做重试决策；
   恢复动作的编排全部在行为树（RecoveryNode / RecoverySequence）。

## 运行时组件交叉引用

| 组件 | 头文件 | 说明 |
|---|---|---|
| NavigationRuntime | `abstract/components/navigation_server_base.hpp` | BT → 执行层的抽象接口（request*/session/plan_version） |
| NavigationSessionState | `abstract/components/navigation_session_state.hpp` | session_id / plan_version 单一来源（pilot 与 server 共享同一对象） |
| RobotStateController | `abstract/components/robot_state_controller.hpp` | 跨 DSO 单例：最新位姿(TF)+最新速度(odom)，帧名一次性缓存 |
| TransformStore | `interfaces/transform_store.hpp` | fast_tf 缓冲的线程安全包装（跨 DSO 单例） |
| NavigationCallbacks | `interfaces/callbacks.hpp` | 旁路输出槽（rviz_viewer / 监控订阅），静态成员只在 libayanami 内定义一份 |
| PluginManager | `abstract/components/plugin_manager.hpp` | dlopen + dynamic_pointer_cast + init_fn + 名称表 |
