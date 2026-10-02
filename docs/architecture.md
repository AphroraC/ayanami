# 总体架构

## 分层总览

ayanami 采用"算法核心与中间件彻底分离"的两包结构：

```
┌──────────────────────────────────────────────────────────────┐
│ ayanami_ros（唯一的 ROS1 边界）                                │
│  AyanamiROS: setTransform / setOdometry / submitSensor /      │
│              submitGoal / drainPendingGoal / start / stop      │
│  TF2Listener / ros_msg_converter / 扩展(rviz_viewer 等)        │
└──────────────┬───────────────────────────────────────────────┘
               │ dlopen(routes/*.json::navigation.so_name)
               │      + AbstractNavigation 接口
┌──────────────▼───────────────────────────────────────────────┐
│ ayanami（纯 C++，零 ROS 头文件）                                │
│                                                               │
│  宿主模块层  rose_navigation_fsm / costmap_navigation_fsm      │
│    ├─ CostmapNavigationServer 成员（地图唯一写者）              │
│    ├─ [rose] EsdfWriter（costmap mask → 派生 ESDF 场）          │
│    └─ FsmDriver（单体 FSM，导航编排）                           │
│                                                               │
│  服务器基类  NavigationServerBase（无地图依赖）                 │
│              六族 ModuleManager + initialize*Module 虚钩子      │
│              + Execution 装配（createActions 尾钩子）           │
│                                                               │
│  执行层      ExecutionBase::runBlocking                        │
│              Planner/Smoother/Controller/Recovery 四族          │
│              （FSM 内联/常驻线程直驱，无 Action 层）             │
│                                                               │
│  模块接口层  AbstractPlanner/Smoother/Controller/Recovery/     │
│              GoalChecker/ProgressChecker（零地图类型）          │
│              + 地图族接口 CostmapPlanner / EsdfmapSmoother      │
│                                                               │
│  地图层      CostmapMapContext / EsdfMap（FSM 类成员）          │
│              （packages/costmap：global/local 双帧 + 层静态组合）│
│              （packages/esdfmap：EsdfMap 快照 + EsdfWriter）    │
│                                                               │
│  基础设施    Config/GlobalConfig、NavigationCallbacks 静态槽、  │
│              RobotStateController(TF 桥+里程计)、ModuleManager  │
└───────────────────────────────────────────────────────────────┘
```

## 两个包、一条红线

- `src/ayanami`：**零 ROS 头文件**（红线）。消息全部自研
  （`messages/`，Eigen 存储），TF 按需经注入的 tf2 源查询。
- `src/ayanami_ros`：**唯一允许包含 ROS 头文件的地方**。职责只有三件：
  1. 把 ROS 世界的数据（TF / odom / scan / pointcloud / goal）转成
     ayanami 消息并送入核心（`setTransform / setOdometry / submitSensor /
     submitGoal`）。
  2. 以固定频率（50Hz）驱动 loop：TF 健康诊断 → `spinOnce()`（FSM 宿主
     现为空实现）→ `drainPendingGoal()` → `ros::spinOnce()`。
  3. 承载 ROS 侧扩展（rviz_viewer / rose_viewer / cmd_chassis_publisher）。

## 四大机制（架构不变量）

1. **参数加载**（`utility/config.hpp`）：多层 JSON，`Config`/`GlobalConfig`
   单一来源；`auditUnusedParameters` 审计"声明了但没人读"的键，
   `global.strict_params = true` 时升级为启动失败。
2. **回调插槽**（`utility/callback_slot.hpp` + `interfaces/callbacks.hpp`）：
   CoW 无锁发射的静态旁路槽；静态成员只在 libayanami 内定义一份
   （跨 .so 单实例）。主控制流不经过这里——只承载可视化/监控输出。
3. **extension 插件**（`utility/load_module.hpp` + `ModuleManager`）：
   dlopen `libayanami_<name>.so` + 零参工厂 `extern "C"
   create_extension_module` + 族 `kApiVersion` ABI 契约；LoadFailure 七值
   分类（None/InvalidSpec/OpenOrSymbolFailed/TypeMismatch/
   ApiVersionMismatch/IdentityMismatch/InitRejected），失败即启动中止。
4. **算法/ROS 分离**：`messages/` 自有消息类型；`RobotStateController`
   全局单例是唯一 TF 查询面（ayanami_ros 启动时注入 tf2 源）；
   `ayanami_ros` 是唯一 ROS 边界。

## 目录结构（ayanami）

| 目录 | 内容 |
|---|---|
| `include/ayanami/abstract/core` | 算法模块抽象接口（零地图类型）：Planner/Smoother/Controller/Recovery/GoalChecker/ProgressChecker/Navigation |
| `include/ayanami/abstract/components` | 运行时组件：ExecutionBase、四族 Execution、**FsmDriver**、ModuleManager、NavigationServerBase、NavigationSessionState、RobotStateController(唯一 TF 查询单例) |
| `include/ayanami/costmap` | costmap 地图族：MapContext、CostmapReader、层构建器（静态组合）、观测管线、CostmapNavigationServer、族级 Execution |
| `include/ayanami/esdfmap` | esdfmap 地图族：EsdfmapSmoother 接口 + EsdfmapSmootherExecution |
| `packages/esdfmap` | ESDF 地图组件（场/写者/快照/查询） |
| `include/ayanami/interfaces` | NavigationCallbacks 全局旁路槽（可视化/监控输出） |
| `include/ayanami/messages` | 自研消息（geometry/nav/sensor/actions/failure/recovery_params/timestamp） |
| `include/ayanami/utility` | Config/GlobalConfig、logging、CallbackSlot/CallbackSubscription、loadModule/loaded_module/extension_module、bootstrap |
| `modules/` | 11 个 dlopen 模块（9 个算法/宿主模块 + 2 个 FSM 宿主，见 [modules.md](modules.md)） |
| `packages/` | `costmap`（Costmap2D/LayeredCostmap/三Layer/CostmapWrapper 合一单库）、`esdfmap`（EsdfMap/EsdfWriter/EDT/快照）；均不依赖 libayanami |
| `tests/` | 32 个 gtest（见 [testing.md](testing.md)） |
| `thirdparty/` | nlohmann_json、ankerl（BehaviorTree.CPP 已随 BT 架构删除） |

## 线程模型

没有进程级线程池；线程模型为"**常驻职责线程 + FSM 内联**"：

| 线程 | 数量 | 职责 |
|---|---|---|
| AyanamiROS loop | 1 | 50Hz：TF 健康诊断(1Hz) → spinOnce(FSM 宿主空) → drainPendingGoal → ros::spinOnce |
| FSM PLAN 线程 | 1/宿主 | FSM tick 循环（默认 20Hz）；goal 邮箱消费 + kPlan/kFollow/kRecovering 一拍；planner/smoother/recovery `runBlocking` 内联 |
| FSM CONTROL 线程 | 1/宿主 | follow 握手等待 → ControllerExecution::runBlocking 阻塞到终态 |
| costmap 写者线程 | 1/server | 观测邮箱消费（TF 有界等待/落图/滚动/清图/层工件发射/传感器活性） |
| ESDF 写者线程 | 1（rose） | mask 读取 → 融合 → EDT → 快照发布（update_rate 上限，generation 门控） |

同步纪律：

- FsmDriver 的 goal 邮箱与 follow 握手均为互斥锁 + 条件变量；
  状态字与版本计数为原子量。跨线程从不 halt 执行——取消一律置位 +
  非阻塞 `cancel()`，由所属线程在自己循环内收尾。
- costmap：单写者多读者（`std::shared_mutex`）；读者视图/快照禁止存为
  长生命周期成员。ESDF：一写者多读者，不可变快照原子换。
- 执行线程异常有 `k_aborted = 250` 兜底终态（见
  [execution-runtime.md](execution-runtime.md)）。

## Action 层的删除

BT 时代的主流控制流是 BT 节点 → `NavigationRuntime::request*` → ActionBase
并发槽 → 槽内驱动线程 → Execution。FSM 架构下该整层已删除：

- Execution 不再被 Action 包装，而是由 FsmDriver **直驱**：planner/smoother/
  recovery 在 PLAN 线程内联 `runBlocking()`，controller 由 CONTROL 线程
  `runBlocking()` 到终态。
- `NavigationServerBase` 相应瘦身：删 Action 组装与 NavigationRuntime 接口；
  `createActions()` 名字保留，但语义变为 loadModules 尾部的**装配钩子**
  （宿主在此建三族 Execution + recovery 列表 + FsmDriver）。
- 同族不重入由构造保证：每族只有一个 Execution 对象，且同一时间只有
  FsmDriver 的一个线程调用它。

## 核心设计决策（Ruling 摘录）

1. **单体 FSM 是 BT 的语义继承者**：重试预算+goal 修订复位、不可恢复短路、
   恢复轮转游标、原地修订、节流重规划、振荡门、每尝试恰一次
   `on_navigation_result`——五项语义从 BT 平移为普通代码（逐项对照见
   [fsm.md](fsm.md)）。策略可读可断点，不再有黑板协议与建树期校验。
2. **latest-wins 单一目标**：同一时间只有一个导航任务。新 goal 覆写邮箱；
   尝试中的 goal 是原地修订（预算复位 + 强制重规划，controller 不打断）；
   新路径经 controller `updatePlan` 原地落地，机器人持续跟随旧路径直到新路径
   平滑落地。`goal_revision` / `plan_version` 两个单调计数。
3. **地图即组件**：地图（costmap/esdfmap）是 FSM 的类成员，无族抽象间接层
   抽象面；`libayanami` 本体不含任何 costmap 符号。地图包
   （packages/costmap、packages/esdfmap）不依赖 libayanami，宿主以普通
   成员组合它们。
4. **地图读契约**（写者永不被算法阻塞）：
   - planner：`snapshotGlobal()` 短锁快照，A* 在不可变副本上跑；
     路径校验经 `isPathValid` 的 O(path) 稀疏探针（FusedMapView，
     global+local 逐点 MAX 融合）；
   - smoother：rose 路线读 ESDF 不可变快照（零锁）；costmap 路线
     `snapshotLocal()` 短锁快照；
   - controller / recovery：纯跟踪或读 local 直读（滚动窗小、调用短）。
5. **失败决策只看结构化 ErrorCode**：执行层失败返回
   `FailureContext{stage, code, ...}`；是否恢复由 `isRecoverable` 分类 +
   预算/游标在 FsmDriver 内决策，模块绝不解析 message。

## 运行时组件交叉引用

| 组件 | 头文件 | 说明 |
|---|---|---|
| FsmDriver | `abstract/components/navigation_fsm.hpp` | 单体 FSM 驱动（goal 邮箱 + PLAN/CONTROL 双线程 + 状态机） |
| NavigationServerBase | `abstract/components/navigation_server_base.hpp` | 六族 ModuleManager + 虚钩子 + Execution 装配（无地图依赖） |
| NavigationSessionState | `abstract/components/navigation_session_state.hpp` | session_id / plan_version 单一来源（宿主与 FsmDriver 共享） |
| RobotStateController | `abstract/components/robot_state_controller.hpp` | 跨 DSO 单例：最新位姿(TF)+最新速度(odom)，帧名一次性缓存 |
| NavigationCallbacks | `interfaces/callbacks.hpp` | 旁路输出槽（rviz_viewer / 监控订阅），静态成员只在 libayanami 内定义一份 |
| ModuleManager | `abstract/components/module_manager.hpp` | dlopen + 动态转换 + apiVersion + 参数注入 + 名称表 + LoadFailure 分类 |
