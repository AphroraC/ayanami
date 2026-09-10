# 执行运行时（Execution / Action / 错误码）

## 三层概念分工（固定原则）

| 概念 | 载体 | 职责 |
|---|---|---|
| **ErrorCode / FailureCode** | `types.hpp::ErrorCode : uint8_t` | 业务失败原因（唯一决策依据） |
| **ExecutionState** | 各 Execution 的内部枚举 | 执行器内部状态，仅诊断与反馈 |
| **GoalStatus** | `messages/actions.hpp`（PENDING/ACTIVE/SUCCEEDED/CANCELED/ABORTED） | 对外任务生命周期（action 层映射） |
| **BT NodeStatus** | BT 节点 | 行为树控制流 |

禁止：BT 依据 ExecutionState 或 message 字符串做决策；只依据明确的
ErrorCode 与结构化端口。`FailureCode` 是 `ErrorCode` 的概念别名
（`messages/failure.hpp`），历史 CamelCase 拼写已删除。

## ErrorCode 枚举（types.hpp）

NONE(0), CANCELED, TIMEOUT, INVALID_INPUT, TRANSFORM_UNAVAILABLE,
MAP_UNAVAILABLE, MAP_STALE, START_INVALID, GOAL_INVALID, PATH_NOT_FOUND,
PATH_INVALID, SMOOTHING_FAILED, NO_VALID_COMMAND, NO_PROGRESS,
GOAL_NOT_REACHED, RECOVERY_FAILED, INTERNAL_ERROR, STALE_RESULT,
RESOURCE_BUSY。

## 返回值模板

```cpp
template <typename DataT>
struct AbstractResultT {
  bool ok; DataT data; ErrorCode error; std::string message;
  static AbstractResultT success(const DataT& / DataT&&);
  static AbstractResultT failure(ErrorCode, const std::string&);
  explicit operator bool() const;
};
using PlannerResultT   = AbstractResultT<nav_msgs::Path>;
using SmootherResultT  = AbstractResultT<nav_msgs::Path>;
using ControllerResultT = AbstractResultT<ControllerCommand>;  // TwistStamped
using RecoveryResultT  = AbstractResultT<RecoveryCycleData>;   // status + optional<TwistStamped>
```

派生 Result 结构（`messages/actions.hpp`）额外携带 `session_id /
plan_version`（BT 节点 onRunning 的陈旧结果校验）与 `reached`（controller）。
**全局回调仍使用 ayanami::message 格式。**

## Execution 层

`ExecutionBase`（每执行一条专有线程）：`start()` spawn →
`runTask()` = preRun → run → postRun；`state` 为 `atomic<int>` +
condition_variable；`stop()`（rigorous，run 内以 STOPPED 退出）、
`cancel()`（置位 + publishStopCommand）、`join()`、
`waitForStateUpdate(timeout)`（事件驱动轮询）。**逃逸异常兜底**：
runTask catch 后置 `k_aborted=250` 统一终态并发布停止命令——驱动任务必须
将其视为终态，否则槽占用与 recovery 互斥会死锁。

四族 Execution 与状态机：

| Execution | 枚举 | 关键行为 |
|---|---|---|
| PlannerExecution | INITIALIZED/STARTED/PLANNING/FOUND_PLAN/NO_PLAN_FOUND/CANCELED/STOPPED/INTERNAL_ERROR | **单发**：失败立即映射 ErrorCode 终态（重试归 BT）；latest-wins `setNewGoal`（revision 递增，采纳最新 pending 目标，不打断在飞调用） |
| SmootherExecution | INITIALIZED/STARTED/SMOOTHING/SMOOTHED_PATH/NO_SMOOTHED_PATH/CANCELED/STOPPED/INTERNAL_ERROR | in-place 平滑输入路径 |
| ControllerExecution | INITIALIZED/STARTED/PLANNING/NO_PLAN/EMPTY_PLAN/INVALID_PLAN/NO_LOCAL_CMD/GOT_LOCAL_CMD/ARRIVED_GOAL/NO_PROGRESS/CANCELED/STOPPED/INTERNAL_ERROR/ROBOT_DISABLED | 周期循环（checkers → compute → 发布）；`setNewPlan/updatePlan` in-place；发布前复查 cancel/stop；速度上限校验；到达判定唯一来源 = goal_checker |
| RecoveryExecution | INITIALIZED/STARTED/RECOVERING/RECOVERY_DONE/RECOVERY_FAILED/CANCELED/STOPPED/INTERNAL_ERROR/PAT_EXCEEDED | 统一协议 start→tick→finish；唯一速度发布者；patience 超时 |

## Action 层

`ActionBase<Goal, Execution>`：单在飞槽 + 抢占语义（对标 MBF
AbstractActionBase；单导航纪律下旧并发槽收敛为单在飞记录 + pending 槽）。

- 每个在飞任务持 execution + **驱动线程**（runImpl 一次完整执行）。
- `start()`：同槽在飞时先 cancel 旧 execution（cancel 内同步发零速）→
  新 goal 停靠 pending 槽 → 旧驱动任务退出时 teardown 链装新驱动线程
  （tick 线程绝不阻塞在慢模块上，同槽模块绝不重入）。
- `runImplCommon`：轮询骨架——先查 `aborted()`（k_aborted 兜底优先），
  再交给派生的 StateMapper（透传 cancel / Execution 状态→GoalStatus 映射 /
  finish 结算），`waitForStateUpdate` 事件驱动 + 20ms 兜底。
- `runImpl` 异常兜底：`abortGoal` 保证 TaskHandle 完成（不完成会让等待方与
  recovery 互斥悬挂）。
- **TaskHandle 完成顺序**：`TaskState::complete` 先跑 onComplete listener
  （旁路 emit + recovery 互斥释放）再 `promise.set_value`——先释放资源后
  发布可见性，杜绝"BT 轮询已见 ready 但互斥未释放"的 RESOURCE_BUSY 竞态。

## 多模块与 goal 级选择（nav2 语义）

- planner/smoother/controller 支持**多模块配置**：每个加载的模块都有独立
  action；recovery 本就按 id 派发。
- goal 携带 `planner_id / smoother_id / controller_id`（BT 端口经
  PlannerSelector/ControllerSelector/SmootherSelector 写黑板后绑定）；空 id
  = 家族默认（第一个配置的模块）；显式未知 id → INVALID_INPUT 结构化拒绝。
- `requestPlanUpdate(goal_id, ...)` 在全部 controller action 中按 goal_id
  寻址；cancelAll*/preemptAll* 覆盖整个 action map（防止非默认 id 残留
  速度输出）。
- 生产配置仍为每族单模块（机制完备、配置从简）。

## Feedback（nav2 对齐）

- `ExePathFeedback{speed, distance_remaining, estimated_time_remaining,
  message}`：controller execution 每控制周期计算（最近点投影剩余长 +
  速度 EMA ETA），经 action 层发到 `on_exe_path_feedback` 插槽。
- planner/smoother/recovery：nav2 本身无 feedback，ayanami 仅发 stage
  message（accept/finished）。
- 均只经全局回调插槽旁路输出；**ayanami_ros 不发布任何 *_feedback topic**。

## NavigationServerBase（NavigationRuntime）

- `requestPath/requestSmoothing/requestExecution/requestRecovery` →
  TaskHandle（主流）；request* 会**盖上当前 session_id / plan_version**
  （测试 mock 必须复刻同一契约），并按 goal 级 id 派发到对应 action。
- `requestRecovery` 资源协议：CAS `recovery_exclusive` → 校验 id →
  **同步 `cancelAllControllers()`** → acceptGoal →
  `onComplete(释放互斥)`。恢复期间其余 request* 返回 RESOURCE_BUSY
  结构化拒绝（valid+ready+ABORTED，不产生旁路事件）。
- `requestPlanUpdate`：controller in-place 路径更新（无 cancel/join）。
- 无人使用时 `cancelAllPlanners/Smoothers/Controllers` 供 halt/抢占调用。
- checker 注入与速度上限注入（`configureControllerExecution`）。
- 空 start 解析上收框架：`PlannerExecution::start/setNewGoal` 把空 start
  （默认构造位姿）经 RobotStateController 解析为最新机器人位姿；模块契约
  为"start 永远有效"，模块不再各自解释哨兵。

## 全局动态代价图重建触发（AyanamiROS）

累计机器人旋转/平移（阈值可配 `localization_clear_*`，默认 1.0 rad / 2.0 m）
超过后调用宿主 `onLocalizationJump()` → 清空全局动态层。信号必须取
**机器人运动量**（odom 边）而不是修正增量——AMCL 修正渐进且可能长期不收敛。
背景：原地旋转时轮式里程计打滑欠报转角（~6%），AMCL 修正跟不上会导致
map 系 scan 持续旋转、全局动态标记整体错位。

## 与 MBF 的对照

| MBF | ayanami |
|---|---|
| AbstractPlannerExecution（一线程 + 状态机 + patience/retries） | ExecutionBase + 四族 Execution（**无内部重试**，重试归 BT） |
| AbstractActionBase 并发槽 + 临时线程 | ActionBase 并发槽 + 槽内驱动线程（每长任务一线程，无池） |
| uint32 outcome + message | ErrorCode 枚举 + Result 结构 + 结构化拒绝（RESOURCE_BUSY 可恢复） |
| pluginlib | dlopen + create_extension_module 工厂 |
| 无 feedback 概念（MBF）| nav2 式 feedback：ExePath 周期度量经 on_exe_path_feedback 插槽 |
