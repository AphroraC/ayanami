# 执行运行时（Execution / 错误码）

## 概念分工（固定原则）

| 概念 | 载体 | 职责 |
|---|---|---|
| **ErrorCode / FailureCode** | `types.hpp::ErrorCode : uint8_t` | 业务失败原因（唯一决策依据） |
| **ExecutionState** | 各 Execution 的内部枚举 | 执行器内部状态，仅诊断与反馈 |
| **GoalStatus** | `messages/actions.hpp`（PENDING/ACTIVE/SUCCEEDED/CANCELED/ABORTED） | 对外任务生命周期（回调槽映射） |
| **FSM State** | `FsmDriver::State` | 导航编排控制流（见 [fsm.md](fsm.md)） |

禁止：FSM 依据 ExecutionState 或 message 字符串做决策；只依据明确的
ErrorCode 与 `FailureContext{stage, code, module_id, message}`。
`FailureCode` 是 `ErrorCode` 的概念别名（`messages/failure.hpp`），
`isRecoverable()` 分类也定义在这里（INVALID_INPUT/START_INVALID/
GOAL_INVALID/TRANSFORM_UNAVAILABLE/CANCELED 不可恢复；RESOURCE_BUSY
刻意不算致命——瞬时态重试是正确响应）。

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
using ControllerResultT = AbstractResultT<ControllerCommand>;  // TwistStamped
using RecoveryResultT   = AbstractResultT<RecoveryCycleData>;  // status + optional<TwistStamped>
```

派生 Result 结构（`messages/actions.hpp`）额外携带 `session_id /
plan_version`（FSM 侧的会话/版本一致性字段）与 `reached`（controller）。
**全局回调仍使用 ayanami::message 格式。**

## Execution 层：FSM 直驱

`ExecutionBase`：`prepareGoal*()` 装目标 → `runBlocking()` =
preRun → run → postRun（一次完整执行，**调用线程就是执行线程**）；
`stop()`（rigorous，run 内以 STOPPED 退出）、`cancel()`（置位 +
publishStopCommand 恰一次零脉冲）、`waitForStateUpdate(timeout)`。
**逃逸异常兜底**：runBlocking catch 后置 `k_aborted=250` 统一终态并发布
停止命令——调用方必须将其视为终态，否则执行器永不再就绪。

**Action 层已删除**：BT 时代的 ActionBase 并发槽 + 槽内驱动线程 +
TaskHandle/latest-wins 邮箱整体移除。现在每族恰好一个 Execution，由
FsmDriver 直接调用：

| Execution | 驱动方 | 模式 |
|---|---|---|
| PlannerExecution（costmap 族：CostmapPlannerExecution） | FSM PLAN 线程内联 | 单发：prepareGoal(start, goal, tolerance) → runBlocking → takePlan |
| SmootherExecution（esdfmap 族：EsdfmapSmootherExecution / costmap 族同名） | FSM PLAN 线程内联 | 单发：prepareGoal(path, max_duration) → runBlocking → takeSmoothedPath |
| ControllerExecution | FSM CONTROL 线程常驻 | 周期循环 runBlocking 阻塞到终态；`setNewPlan/setPlanVersion` 开新跟随，`updatePlan` 原地换路径 |
| RecoveryExecution | FSM PLAN 线程内联 | 单发 runBlocking（内部 start → tick×N → finish 统一协议） |

四族 Execution 与状态机：

| Execution | 枚举 | 关键行为 |
|---|---|---|
| PlannerExecution | INITIALIZED/STARTED/PLANNING/FOUND_PLAN/NO_PLAN_FOUND/CANCELED/STOPPED/INTERNAL_ERROR | **单发**：失败立即映射 ErrorCode 终态（重试归 FSM）；latest-wins `setNewGoal`（revision 递增，采纳最新 pending 目标，不打断在飞调用） |
| SmootherExecution | INITIALIZED/STARTED/SMOOTHING/SMOOTHED_PATH/NO_SMOOTHED_PATH/CANCELED/STOPPED/INTERNAL_ERROR | in-place 平滑输入路径 |
| ControllerExecution | INITIALIZED/STARTED/PLANNING/NO_PLAN/EMPTY_PLAN/INVALID_PLAN/NO_LOCAL_CMD/GOT_LOCAL_CMD/ARRIVED_GOAL/NO_PROGRESS/CANCELED/STOPPED/INTERNAL_ERROR/ROBOT_DISABLED | 周期循环（checkers → compute → 发布）；`setNewPlan/updatePlan` in-place（内容等价路径去重，1mm/1e-3 容差）；发布前复查 cancel/stop；速度上限校验；到达判定唯一来源 = goal_checker |
| RecoveryExecution | INITIALIZED/STARTED/RECOVERING/RECOVERY_DONE/RECOVERY_FAILED/CANCELED/STOPPED/INTERNAL_ERROR/PAT_EXCEEDED | 统一协议 start→tick→finish；恢复期间唯一速度发布者；patience 超时 |

## ControllerExecution 周期语义（nav2 对齐，全部保留）

每个周期（1/controller_frequency）：

1. cancel/stop 检查；`safetyCheck()`（ExecutionBase 虚钩子，基类默认恒真；
   costmap 族 controller execution 覆写 = local wrapper `isCurrent()`。
   ESDF 新鲜度门在 smoother 侧：EsdfmapSmootherExecution = `EsdfMap::current()`）。
2. `hasNewPlan()` → 原子取 `(plan, plan_version)` → 内容不等价才
   `controller->setPlan` → goal 位姿快照 + checker reset → applied 版本推进。
3. `RobotStateController::getRobotState`（位姿 = TF 最新；速度 = 最新
   odom，超 `velocity_timeout` 返回零速）；失败 → TRANSFORM_UNAVAILABLE 终态。
4. 进度度量（nav2 FollowPath feedback 语义）：speed、distance_remaining、
   estimated_time_remaining，经 `setFeedbackCallback` → `on_exe_path_feedback`。
5. `goal_checker->isGoalReached`（唯一到达判定）→ 到达：零速 + ARRIVED_GOAL。
6. `progress_checker->check`（无进展判定）→ 失败：NO_PROGRESS 终态。
7. `callController` → 有效命令：清零后经 `on_velocity_command` 发布；
   **发布前复查 cancel/stop**。
8. **静默失败语义**（nav2 controller_server）：模块失败/命令无效/安全检查
   不过 → 该周期**完全不发布**（不是发零速流），最后一条有效命令自然
   超时停车；仅当 `controller_patience` > 0 且无有效命令持续超过该时长才
   终态上报（TIMEOUT / MAP_STALE）。

**终态单零脉冲**：cancel/到达/卡死/异常的终态迁移**恰发布一次**零速度
阶跃，绝无零速度流。停机减速整形归属下游（见
[ros-integration.md](ros-integration.md)）。

## checker/参数装配（NavigationServerBase）

宿主在 `createActions()` 装配时经基类钩子完成：

- `configureControllerExecution`：goal_checker / progress_checker 注入、
  `max_vel_x/y/theta` 速度上限（发布前校验，超限清零）、
  `controller_frequency` 节拍、`controller_patience`、`velocity_timeout`。
- `configureRecoveryExecution`：速度上限 + `recovery_patience`
  （单次恢复框架级超时，≤0 关闭）+ 10ms tick 节拍（`setCyclePeriod`，
  模块用它积分，禁止自读框架参数）。
- 空 start 解析上收框架：`PlannerExecution::start/setNewGoal` 把空 start
  （默认构造位姿）经 RobotStateController 解析为最新机器人位姿；模块契约
  为"start 永远有效"。
- 每次新尝试 `FsmDriver::startAttempt` 调 `controller_->resetCheckers()`：
  Execution 复用模型下防止上一尝试的 checker 基线泄漏（重复同 goal
  不会触发 run loop 自己的 reset）。

## Feedback

- `ExePathFeedback{speed, distance_remaining, estimated_time_remaining,
  message}`：controller execution 每控制周期计算（最近点投影剩余长 +
  速度 EMA ETA），经 `setFeedbackCallback` 发到 `on_exe_path_feedback` 插槽。
- planner/smoother/recovery：单发执行没有周期 feedback；FSM 架构下
  `on_get_path_feedback` / `on_smooth_path_feedback` /
  `on_recovery_feedback` 槽位保留、无生产发射点。
- 均只经全局回调插槽旁路输出；**ayanami_ros 不发布任何 *_feedback topic**。

## 与 MBF 的对照

| MBF | ayanami |
|---|---|
| AbstractPlannerExecution（一线程 + 状态机 + patience/retries） | ExecutionBase + 四族 Execution（**无内部重试**，重试归 FSM 预算） |
| AbstractActionBase 并发槽 + 临时线程 | **已删除**：FSM 内联/常驻直驱（每族一个 Execution，构造上不可重入） |
| uint32 outcome + message | ErrorCode 枚举 + Result 结构 + FailureContext（isRecoverable 分类） |
| pluginlib | dlopen + create_extension_module 工厂 |
| 无 feedback 概念（MBF）| nav2 式 feedback：ExePath 周期度量经 on_exe_path_feedback 插槽 |
