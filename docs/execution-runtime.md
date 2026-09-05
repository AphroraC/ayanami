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

`ActionBase<Goal, Execution>`：并发槽 + 抢占语义（对标 MBF
AbstractActionBase）。

- 每个 `ConcurrencySlot` 持 execution + **驱动线程**（runImpl 一次完整执行）。
- `start()`：同槽先 cancel 旧 execution → 等旧驱动任务 `task_done` →
  join 旧线程 → 换装新 execution → spawn 新驱动线程（同槽绝不重入）。
- `runImplCommon`：轮询骨架——先查 `aborted()`（k_aborted 兜底优先），
  再交给派生的 StateMapper（透传 cancel / Execution 状态→GoalStatus 映射 /
  finish 结算），`waitForStateUpdate` 事件驱动 + 20ms 兜底。
- `runImpl` 异常兜底：`abortGoal` 保证 TaskHandle 完成（不完成会让等待方与
  recovery 互斥悬挂）。

## NavigationServerBase（NavigationRuntime）

- `requestPath/requestSmoothing/requestExecution/requestRecovery` →
  TaskHandle（主流）；request* 会盖上当前 session_id / plan_version。
- `requestRecovery` 资源协议：CAS `recovery_exclusive` → 校验 id →
  **同步 `cancelAllControllers()`** → acceptGoal →
  `onComplete(释放互斥)`。恢复期间其余 request* 返回 RESOURCE_BUSY
  结构化拒绝（valid+ready+ABORTED，不产生旁路事件）。
- `requestPlanUpdate`：controller in-place 路径更新（无 cancel/join）。
- 无人使用时 `cancelAllPlanners/Smoothers/Controllers` 供 halt/抢占调用。
- checker 注入与速度上限注入（`configureControllerExecution`）。

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
| uint32 outcome + message | ErrorCode 枚举 + Result 结构 + 结构化拒绝 |
| pluginlib | dlopen + create_extension_module 工厂 |
