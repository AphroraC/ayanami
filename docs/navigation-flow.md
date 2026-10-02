# 一次导航的完整生命周期

以 RViz/Gazebo 下发目标 `(-7, -3)`、机器人从 `(0, 0)` 出发为例，梳理从目标
下发到导航完成的每一步（含涉及的线程与组件）。编排主体是 `FsmDriver`
（见 [fsm.md](fsm.md)）；本页按时间顺序走一遍状态流转。

## 1. 目标进入（latest-wins 邮箱链）

```
RViz/Gazebo ──/move_base_simple/goal──► AyanamiROS::goalCallback（ros::spinOnce 线程）
    └─ submitGoal(pose)                 # mutex + pending_goal 覆写（不排队）
loop 线程（50Hz）├─ drainPendingGoal()  # 每周期最多投递最新一条
    └─ 宿主 so（AbstractNavigation::setGoal）
         └─ FsmDriver::beginNavigation(pose)  # pending_goal_ 覆写 + goal_cv 唤醒
```

消费永远发生在 FSM 的 PLAN 线程（`consumeGoalMailbox`，每拍 tick 边界）：

- **空闲**（kWaitGoal/终态驻留）：`startAttempt`——goal_revision +1、
  plan_version 归零、重试预算/恢复游标/位置历史复位、`resetCheckers()`，
  进 **kPlan**。
- **尝试中**：**原地修订**——goal_revision +1、重试预算复位、
  `force_replan_ = true`；controller 不被打断，继续跟随当前路径。

## 2. kPlan 一拍（规划 → 校验 → 平滑 → 校验 → 交班）

`tickPlan` → `runPlanOnce`（PLAN 线程内联）：

1. **plan**：`planner_->prepareGoal(start=空, goal, tolerance)` +
   `runBlocking()`（空 start 由 execution 经 RobotStateController 解析为
   最新位姿）。成功取 `takePlan()` 为 raw path，`raw_path_revision_` +1；
   失败写 `failure_context_`，走恢复/终态判定。
2. **校验 raw path**：`pathValid()` → 注入的 path validator
   （= planner 模块 `isPathValid`，FusedMapView global+local 融合探针；
   默认非空即有效）。
3. **smooth**：`smoother_->prepareGoal(raw_path, max_smooth_duration)` +
   `runBlocking()`。平滑失败 → **DegradeToRawPath**（降级 raw path，
   不失败）。
4. **校验平滑产物**：不合法且 raw 合法 → 回退 raw path；两者都不合法
   → 失败。
5. `current_path_` 落定，`plan_version` +1，`kickFollow()`：
   `controller_->setNewPlan + setPlanVersion` + follow 握手唤醒 CONTROL
   线程，进 **kFollow**。

每一步都伴随回调槽发射（见 §6）。

## 3. kFollow 一拍（跟随 + 监督）

`tickFollow`（PLAN 线程，`fsm_frequency` 节拍）：

1. **终态检查**：在 follow 条件变量上等至多一拍；CONTROL 线程报
   controller 到终态 → 发射 `on_exe_path_result`；到达
   （`ARRIVED_GOAL`）→ `finishAttempt(SUCCEEDED)`；失败 → 恢复/终态判定。
2. **振荡门**（原 IsRobotProgressing）：记录当前位置进 deque 并按
   `oscillation_timeout` 老化；若机器人仍停在 `oscillation_distance`
   窗口内 → NO_PROGRESS，`preemptController`（cancel 内恰一次零脉冲）
   后进恢复/终态判定。
3. **节流重规划**：`force_replan_`（goal 修订）或 `!pathValid(current_path_)`
   （路径失效）或 `planner_frequency` 到期 → 重跑 `runPlanOnce`，成功后
   `controller_->updatePlan(current_path_, plan_version)` **原地**换路径
   （无 cancel/无零脉冲，控制周期下一拍消费新路径）。
4. **迭代重平滑**：`smoother_frequency` 到期 → 以**当前平滑路径**为输入
   再平滑；候选通过校验才替换并 `updatePlan`；失败只告警（保持当前路径）。

CONTROL 线程（`controlLoop`）独立运行：握手后
`controller_->runBlocking()` 阻塞到终态——controller execution 每周期
（`controller_frequency`，出厂 50Hz）checkers → compute → 发布，语义见
[execution-runtime.md](execution-runtime.md)。

## 4. 恢复（kRecovering，轮转）

`tickRecovering`（PLAN 线程内联，逐恢复阻塞）：

- **轮转游标**（原 RecoverySequence 的 RoundRobin 语义）：
  `recoveries_[cursor]` 后游标 `(cursor+1) % size`——连续恢复轮换手段，
  不反复执行第一个。
- 进恢复先 `preemptController()`（运动恢复需要机器人停住；cancel 同步发
  恰一次零速）。
- 恢复 execution `runBlocking()`（内部 start → tick×N → finish，10ms 节拍，
  `recovery_patience` 框架级超时）；`RecoveryGoal.cause` = 当前
  failure_context（"谁/为什么触发恢复"）。
- **成功**：`retry_count_` +1、清失败上下文、`force_replan_` → 回 kPlan
  重规划重试。
- **失败/超时/取消**：`finishAttempt(ABORTED)`——对应 BT RecoveryNode
  "恢复子失败即整体失败"。

**进入恢复前**（`enterRecoveryOrTerminal`）的三道闸：

1. `!isRecoverable(code)`（INVALID_INPUT/START_INVALID/GOAL_INVALID/
   TRANSFORM_UNAVAILABLE/CANCELED）→ 直接 ABORTED（为坏目标烧恢复预算
   没有意义）；
2. recovery 列表为空 → ABORTED（"no recovery configured"）；
3. `retry_count_ >= max_nav_retries` → ABORTED（"retry budget exhausted"）。

出厂配置的恢复列表只有 `abstract_nothing_recovery`（占位：跑完整协议、
一拍即成功）——恢复**编排**已被 fsm_test 锁定，换真恢复行为是换插件，
不是改框架。

## 5. 终态与驻留

- `finishAttempt(status, why)`：`preemptController` → 发射
  `on_navigation_result`（**每次尝试恰一次**；SUCCESS/CANCELED/ABORTED +
  FailureContext）→ 置对应终态。
- **终态驻留**（原 pilot 空闲语义）：kSucceeded/kAborted/kCanceled 不再
  tick、不再规划——修复过"到达后 100Hz 空转、plan_version 以 60/s 膨胀"
  的历史事故。下一个 `beginNavigation` 从终态直接开新尝试，两个尝试之间
  没有任何残留状态。
- `cancelNavigation()`：置位 + 非阻塞 cancel 全部在飞 execution，所在状态
  的下一拍边界结算 CANCELED。

## 6. 回调槽发射点（全地图）

| 槽（`interfaces/callbacks.hpp`） | 发射点 |
|---|---|
| `on_get_path_goal` / `on_get_path_result` | runPlanOnce 的规划调用前/后（成功与失败都发） |
| `on_smooth_path_goal` / `on_smooth_path_result` | runPlanOnce 的平滑调用前/后（降级也发 SUCCEEDED，error=NONE） |
| `on_exe_path_goal` | kickFollow（controller 一次跟随运行开始） |
| `on_exe_path_feedback` | controller execution 每控制周期（speed / distance_remaining / ETA） |
| `on_exe_path_result` | kFollow 收到 controller 终态握手时（reached 区分 SUCCEEDED/ABORTED） |
| `on_recovery_goal` / `on_recovery_result` | tickRecovering 的恢复调用前/后 |
| `on_velocity_command` | controller execution 每个有效控制周期（恢复期间由 recovery execution 发布） |
| `on_navigation_result` | finishAttempt（每尝试恰一次） |
| `on_global_costmap_static/inflation`、`on_local_costmap_obstacle/inflation` | costmap 唯一写者，generation 门控内容变更时 |

`*_feedback` 槽（on_get_path_feedback / on_smooth_path_feedback /
on_recovery_feedback）在 FSM 架构下无生产发射点（planner/smoother/recovery
单发执行没有周期反馈），槽位保留供扩展使用。

## 与 nav2 的语义对照

| 场景 | nav2 | ayanami |
|---|---|---|
| 目标抢占 | preemption + 新树 | goal 邮箱 + 原地修订（预算复位 + 强制重规划） |
| 重规划 | BT RateController 1Hz | `planner_frequency` 节流 + 路径失效即刻重规划 |
| 恢复 | RecoveryNode + RecoverySequence(BT) | 重试预算 + 不可恢复短路 + 轮转游标（普通代码） |
| 平滑失败 | SmoothPath FAILURE 进恢复分支 | DegradeToRawPath 降级（增强性失败不是导航失败） |
| 代价图失同步 | 手动/恢复清除 | 路径失效探针即时重规划（见 map-and-costmap.md） |
| error 传递 | error code 字段 | ErrorCode + FailureContext + isRecoverable 分类 |
