# 一次导航的完整生命周期

以 RViz/Gazebo 下发目标 `(-7, -3)`、机器人从 `(0, 0)` 出发为例，梳理从目标
下发到导航完成的每一步（含涉及的线程与组件）。

## 1. 目标进入（latest-wins 邮箱链）

```
RViz/Gazebo ──/move_base_simple/goal──► AyanamiROS::goalCallback（ros::spinOnce 线程）
    └─ submitGoal(pose)                 # mutex + pending_goal 覆写（不排队）
loop 线程（50Hz）├─ drainPendingGoal()  # 每周期最多投递最新一条
    └─ CostmapNavigation::setGoal(pose)
         ├─ current_goal = pose（+ state=RUNNING）
         └─ bt_pilot.beginNavigation(pose)   # pending_goal 覆写 + emitWakeUpSignal
              └─ 若 navigation_thread 未建则创建（bt_pilot.run 常驻循环）
```

**绝不跨线程 halt 树**：抢占（halt/cancel/零速度）全部落在 tick 线程的
`applyPendingGoal` 中完成。

## 2. applyPendingGoal（tick 线程）

- 首个 goal：`session_state->startNewSession()`（session+1，plan_version=0），
  `goal_revision = 0`。
- 后续 goal：`++goal_revision`（同一会话内的目标修订）。
- 重写黑板：`goal / goal_revision / session_id / raw_path_revision /
  failure_context(清零)`。
- goal_revision 的变化同时是树上 planner RateController 与
  RecoverySequence 的 **reset_token**（新导航轮重置游标与缓存）；
  smoother 分支的 reset_token 是 {raw_path_revision}（规划落地即重平滑）。

## 3. 规划 → 平滑 → 跟随（Pipeline 流水线）

生产默认树（内嵌于 navigation_behavior_tree 扩展，`spec §6.3`）：

```xml
<RecoveryNode name="NavigateRecovery" number_of_retries="6">
  <PipelineSequence name="NavigateWithReplanning">
    <RateController hz="{planner_frequency}" reset_token="{goal_revision}">
      <RecoveryNode name="ComputePath" number_of_retries="1">
        <Sequence>
          <ComputePathToPose goal="{goal}" path="{raw_path}"/>
          <IsPathValid path="{raw_path}"/>
        </Sequence>
        <Sequence><Recovery recovery="{clear_costmap_id}"/></Sequence>
      </RecoveryNode>
    </RateController>
    <RateController hz="{smoother_frequency}" reset_token="{raw_path_revision}">
      <SmoothPath input_path="{raw_path}" path="{smoothed_path}"/>
    </RateController>
    <RecoveryNode name="FollowPath" number_of_retries="1">
      <FollowPath path="{smoothed_path}" frequency="{controller_frequency}"/>
      <Sequence><Recovery recovery="{clear_costmap_id}"/></Sequence>
    </RecoveryNode>
  </PipelineSequence>
  <ReactiveFallback name="RecoveryActions">
    <GoalUpdated goal="{goal}"/>
    <RecoverySequence name="RecoveryActions" reset_token="{goal_revision}">
      <Sequence><Recovery recovery="{spin_id}"/></Sequence>
      <Sequence><Recovery recovery="{wait_id}"/></Sequence>
      <Sequence><Recovery recovery="{backup_id}"/></Sequence>
    </RecoverySequence>
  </ReactiveFallback>
</RecoveryNode>
```

节点与执行层的协作（全部经 `NavigationRuntime::request*` 返回 TaskHandle）：

| BT 节点 | onStart | onRunning |
|---|---|---|
| ComputePathToPose | `requestPath`，注册 `onComplete → emitWakeUpSignal` | goal 端口变化 → 重发请求；session 校验（版本陈旧拒绝不适用于 planner，它自己是版本推进者）；成功写 `raw_path`，并推进 `raw_path_revision`（黑板）+ 实时版本 |
| SmoothPath | `requestSmoothing` | session 校验；输入签名与上次一致时直接 SUCCESS（跳过重复平滑）；成功写 `smoothed_path` 并 `incrementPlanVersion` |
| FollowPath | `requestExecution`（记录 submitted_plan_version） | **先查 task.ready()**；plan_version 变化 → `requestPlanUpdate`（in-place，不 cancel）——update miss 时不误报失败，重查已完成结果或下一 tick 重试 |
| Recovery | `requestRecovery`（cause=黑板 failure_context；支持按次覆盖端口 target_yaw/backup_distance/backup_speed/wait_duration） | 成功即 SUCCESS |

**版本纪律**：SmoothPath 成功后才 `incrementPlanVersion`；
FollowPath 发现有新版本时对运行中的 controller 执行 in-place `updatePlan`
（无 cancel/无新 Execution/无中间零速），机器人持续跟随旧路径直到新路径
落地——这就是"已重规划但机器人不跟随"问题的修复机制。

## 4. 控制器周期（ControllerExecution::run，独立线程）

每个周期（1/controller_frequency，默认 20Hz）：

1. cancel/stop 检查；`safetyCheck()`（costmap 地图族 = wrapper isCurrent）。
   **失败计入 controller_patience**（nav2 failure_tolerance 语义：<0 无限
   容忍、0 立即失败、>0 容忍窗口），超时以 **MAP_STALE 终态**上报，交 BT
   恢复分支（clear costmap）——不再无限零速循环。
2. `hasNewPlan()` → 原子取 `(plan, plan_version)` → `controller->setPlan` →
   **goal 位姿快照 + goal_checker/progress_checker reset** → applied 版本推进。
3. `RobotStateController::getRobotState`（位姿 = TF 最新；速度 = 最新
   odom，超 `velocity_timeout` 返回零速）；失败 → TRANSFORM_UNAVAILABLE 终态。
4. 进度度量（nav2 FollowPath feedback 语义）：speed、distance_remaining
   （最近点投影到路径末端）、estimated_time_remaining（平均速度 ETA），
   经 `on_exe_path_feedback` 插槽每周期发出。
5. `goal_checker->isGoalReached`（唯一到达判定）→ 到达：零速 + ARRIVED_GOAL。
6. `progress_checker->check`（无进展判定）→ 失败：NO_PROGRESS 终态。
7. `callController`（地图族 execution 覆写为 typed 直调，局部图加锁）→
   有效命令：清零后经 `on_velocity_command` 发布；**发布前复查
   cancel/stop**，防止恢复启动瞬间漏出一条旧命令。
8. 无效命令（非有限/超限）：清零发布，按 controller_patience 累计终态
   （TIMEOUT）。

## 5. 恢复（资源协议）

- **requestRecovery 独占**：CAS `recovery_exclusive`（true=占住）→ 校验
  recovery_id → `preemptAllControllers()`（非阻塞：cancel 内已同步发零速，
  任务自行收尾，BT tick 线程不阻塞等慢模块）→ acceptGoal →
  `task.onComplete(释放互斥)`（**listener 先于 future 发布**，杜绝"BT 已见
  完成但互斥未释放"的竞态窗口）。恢复期间其余 request* 返回
  `RESOURCE_BUSY` 结构化拒绝；RESOURCE_BUSY 归类为**可恢复**（瞬时态，
  交恢复分支重试）。
- RecoveryExecution 驱动模块统一协议 `start → tick×N → finish`，是恢复期间
  **唯一速度发布者**（模块 tick 只返回期望速度 optional，执行层校验
  finite+max_vel 后发布；终态一律先零速）。
- 恢复成功后外层 RecoveryNode `++retry_count` 并 `resetChildren()`，回到
  主任务重试；预算耗尽树返回 FAILURE，pilot 释放本尝试的树实例
  （`releaseTreeInstance`）——下一次 `beginNavigation` 会建一棵全新的树
  （planner 从零重新规划，也不继承任何节点状态）。

## 6. 完成与失败

- FollowPath 结果 `reached=true && error==NONE` 且 session 当前 → 整树
  SUCCESS → pilot 重置树状态并**进入空闲**（不再 tick；到达后若继续以
  tick 频率空跑, 树会立即重新"规划→平滑→到达"循环, 实测 ~100Hz 空转、
  plan_version 以 60/s 膨胀）。空闲期消费 halt 请求, 下一个
  `beginNavigation` 投递恢复 tick（nav2 navigator 完成后停树的等价物）。
- 任一分支结构性失败（如 stale session、预算耗尽）→ 树 FAILURE → 重置 →
  待命；全局动态层已按累计运动自动清理，下一次导航从干净状态开始。

## 与 nav2 的语义对照

| 场景 | nav2 | ayanami |
|---|---|---|
| 目标抢占 | preemption + 新树 | 树实例按尝试新建 + goal_revision reset_token + 邮箱 |
| 重规划 | RateController 1Hz | RateController hz 可配（1.0），单发模式由 reset_token 补偿 |
| 恢复 | RecoveryNode + ClearCostmap 服务 | RecoveryNode 语义一致 + 命令端口清图 |
| 代价图失同步 | 手动/恢复清除 | 累计旋转/平移自动清全局动态层 |
| error 传递 | error code 字段 | ErrorCode + FailureContext（黑板）+ 结构化拒绝 |
