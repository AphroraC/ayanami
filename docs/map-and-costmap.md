# 地图族抽象与 Costmap

## 地图族抽象（Map Family）

核心设计目标：**核心服务器不知道 CostmapContext，只知道地图族工厂**。

```
NavigationCore
    ├── AbstractMapContext        (map/abstract_map_context.hpp)
    │     ├── map identity        (mapFamily())
    │     └── write command port  (commandPort())
    │       （刻意最小：其余一切为地图族细节，见下）
    ├── AbstractPlanner / AbstractSmoother / AbstractController / AbstractRecovery
    │     （抽象层零地图类型）
    └── MapFamilyFactory          (map/map_family_factory.hpp)
          └── CostmapMapFamilyFactory (costmap 族实现，context 唯一创建点)
                └── CostmapMapContext : AbstractMapContext
                      ├── global costmap（global_costmap_frame，通常 map）
                      ├── local costmap（local_costmap_frame，通常 odom）
                      ├── footprint（configure 期不可变缓存）
                      ├── obstacle update（handleObstacle* 落图管线）
                      ├── composePlanningOverlay()（规划输入物化）
                      └── costmap clear command（CostmapCommandPort）
```

## 帧模型（nav2 惯例：双帧）

- **global costmap** 建 在 `costmap.global_costmap_frame`（通常 `map`）：
  静态层 + 全局动态层。
- **local costmap** 建在 `costmap.local_costmap_frame`（通常 `odom`）：
  滚动窗口动态层。局部图位于里程计系，**对定位（AMCL）跳变免疫**——
  校正移动的是 map←odom 边，不影响 local 标记所依据的 odom←sensor 边，
  短时局部几何保持一致（`onLocalizationJump()` 因此只清全局动态层）。
- 两个 grid 各自的帧以 wrapper 自身记录为准；TF 查询统一走
  `RobotStateController` 全局单例（框架唯一 TF 查询面，不再注入
  TransformLookup，也不另设 getRobotPose 类接口）。

## 读图契约（全地图族约定）

- **planner 读 global+local 的叠加**（costmap 族 =
  `composePlanningOverlay(buffer)`）：合成阶段同时持有两把 shared 锁
  （获取顺序 local→global），逐 cell 取 max(global, local)，local 仅在
  窗口内参与，NO_INFORMATION 保守占优；local←global 变换**每次合成只
  解析一次**（缓存矩阵逐 cell 应用，搜索过程零变换）。**合成完成即释放
  全部锁**，A* 事后只查 buffer（O(1)/cell、无锁）——搜索再慢（不可达
  目标/大图）也不会阻塞写者落图。锁内耗时被钉在 O(N) 合成上，"<10ms"
  由机制保证而非纪律约定。
- **smoother 只读 local**（`lockLocal()`）；路径若在另一帧（生产：map），
  grid←path 变换**每次 smooth 只解析一次**，输出路径保持原帧。
- **controller 读 local**（dwb 每 cycle `lockLocal()`；pid 不读图）。
  局部图帧可能与路径/位姿帧不同：dwb 在插件内经 RSC 变换（已有逻辑）。
- CostmapView = RAII shared 锁 + 零拷贝 master grid 指针，**只用于短时
  调用**；视图禁止存为长生命周期成员（历史事故，见
  troubleshooting.md）。规划器不走视图，走 overlay 物化。
- 机器人状态查询统一走 `RobotStateController`（视图内 robot_pose 是
  构建期按视图帧解析的缓存）。
- map version 仍然不做：新鲜度只有 `isCurrent()`。

## 族级插件接口

在抽象接口之上只增加注入点：
`configure(const PluginContext&, CostmapMapContext::ConstPtr)`
（对标 MBF `CostmapPlanner : AbstractPlanner` + initialize(name, costmap)）。

`NavigationServerBase` 通过虚钩子 `initialize*Plugin` 装配，**本体不含任何
costmap 头**；`map_family_abstraction_test` 用一个 Grid 假族证明该接缝。
未来 mesh/voxel 族：平行实现 `{Family}MapContext`、族级插件接口与
`{Family}MapFamilyFactory` 即可，族间互不感知。

## Costmap 数据流水线（costmap 族）

```
/scan /pointcloud (ROS) ──► CostmapNavigationServer::handleObstacle*
   └─ markObstaclesInFrame(frame_id, stamp, points, clearing_points)
        ├─ 逐图各自帧的 stamp-time 变换查询（local←laser / global←laser，
        │  零容差，失败立即回退 latest 并计数，节流日志暴露回退率）
        ├─ 位姿解析在拿写锁**之前**（local 用 local_frame←robot，
        │  global 用 global_frame←robot；失败回退各自上一帧成功值）
        ├─ unique_lock(wrapper)
        │    ├─ raytraceFreespace（光束清除）
        │    ├─ clearArea(该图上一帧观察窗 + 膨胀带外扩)   # Ruling #7
        │    ├─ markObstacles
        │    └─ updateMapUnlocked(robot_x, robot_y, robot_yaw)  # 滚动居中
        └─ 两图顺序落图（local 先、global 后，与读者锁序一致）
```

- **唯一写者**：CostmapNavigationServer 是地图的唯一写者；插件与 BT 只读。
- 写命令端口：`CostmapCommandPort : MapCommandPort`
  （`ClearCostmapCommand{affected_maps, clear_radius, enabled}`），恢复插件
  与定位跳变处理都经端口提交，由 server 的 `executeClearCommand` 执行
  （各图按自己的 frame 解析清图中心）。
- Ruling #7：InflationLayer 只做 `cost > old` 覆盖，障碍消失不会自动去膨胀
  ——动态更新前必须先 clearArea 上一帧观察窗（外扩 inflation+1 cell）。
- **落图时序（scan-stamp 查询）**：传感器变换按 **scan 自身时间戳**查询
  （零容差，失败立即回退 latest 并计数）。机器人位姿（滚动居中/清图中
  心）仍用 latest。fast_tf 动态边回溯窗口由 `tf_cache_duration`（默认
  10s）配置。
- **滚动窗口与传感器流解耦**：宿主 `spinOnce()` 周期调用
  `CostmapNavigationServer::updateRollingWindows()`——local 按
  local_frame←robot 居中，global（若配置滚动）按 global_frame←robot。

## 分层与组件

| 组件 | 文件 | 职责 |
|---|---|---|
| Costmap2D | `packages/costmap_2d` | 主 grid、worldToMap、updateOrigin（滚动平移）、resetMap |
| LayeredCostmap | 同上 | 层列表 + updateMap：rolling→updateOrigin；updateBounds 收集脏窗；resetMap 脏窗；updateCosts |
| StaticLayer | `packages/costmap_layers` | PGM+JSON 元数据加载；clear 后 addUpdateBounds 恢复 |
| SimpleObstacleLayer | 同上 | markObstacles（延迟标记列表）/raytraceFreespace（Bresenham 清除）/clearArea；组合方式 MAXIMUM（全局）/OVERWRITE（局部） |
| InflationLayer | 同上 | 只做 `cost > old` 膨胀覆盖 |
| CostmapWrapper | `packages/costmap_server` | LayeredCostmap + `std::shared_mutex` + active/current + updateMap(Unlocked)；自身记录帧名 |
| CostmapView | `ayanami/costmap/core/costmap_view.hpp` | 零拷贝 RAII 读视图（shared_lock + master grid 指针 + 帧名（取 wrapper）/footprint/robot_pose/current） |
| PlanningOverlay | `ayanami/costmap/core/planning_overlay.hpp` | 规划输入物化：叠加代价 buffer + 全局 grid 几何 + 机器人位姿；消费者无锁 O(1) 查询 |
| CostmapMapContext | `ayanami/costmap/components` | 双帧 wrapper 聚合 + lockLocal/lockGlobal + composePlanningOverlay + isPathValid + commandPort + footprint 缓存 |

## 锁与所有权纪律

- 单写者多读者：写者（server）持 `unique_lock`；读者持 `shared_lock`。
- 插件在 `configure` 时持有 `CostmapMapContext::ConstPtr`（const 智能指
  针），**绝不改图**；短时读图经 `lockLocal()/lockGlobal()` 取视图；规划
  器经 `composePlanningOverlay()` 物化后无锁搜索。视图存活期即持锁期，
  **禁止把视图存成长生命周期成员**（会把写者永久挡在门外——历史事故：
  dwb 存 `local_view` 成员导致落图/滚动永久阻塞）。
- 可变访问器（`globalCostmap()/localCostmap()`）只提供给地图写者与
  execution lifecycle（checkActivate/checkDeactivate/isCurrent）。
- 位姿解析在拿地图**写**锁之前完成（避免 RSC 锁与地图写锁嵌套）；
  shared 锁读者视图内解析（map 锁→RSC 锁单向嵌套，无反向路径，无环）。
- footprint：configure 期缓存不可变副本，读者只读缓存，绝不读 live
  LayeredCostmap。

## isPathValid 语义（nav2 planner_server 对齐）

`CostmapMapContext::isPathValid`（BT `IsPathValid` 条件的最终实现）：
- 经 `composePlanningOverlay` 取叠加（锁只住合成期），**从距机器人最近
  的路径点开始检查**——已走过路段可能被新障碍占据，属误判源，nav2 显式
  跳过；
- 每点须在界内（出界对标 nav2 按 lethal 处理），代价非 NO_INFORMATION 且
  非 lethal（`isLethal >= 253` 含 inscribed，对标 nav2 use_radius 模式）；
- 机器人位姿不可用时退化为全路径检查（保守）；
- 无 global wrapper 时退化为非空检查；local 变换不可用时 overlay 退化为
  global-only（`local_overlay_valid=false`）。

## 机器人状态来源

- `RobotStateController`（跨 DSO 单例）是**唯一的位姿/速度/TF 查询点**：
  位姿 = TransformStore 最新 `global_frame←robot`；速度 = 最新 odom
  （`velocity_timeout` 过期返回零速但位姿仍有效）。**所有需要查询 TF 的
  地方直接 include 并使用该单例**，不要再自创 getRobotPose 之类的包装。
- 写入端：ayanami_ros 把 `tf_sync_edges` 的 ROS TF 同步进 TransformStore；
  odom twist 直写 `setOdometry`。
- TF 注意事项：fast_tf 禁止双向外推。查询分两类：位姿类（控制周期位姿/
  滚动居中/清图中心）用 `Timestamp::latest()`；观测类（障碍落图）用
  传感器 header.stamp 精确回溯（窗口 `tf_cache_duration`，默认 10s）。
  fast_tf 动态边 `set` 对同一时间戳不覆盖（emplace 语义）；`set` 到
  "新父帧+已存在子帧"需经已存在的中间帧挂载（与 50Hz 同步的边序一致），
  直接挂会抛 "cannot merge unconnected trees"。
