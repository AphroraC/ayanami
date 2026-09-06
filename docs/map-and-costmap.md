# 地图族抽象与 Costmap

## 地图族抽象（Map Family）

核心设计目标：**核心服务器不知道 CostmapContext，只知道地图族工厂**。

```
NavigationCore
    ├── AbstractMapContext        (map/abstract_map_context.hpp)
    │     ├── map identity        (mapFamily())
    │     ├── coordinate frames   (globalFrame/localFrame/robotFrame)
    │     ├── robot state query   (getRobotState)
    │     ├── snapshot/version    (isCurrent/version)
    │     ├── collision/query     (isPathValid + capabilities)
    │     └── write command port  (commandPort)
    ├── AbstractPlanner / AbstractSmoother / AbstractController / AbstractRecovery
    │     （抽象层零地图类型）
    └── MapFamilyFactory          (map/map_family_factory.hpp)
          └── CostmapMapFamilyFactory (costmap 族实现，context 唯一创建点)
                └── CostmapMapContext : AbstractMapContext
                      ├── global costmap / local costmap (CostmapWrapper)
                      ├── footprint（configure 期不可变缓存）
                      ├── obstacle update（handleObstacle* 落图管线）
                      └── costmap clear command（CostmapCommandPort）
```

- 族级插件接口在抽象接口之上只增加注入点：
  `configure(const PluginContext&, CostmapMapContext::ConstPtr)`
  （对标 MBF `CostmapPlanner : AbstractPlanner` + initialize(name, costmap)）。
- `NavigationServerBase` 通过虚钩子 `initialize*Plugin` 装配，**本体不含任何
  costmap 头**；`map_family_abstraction_test` 用一个 Grid 假族证明该接缝。
- 未来 mesh/voxel 族：平行实现 `{Family}MapContext`、族级插件接口与
  `{Family}MapFamilyFactory` 即可，族间互不感知（已在
  `map/map_family_factory.hpp` 登记，不写空壳类）。

## Costmap 数据流水线（costmap 族）

```
/scan /pointcloud (ROS) ──► CostmapNavigationServer::handleObstacle*
   └─ markObstaclesInFrame(frame_id, stamp, points, clearing_points)
        ├─ lookupTransform(local_frame ← frame_id, latest, 0.2s)   # 障碍点→局部帧
        ├─ (Local) unique_lock(local wrapper)
        │    ├─ raytraceFreespace（光束清除）
        │    ├─ clearArea(上一帧观察窗 + 膨胀带外扩)   # Ruling #7
        │    ├─ markObstacles
        │    └─ updateMapUnlocked(robot_x, robot_y, robot_yaw)  # 滚动居中
        └─ (Global) 同构管线，使用 map←laser / map←base（AMCL 修正链）
```

- **唯一写者**：CostmapNavigationServer 是地图的唯一写者；插件与 BT 只读。
- 写命令端口：`CostmapCommandPort : MapCommandPort`
  （`ClearCostmapCommand{affected_maps, clear_radius, enabled}`），恢复插件
  与定位跳变处理都经端口提交，由 server 的 `executeClearCommand` 执行
  （`clear_radius<=0` = 清空整个动态层，静态层经 addUpdateBounds 恢复）。
- Ruling #7：InflationLayer 只做 `cost > old` 覆盖，障碍消失不会自动去膨胀
  ——动态更新前必须先 clearArea 上一帧观察窗（外扩 inflation+1 cell）。
- **落图时序（scan-stamp 查询）**：传感器变换（local←laser /
  global←laser）优先按 **scan 自身时间戳**查询（零容差，失败立即回退
  latest 并计数，节流日志暴露回退率）。旋转时 latest 查询会把
  ω×(now-T_scan) 的时序歪斜烙进代价地图（实测 1.2 rad/s 旋转时点云相邻帧
  漂移 0.13~0.28m；按戳落图后局部图漂移 ≤1 cell）。机器人位姿（滚动居中 /
  清图中心）仍用 latest。fast_tf 动态边回溯窗口由 `tf_cache_duration`
  （默认 10s）配置，scan 戳必须落在窗内，否则回退 latest。
- **滚动窗口与传感器流解耦**：宿主 `spinOnce()` 周期调用
  `CostmapNavigationServer::updateRollingWindows()`（频率
  `costmap_local.update_frequency`=10Hz / `costmap_global.update_frequency`=5Hz，
  仅对启用 rolling_window 的地图生效）——scan 断流时窗口仍跟随机器人。
  否则窗口冻结后 DWB 前进轨迹逐渐"出界"，机器人卡死在窗口边缘且
  current 标记不可检测（真实卡死案例）。

## 分层与组件

| 组件 | 文件 | 职责 |
|---|---|---|
| Costmap2D | `packages/costmap_2d` | 主 grid、worldToMap、updateOrigin（滚动平移）、resetMap |
| LayeredCostmap | 同上 | 层列表 + updateMap：rolling→updateOrigin；updateBounds 收集脏窗；**resetMap 脏窗**；updateCosts |
| StaticLayer | `packages/costmap_layers` | PGM+JSON 元数据加载；clear 后 addUpdateBounds 恢复 |
| SimpleObstacleLayer | 同上 | markObstacles（延迟标记列表）/raytraceFreespace（Bresenham 清除）/clearArea；组合方式 MAXIMUM（全局）/OVERWRITE（局部） |
| InflationLayer | 同上 | 只做 `cost > old` 膨胀覆盖 |
| CostmapWrapper | `packages/costmap_server` | LayeredCostmap + `std::shared_mutex` + active/current + updateMap(Unlocked) |
| CostmapView | `ayanami/costmap/core/costmap_view.hpp` | 零拷贝 RAII 读视图（shared_lock + master grid 指针 + frame/footprint/robot_pose/current） |
| CostmapMapContext | `ayanami/costmap/components` | global/local wrapper 聚合 + lockLocal/lockGlobal/readLocal/readGlobal + commandPort + footprint 缓存 |

## 锁与所有权纪律

- 单写者多读者：写者（server）持 `unique_lock`；读者持 `shared_lock`。
- 插件在 `configure` 时持有 `CostmapMapContext::ConstPtr`（const 智能指针），
  **绝不改图**；读图一律经 `lockLocal()/lockGlobal()` 取视图，视图存活期
  即持锁期，**禁止把视图存成长生命周期成员**（会把写者永久挡在门外——
  历史事故：dwb 存 `local_view` 成员导致落图/滚动永久阻塞）。
- 可变访问器（`globalCostmap()/localCostmap()`）只提供给地图写者与
  execution lifecycle（checkActivate/checkDeactivate/isCurrent）。
- 位姿解析在拿地图锁**之前**完成（避免 RSC 锁与地图锁嵌套）。
- footprint：configure 期缓存不可变副本，读者只读缓存，绝不读 live
  LayeredCostmap。

## isPathValid 语义（nav2 planner_server 对齐）

`CostmapMapContext::isPathValid`（BT `IsPathValid` 条件的最终实现）：
- **从距机器人最近的路径点开始检查**——已走过路段可能被新障碍占据，属
  误判源，nav2 显式跳过（planner_server.cpp:596 closest_point_index）；
- 每点须在界内（出界对标 nav2 按 lethal 处理），代价非 NO_INFORMATION 且
  非 lethal（`isLethal >= 253` 含 inscribed，对标 nav2 use_radius 模式）；
- 机器人位姿不可用时退化为全路径检查（保守）。

## 机器人状态来源

- `RobotStateController`（跨 DSO 单例）是**唯一的位姿/速度查询点**：
  位姿 = TransformStore 最新 `global←robot`；速度 = 最新 odom
  （`velocity_timeout` 过期返回零速但位姿仍有效）。
- 写入端：ayanami_ros 50Hz 循环把 `tf_sync_edges` 的 ROS TF 同步进
  TransformStore；odom twist 直写 `setOdometry`。
- TF 注意事项：fast_tf 禁止双向外推。查询分两类：位姿类（控制周期位姿/
  滚动居中/清图中心）用 `Timestamp::latest()`（多跳链路解析为 latest
  common time——tf2 语义的统一切刻采样）；观测类（障碍落图）用传感器
  header.stamp 精确回溯（窗口 `tf_cache_duration`，默认 10s，窗内插值）。
  fast_tf 动态边 `set` 对同一时间戳不覆盖（emplace 语义）；`set` 到
  "新父帧+已存在子帧"需经已存在的中间帧挂载（与 50Hz 同步的边序一致），
  直接挂会抛 "cannot merge unconnected trees"。
