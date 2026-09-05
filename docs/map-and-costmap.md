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

## 机器人状态来源

- `RobotStateController`（跨 DSO 单例）是**唯一的位姿/速度查询点**：
  位姿 = TransformStore 最新 `global←robot`；速度 = 最新 odom
  （`velocity_timeout` 过期返回零速但位姿仍有效）。
- 写入端：ayanami_ros 50Hz 循环把 `tf_sync_edges` 的 ROS TF 同步进
  TransformStore；odom twist 直写 `setOdometry`。
- TF 注意事项：fast_tf 无外推，core 侧查询一律用 `Timestamp::latest()`
  （与 50Hz 同源），容差仅作兜底；fast_tf 动态边 `set` 对同一时间戳不覆盖
  （emplace 语义）；`set` 到"新父帧+已存在子帧"需经已存在的中间帧挂载
  （与 50Hz 同步的边序一致），直接挂会抛 "cannot merge unconnected trees"。
