# 地图族抽象、Costmap 与 ESDF

## 地图族抽象（Map Family）

核心设计目标：**核心服务器不知道 CostmapContext，只知道地图族工厂**；
FSM 宿主以普通成员组合地图机器（地图包不依赖 libayanami）。

```
宿主（rose_navigation_fsm / costmap_navigation_fsm）
    │     ├── map identity        (mapFamily())
    │       （刻意最小：其余一切为地图族细节，见下）
    ├── AbstractPlanner / AbstractSmoother / AbstractController / AbstractRecovery
    │     （抽象层零地图类型）
    │           └── CostmapMapContext（FSM 类成员，无族抽象基类）
    │                 ├── global costmap（static+inflation，通常 map 帧）
    │                 ├── local costmap（obstacle+inflation，滚动，odom 帧）
    │                 ├── readGlobal()/readLocal()（唯一读入口，回调作用域）
    │                 ├── snapshotGlobal()/snapshotLocal()（短锁快照）
    │                 ├── localFromGlobal()（变换一次解析，锁外）
    │                 ├── isPathValid()（FusedMapView 探针）
    │                 └── costmap clear command（CostmapCommandPort）
    └── [rose] ::esdfmap 包对象（EsdfMap + EsdfWriter，普通成员）
          EsdfmapSmoother 经 configure(context, esdf_map) 注入唯一读者句柄
```

## 包结构（packages/，均不依赖 libayanami）

| 包 | 内容 | 说明 |
|---|---|---|
| `packages/costmap` | 单库 `costmap::costmap`（libcostmap） | **三包合一**（原 costmap_2d + costmap_layers + costmap_server）：`costmap_2d.hpp`（Costmap2D/LayeredCostmap）、`layer.hpp`/`layered_costmap.hpp`（保留类抽象）、`static_layer.hpp`/`simple_obstacle_layer.hpp`/`inflation_layer.hpp`（三层）、`costmap_wrapper.hpp`（锁 + 帧 + generation） |
| `packages/esdfmap` | `esdfmap::esdfmap` 库 | EsdfMap（快照持有 + 查询契约）、EsdfWriter（写者）、edt.hpp（精确平方 EDT）、EsdfSnapshot（不可变快照）——Eigen-free，TF 经回调注入 |

## 层静态组合（dlopen 层插件已删除）

`costmap_layers_builder.hpp` 从路由 json 直接构建**固定组合**：

- **global 恒 StaticLayer + InflationLayer**（纯静态内容）——动态障碍只进
  局部图，全局图不存在需要定位跳变清理的动态层；
- **local 恒 SimpleObstacleLayer + InflationLayer**（滚动 + 动态标记）+
  观测源解析 + CostmapObstacleApplier。

层参数**内嵌**在路由 json 的 `costmap_global.static_layer/.inflation_layer`
与 `costmap_local.obstacle_layer/.inflation_layer` 子段；旧配置里逐层的
`so_name` 键与 `layers` 列表已不再被读取（组合是编译期固定的）。StaticLayer
接管了原静态插件的 onAddedToMap 职责：解析 static_map 路径（相对部署根）、
加载 PGM + `<pgm>.json` 元数据、按图定 master 尺寸（无图文件时回退
`size_x/size_y/resolution`，两者皆无 = 启动报错）。

层仍是类抽象（`layer.hpp` 的 Layer 基类 + LayeredCostmap 的
updateBounds/updateCosts 管线），只是不再经 dlopen 组合。

## 障碍 mask 读接口（供 ESDF 派生）

SimpleObstacleLayer 暴露 **presence 网格**读面（`presenceGrid()` + 几何
访问器），供宿主的 mask 读者在**地图共享锁内拷出**：

```
rose_navigation_fsm::readLocalMask()
  └─ std::shared_lock(wrapper->mutex())
       └─ cells = obstacle_layer->presenceGrid()   # 拷贝：写者只在本调用内消费
          view = {width, height, resolution, origin, frame, generation}
```

- `view.generation` = wrapper 的单调代数（同代数 = 字节相同内容），
  EsdfWriter 用它做**重建门控**；
- 全局 mask 读者走 `snapshotGlobal()`（短锁快照）并按
  `cost >= LETHAL_OBSTACLE` 二值化——只取 lethal：膨胀梯度是 costmap 的
  安全表达，不是障碍几何，ESDF 自己算距离。

## ESDF 派生流水线（packages/esdfmap，rose 路由）

```
local obstacle mask（共享锁拷出）──┐
                                  ├─ 融合：local mask OR global static mask
global static mask（短锁快照）─────┘   （static 经注入 TransformLookup 逐周期
                                        重投影——滚动窗 origin 在动）
        ▼
精确有符号 EDT（Felzenszwalb-Huttenlocher，逐列 + 逐行两遍 1D 变换；
正负两场：自由空间到障碍距离 / 障碍内到自由空间穿透深度，±max_distance 钳位）
        ▼
发布不可变 EsdfSnapshot（atomic shared_ptr 换；sequence 递增、
received = 发布时刻、附 occupied 二值栅格）
```

- **一写者多读者**：写者线程每周期（`update_rate` 上限）重建；读者
  `currentSnapshot()` **无锁**取当前快照并持有任意久——已发布快照不可变，
  快照交换即一致性保证（无读者锁、无回调视图）。
- **generation 门控**：源代数与栅格几何未变 → 整个重建跳过
  （`skippedRebuilds()` 可观测：健康部署大多数周期都在跳）。
- **新鲜度**：`EsdfMap::current()` = 快照存在且（`stale_timeout` > 0 时）
  年龄未超限；execution 层安全检查消费它，过期表现为 MAP_STALE。
- **场语义**：滚动窗外的点是 UNOBSERVED 而非 blocked（isPathValid 放行，
  查询返回 kOutOfField + max_distance）。

## 帧模型（nav2 惯例：双帧）

- **global costmap** 建在 `costmap.global_costmap_frame`（通常 `map`）：
  静态层 + 膨胀层（纯静态）。
- **local costmap** 建在 `costmap.local_costmap_frame`（通常 `odom`）：
  滚动窗口动态层。局部图位于里程计系，**对定位（AMCL）跳变免疫**——
  校正移动的是 map←odom 边，不影响 local 标记所依据的 odom←sensor 边。
- ESDF 场建在 map 系（rose 静态图模式）：BASE mask = 全局静态图（固定窗，定义场的几何与坐标系），滚动局部活障作为 OVERLAY mask 经注入的
  TransformLookup（rose 宿主实现为 RobotStateController::lookupTransform）。
- TF 查询统一走 `RobotStateController` 全局单例（框架唯一 TF 查询面）。

## 读图契约（全地图族约定）

- **planner 短锁快照**（`snapshotGlobal()`）：共享锁只住一次 master grid
  的 memcpy（`CostmapSnapshot`，含几何/帧/位姿/generation），A* 随后在
  不可变副本上搜索——**全程不持有地图锁**（astar 模块把 global 快照存为
  自己的成员副本；全局图纯静态，内容跨调用不变，快照可复用；local 侧每次
  规划重新快照，是新鲜动态标记的唯一来源）。
- **controller / recovery 读 local**：pid 控制器不读图（纯跟踪）；
  需要读图的控制器在模块内经 RSC 变换。
- **读接口只有一个**：`readGlobal(fn)` / `readLocal(fn)`，把
  `const CostmapReader&` 交给回调，锁在回调返回时释放。按值返回视图的
  `lockLocal()/lockGlobal()` 已删除——"把视图存成成员或塞进 lambda"曾
  静默把写者挡到对象析构（历史事故）。现在没有可保存的值。
- **`CostmapReader` 的 grid 指针只在回调内有效**：需要更久的算法走
  `snapshotGlobal()/snapshotLocal()` 快照。
- **位姿在加锁之前解析**：buildReader 先解析机器人位姿、再取地图锁
  （"持地图锁做 TF 查询"的锁序反转已修）。
- 同一次回调内需要两张图时按 local 外层、global 内层嵌套（与写者顺序一致）。

## 观测管线与唯一写者（costmap 族）

```
/scan /pointcloud (ROS) ──► 宿主 setPointcloud/setLaserScan
    └─ CostmapNavigationServer::handleObstacle*
         └─ 仅转换 + 入队（latest-wins 邮箱，O(enqueue)，调用线程不阻塞）
              ▼
        唯一写者线程（resident map-writer）逐批：
          ├─ stamp-time TF 有界等待（≤ transform_tolerance，写者内轮询；
          │  stamp-time 变换不齐的整帧丢弃，地图 stale-but-stable）
          ├─ 位姿解析在拿写锁之前（local 用 local_frame←robot）
          ├─ 静态区过滤（costmap_local.filter_static_marks：全局阻挡栅格内
          │  的局部命中不落图——static→local 变换在锁外解析）
          ├─ unique_lock(wrapper)
          │    ├─ raytraceFreespace（光束清除）
          │    ├─ clearArea（上一帧观察窗 + 膨胀带外扩）
          │    ├─ markObstacles
          │    └─ updateMapUnlocked（滚动居中）
          ├─ 滚动窗口更新（写者循环自驱动，不再依赖外部 spinOnce）
          ├─ clear 命令执行（经 CostmapCommandPort 提交，写者执行）
          ├─ 传感器活性检查（expected_update_rate > 0 的源静默超率 →
          │  地图 not-current，节流告警；下游 = MAP_STALE）
          └─ 层工件发射（generation 门控：内容变了才发）
```

- **唯一写者**：CostmapNavigationServer 是地图的唯一写者；模块与 FSM 只读。
- **Ruling #7 保留**：InflationLayer 只做 `cost > old` 覆盖，障碍消失不会
  自动去膨胀——动态更新前必须先 clearArea 上一帧观察窗。
- **层工件槽**：`on_global_costmap_static / on_global_costmap_inflation /
  on_local_costmap_obstacle（presence，无膨胀）/ on_local_costmap_inflation`
  ——唯一写者在内容变更时无条件发射（generation 门控，无任何频率参数），
  接收方自持缓存。

## isPathValid 语义（FSM 的路径校验探针）

`AbstractPlanner::isPathValid`（接口默认 = 非空检查）→
`costmap_astar_planner` 覆写为 `CostmapMapContext::isPathValid`
（nav2 planner_server 对齐 + static-global 策略感知）：

- **不走** `snapshotGlobal()` 全图拷贝：路径校验是 O(path) 的稀疏探针；
  local 侧取一次短锁快照 + 一次锁外变换（**锁不嵌套**），global 侧持
  共享锁做点扫描（短临界区）。
- **FusedMapView 融合**：全局格（static+inflation）与局部滚动快照逐点
  **MAX** 融合（两源代价不相交，各带自己的膨胀，无双重膨胀）；局部窗外
  的点由全局格单独把门。
- 检查从**距机器人最近的路径点**开始——已走过路段可能被新障碍占据，是
  误判源（nav2 显式跳过）；机器人旁、足印内切半径内的连续前缀豁免
  （机器人已在那里；否则蹭带行驶会让每次校验都失败，START_INVALID 死锁）。
- 每点须在界内（出界按 lethal），代价非 NO_INFORMATION 且非 lethal
  （`isLethal >= 253` 含 inscribed，对标 nav2 use_radius 模式）。
- 退化路径：无 global wrapper → 非空检查；local 图/变换缺失 →
  global-only；机器人位姿不可用 → 全路径检查（保守）。
- FSMDriver 经 `setPathValidator` 注入该探针；kPlan/kFollow 每次规划与
  每拍监督都消费它（失效即刻重规划）。

## 锁与所有权纪律

- costmap 单写者多读者：写者持 `unique_lock`；读者持 `shared_lock`；
  `wrapper->generation()` 单调递增（mask 门控 / 层工件发射的依据）。
- 模块在 `configure` 时持有 `CostmapMapContext::ConstPtr`，**绝不改图**；
  短时读图经 `readLocal()/readGlobal()` 回调；规划器经快照后无锁搜索。
  回调存活期即持锁期，**禁止把视图存成长生命周期成员**（历史事故：
  dwb 存 `local_view` 成员导致落图/滚动永久阻塞）。
- ESDF 一写者多读者：不可变快照原子换，无读锁；写者 mask 读取在 costmap
  共享锁内**拷出**，绝不在锁内做 EDT。
- 可变访问器（`globalCostmap()/localCostmap()`）只提供给地图写者与
  execution lifecycle（checkActivate/checkDeactivate/isCurrent）。
- 位姿解析在拿地图**写**锁之前完成；shared 锁读者视图内解析
  （map 锁→RSC 锁单向嵌套，无反向路径，无环）。

## 机器人状态来源

- `RobotStateController`（跨 DSO 单例）是**唯一的位姿/速度/TF 查询点**：
  位姿 = 注入的 tf2 源最新 `global_frame←robot`；速度 = 最新 odom
  （`velocity_timeout` 过期返回零速但位姿仍有效）。所有需要查询 TF 的地方
  直接 include 并使用该单例。
- TF 桥（按需查询模型）：ayanami_ros 启动时把 tf2 backed 的查询源注入
  `RobotStateController`（`setTransformSource`）；核心线程的
  `lookupTransform / canTransform / lookupPose` 按需直达 tf2 buffer——
  插值、静态边、stamp 时间查询全部由 tf2 提供。没有周期采样，没有核心侧
  变换缓存；内部仅保留 boost::circular_buffer 里程计。超时为 steady_clock
  轮询（不用 tf2 的 ros::Time 时钟等待，避免 /clock 停滞时挂死）。
