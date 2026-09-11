# ayanami 优化计划(2026-09)

依据 2026-09-11 架构评审结论与已确认的决策清单制定。所有实施在容器内进行:
`docker exec -it dynamicx bash`,工作区挂载于 `/share/nav_ws`。

## 0. 决策清单与范围

| # | 评审发现 | 决策 | 阶段 |
|---|---|---|---|
| 1 | 全局图无动态障碍通道,重规划失明 | **对齐 nav2**:global = static + obstacle + inflation;clear_costmap 清 global | Phase 1 |
| 2 | costmap 层不是插件、观测策略在 server | **层组合配置化**(so+json 模式);观测过滤下沉障碍层;server 只留"唯一写者调度 + TF" | Phase 1 |
| 3 | robot_radius 三处配置双源 | 统一由 `costmap.robot_radius` 单源推导 | Phase 1 |
| 4 | smoother 嵌套双共享锁 + 定时重跑 | smoother **订阅全局回调插槽**获取 costmap 内容,不再持锁 | Phase 2 |
| 5 | SmoothPathNode::pathEquals 漏朝向 | **删除等价判断**:smoother 每次输出结果,controller 无条件跟踪 | Phase 2 |
| 6 | CostmapSmacPlanner 名不副实 | 更名 **CostmapAstarPlanner**,只保留普通 A* | Phase 3 |
| 7 | planner 全程持共享锁搜索 | 规划前**短锁拷贝**,搜索无锁 | Phase 3 |
| 8 | failure_context 单槽协议脆弱 | FailureContext 随**端口**传递,不走共享黑板槽 | Phase 4 |
| 9 | 树参数硬编码在 XML | 树参数可从 config 注入 | Phase 4 |
| 10 | 恢复树形状与 nav2 有差异 | RecoverySequence 直接挂 Recovery;加入导航级 **OSCILLATING** 触发 | Phase 4 |
| 11 | skipSubmit 死钩子与文档漂移 | 移除该钩子及其注释 | Phase 4 |
| 12 | TF 桥为自造 50Hz 采样副本 | **彻底重构**:删除 RobotStateController/TransformStore,统一为 `ayanami::TransformFrameListener` 全局单例;查询需要时由 ayanami_ros 调 ros tf2 查询并经单例回馈核心 | Phase 5 |
| — | 抽象地图分支无宿主 | 暂不优化 | — |
| — | 地图无运行时更新 seam | 暂不优化 | — |
| — | 默认控制器为无避障 PID | 暂不优化(保持现状) | — |
| — | 版本/revision 机器过重、requestPathUpdate 死路径 | 暂不优化 | — |

---

## Phase 1 地图层重构(P0)

阶段内顺序:1.4 → 1.2 → 1.3 → 1.1(全局动态通道落在插件化的层组合之上)。

### 1.4 robot_radius 单源化(先行,改动最小)

- 删除 `costmap_global.inscribed_radius`、`costmap_local.inscribed_radius` 配置项。
- `CostmapNavigationServer::configureCoreMaps` 只读 `costmap.robot_radius`,构造圆形
  footprint(`Footprint` 采样圆周),对 global/local 两个 `LayeredCostmap` 调
  `setFootprint()`(inscribed/circumscribed 由 LayeredCostmap 推导,InflationLayer 经
  `onFootprintChanged` 自动跟随)。
- 同步 `config_sentry/` 镜像配置与 `config_consistency_test`。

### 1.2 层组合配置化(so+json 模式)

**新增接口** `include/ayanami/costmap/core/costmap_layer_module.hpp`:

```cpp
class CostmapLayerModule : public ExtensionModule {
public:
  static constexpr std::uint32_t kApiVersion = 1;
  std::uint32_t apiVersion() const override { return kApiVersion; }
  // 交付底层 costmap_2d::Layer;server 负责加入 LayeredCostmap
  virtual std::shared_ptr<costmap_2d::Layer> layer() = 0;
};
```

- 适配器模式:`packages/costmap_layers` 保持纯 C++ 不依赖 ayanami 头;每个内置层新增
  薄适配 .so(`libayanami_costmap_static_layer.so` 等,module.cpp 导出
  `create_extension_module`,返回持有 Layer 的 CostmapLayerModule 适配器),与
  `modules/*/module.cpp` 模式一致。
- **配置形态**(复用 `Config::child`,不新增文件):`costmap_navigation.json` 中

```json
"costmap_global": {
  "layers": ["static_layer", "obstacle_layer", "inflation_layer"],
  "static_layer":   { "so_name": "libayanami_costmap_static_layer.so", "...": "params" },
  "obstacle_layer": { "so_name": "libayanami_costmap_obstacle_layer.so", "...": "params" },
  "inflation_layer":{ "so_name": "libayanami_costmap_inflation_layer.so", "inflation_radius": 0.5 }
}
```

  每层参数从 `child(section=costmap_global, key=<层名>)` 读入,享受 unusedKeys 审计。
- `configureCoreMaps` 改为:遍历 `layers` 列表 → dlopen 适配器 → apiVersion 检查 →
  `configureParameters` → `layer()` → `layered->addLayer()`。列表缺失/so 加载失败 =
  启动报错(与 BT XML 缺失同策略)。
- server 对障碍层的写通道通过 `dynamic_pointer_cast` 到已知障碍层类型识别(见 1.3);
  其余层一律视为纯合成层,server 不感知。
- 删除 configureCoreMaps 中的硬编码 `new StaticLayer/SimpleObstacleLayer/InflationLayer`。

### 1.3 观测过滤下沉障碍层

目标分工:**server = 唯一写者调度 + TF + 命令端口执行;障碍层 = 观测语义过滤 + 标记/
raytrace + 足迹清除 + 传感器活跃度**。

- `SimpleObstacleLayer` 扩展(与 nav2 ObstacleLayer 语义对齐):
  - 参数(自身层配置段内):`observation_sources` 列表 + 每源 child 块
    (`type/marking/clearing/min_obstacle_height/max_obstacle_height/obstacle_min_range/
    obstacle_max_range/raytrace_min_range/raytrace_max_range/expected_update_rate`),
    从 `costmap_local.observation_sources` 整体迁入。
  - 新 API:`observeObservation(source_name, map_frame_points, clearing_points,
    sensor_origin_xy)`——server 完成 TF 变换后调用;层内做 z 高度窗、源自距离窗、
    marking/clearing 过滤后进入 mark/raytrace。
  - 活跃度:层内维护每源 last-seen;`expected_update_rate` 超时 → 层 `isCurrent()` 返回
    false(现有 `LayeredCostmap::isCurrent` 链路自动传导为 MAP_STALE)。
  - 足迹清除:层在 `updateCosts` 内依据 updateMap 传入的 robot 位姿清自身足迹区
    (`footprint_clearing` 参数随层)。
- server 删除:`observation_sources` 解析、`source_last_seen/source_stale_warn_at`、
  `clearFootprintRegion`、`prev_obstacles_local/has_prev_obstacles`(标记状态由层自持,
  raytrace 即动态清除)、`findSource`。
- server 保留:writer 线程 + pending FIFO、`lookupWithBoundedWait`(TF)、
  rolling window 调度、`executeClearCommand`、map epoch 发布。
- 为支撑"内容是否变化"的精确判定(供 1.1/Phase 2 快照发射复用),`CostmapWrapper`
  增加 `uint64_t epoch`,在 `updateMapUnlocked` 实际重算窗口时自增。

### 1.1 全局图动态障碍通道(对齐 nav2)

- 默认层组合:`costmap_global.layers = ["static_layer", "obstacle_layer",
  "inflation_layer"]`;global 的障碍层同样挂在 `costmap_global` 配置段(观测源可与
  local 不同,例如 global 只 marking 不 clearing)。
- `processObservation`:TF 变换后**同时**喂给 global 与 local 的障碍层
  (各自按自己的源参数过滤);global 非滚动窗口,标记持久存在于 map 系(nav2 语义)。
- **clear_costmap 语义修正**:`ClearCostmapCommand` 针对 global 时,清除 global 障碍层
  的动态标记(层 `reset()`/clearArea),StaticLayer 在下次 updateCosts 恢复静态底图;
  `affected_maps="both"` 恢复其字面语义。删除"global 清除降级 no-op"的行为与注释。
- `CostmapMapContext::isPathValid` **无需改逻辑**(本就读全局主栅格,动态标记进入后
  自动生效);修正其注释中"local marks deliberately do NOT invalidate"的错误表述,以及
  `handleObstaclePointCloud` 头注释(改为与实现一致:两图都标记)。
- 回归守护:`local_window_rolling_test`(定位跳变语义)、
  `costmap_navigation_server_test`(新增 global 标记/清除断言)、`recovery_vs_obstacle_test`
  (障碍落在局部窗口外时重规划应绕行——新场景)、`headless_e2e_test`。

---

## Phase 2 smoother 解锁(P1)

### 2.1 原始 costmap 快照回调插槽

- 现有 `on_costmap_visualization` 的 OccupancyGrid 转换**有损**(lethal 折叠为 100),
  不能作为算法输入。新增无损通道:
  - `messages/nav_msgs.hpp` 新增 `CostmapRaw { frame_id, resolution, width, height,
    origin(Pose), stamp, epoch, std::vector<uint8_t> costs }`。
  - `NavigationCallbacks` 新增单槽
    `on_costmap_raw(const std::string& name, const CostmapRaw&)`(name = "global"/"local")。
- 发射点:**CostmapNavigationServer writer 线程**,在 `updateMapUnlocked` 完成后、仍持
  唯一写锁时,比对 `wrapper->epoch()` 与上次已发 epoch,变化则整幅快照发出(拷贝
  O(N),local 14KB/20Hz、global 仅变化时发,启动/清除后首帧必发)。发射即拷贝,
  订阅方约定"回调内只做轻量交换,不得阻塞"(写进 slots 文档)。
- rviz 有损可视化链路保持现状不动。

### 2.2 smoother 改为订阅快照

- `CostmapSavitzkySmoother`:
  - 成员持有两个 `CallbackSubscription`,订阅 `on_costmap_raw`,原子换入最新
    global/local 快照副本;
  - `smoothPath` 全程**无锁**:路径变换进快照 frame,在副本上做碰撞检查;
  - 快照未到达(启动初期)时降级为无碰撞检查平滑并 warn 一次;
  - `configure(context, map)` 家族签名保留,map 指针不再用于读取(帧信息取自快照)。
- 删除 `readGlobal{ readLocal{ ... } }` 嵌套持锁路径;`CostmapReader` 契约注释中
  把 smoother 从"GLOBAL+LOCAL 读能力"改为"经快照订阅"。

### 2.3 删除 pathEquals

- `SmoothPathNode::onSuccessImpl`:`setOutput("path", ...)` 后**无条件**
  `incrementPlanVersion()`;删除 `last_smoothed/has_last_smoothed/pathEquals`。
- 已知副作用(接受):`smoother_frequency=10` 下每次重平滑都会推进版本、触发一次
  controller in-place `updatePlan`(轻量:path 拷贝 + 版本号)。后续可另开议题用
  epoch 门控重平滑频率,本计划不做。

---

## Phase 3 planner(P1)

### 3.1 更名 CostmapAstarPlanner

- `modules/costmap_smac_planner/` → `modules/costmap_astar_planner/`;类
  `CostmapSmacPlanner` → `CostmapAstarPlanner`;目标 `libayanami_costmap_astar_planner.so`;
  `config/modules/costmap_smac_planner.json` → `costmap_astar_planner.json`(name/so_name/
  section 同步);`costmap_navigation.json::config_planner` 与 `config_sentry` 镜像;
  `costmap_smac_planner_test` → `costmap_astar_planner_test`;docs/modules.md。

### 3.2 规划前短锁拷贝

- `CostmapMapContext` 新增:
  `costmap_2d::Costmap2D snapshotGlobal()`——在 `readGlobal` 回调内整体拷贝主栅格
  (Costmap2D 数据为 `std::vector`,隐式拷贝即可;持锁时间 = memcpy,亚毫秒~毫秒级,
  有界且与图内容无关)。
- `CostmapAstarPlanner::makePlan`:先取快照与预解析机器人位姿,释放锁后在副本上跑
  A*(makePlanWithMap 改为 makePlanOnCopy)。写者与搜索彻底解耦,不再有"搜索拖住
  writer"的窗口。
- 回归:`planner_writer_concurrency_test`、`snapshot_stability_test`、
  `concurrent_controller_obstacle_test`。

---

## Phase 4 行为树改造(P1/P2)

### 4.1 FailureContext 端口化

- 所有失败写点(`NavigationActionNode::fail/failWithCurrent`、`IsPathValid`)改为通过
  **可选输出端口** `failure`(BT::OutputPort<FailureContext>)写上下文;端口未绑定 =
  不写(兼容现有测试树)。
- `RecoveryNode` 增加输入端口 `failure`(默认 ""):
  - 每轮 tick child0 **之前**清空该绑定键(轮转语义本地化);
  - child0 FAILURE 后立即经端口读回并分类(空 = 可恢复,保持现状);
  - 不清除,交由 child1 的 Recovery 读取后清除。
- `RecoveryActionNode` 的 cause 改为输入端口 `cause`,读后清除(删除黑板直读直写)。
- 删除 `k_failure_context` 共享协议键、`PipelineSequence` 成功路径的清槽逻辑、
  `blackboard_protocol` 中相应条目。
- XML 形态(同一 RecoveryNode 子树内同键绑定):

```xml
<RecoveryNode name="ComputePath" number_of_retries="1" failure="{compute_failure}">
  <Sequence>
    <ComputePathToPose goal="{goal}" path="{raw_path}" failure="{compute_failure}"/>
    <IsPathValid path="{raw_path}" failure="{compute_failure}"/>
  </Sequence>
  <Recovery recovery="{clear_costmap_id}" cause="{compute_failure}" failure="{compute_failure}"/>
</RecoveryNode>
```

- 回归:`production_tree_recovery_test`、`pipeline_sequence_scenarios_test`、
  `recovery_exception_test`、`behavior_tree_control_test`。

### 4.2 树参数 config 注入

- `navigation` 配置段新增:`navigate_retries(6)/compute_path_retries(1)/
  follow_path_retries(1)/oscillation_distance/oscillation_angle/oscillation_timeout`。
- pilot `writeRequiredBlackboardEntries` 注入同名黑板键;XML 改绑
  `number_of_retries="{navigate_retries}"` 等(XML 内保留字面默认值,键缺失时兜底)。

### 4.3 恢复树形状 + OSCILLATING 导航级触发

- XML 直挂:`RecoverySequence` 的每个子节点由 `<Sequence><Recovery/></Sequence>`
  改为直接 `<Recovery/>`;同步 `config_sentry/BT/navigation.xml`。
- 新增核心装饰节点 **`OscillationGuard`**(`behavior_tree/decorator`,注册进
  navigation.hpp 节点表,由 navigation_behavior_tree 扩展暴露):
  - 黑板参数:`oscillation_distance(默认 0.02m)/oscillation_angle(0.1rad)/
    oscillation_timeout(2s)`;
  - child RUNNING 期间每 tick 采样机器人位姿(TF 查询走 4.3 当时的姿态面,
    Phase 5 后统一为 TransformFrameListener);净位移与净转角在 timeout 内均低于
    阈值 → halt child、经输出端口写 `FailureContext{code=OSCILLATION}`、返回 FAILURE;
  - FAILURE 带 OSCILLATION 属可恢复 → 走现有 RecoveryNode 阶梯(对齐 MBF
    MoveBase 的 oscillation_timeout/oscillation_distance 语义,覆盖"反复规划-失败"
    式原地振荡);
  - child SUCCESS/位姿显著前进时重置窗口。
- FollowPath 分支包裹:`<OscillationGuard ...><FollowPath .../></OscillationGuard>`
  作为 FollowPath RecoveryNode 的 child0。

### 4.4 移除 skipSubmit

- 删除 `NavigationActionNode::skipSubmit` 钩子及其类文档中的"skipSubmit hook ->
  submit"描述(无任何派生类 override,纯死代码)。

---

## Phase 5 TF 桥彻底重构(P1,爆炸半径最大,放最后)

### 5.1 目标形态

`include/ayanami/interfaces/transform_frame_listener.hpp`:
`ayanami::TransformFrameListener` **全局单例**(实例定义在 libayanami 的
`src/interfaces/transform_frame_listener.cpp`,跨 DSO 唯一,沿用 NavigationCallbacks
的纪律),吸收并取代 RobotStateController + TransformStore 全部职责:

```cpp
class TransformFrameListener {
public:
  static TransformFrameListener& instance();
  // ---- TF(原 TransformStore + RobotStateController 合并) ----
  void configureCacheDuration(double seconds);
  void setTransform(parent, child, Timestamp, Isometry3d, bool is_static); // push 缓存
  bool lookupTransform(target, source, Timestamp, Isometry3d&, double timeout_s);
  // ---- 里程计(原 RobotStateController) ----
  void setOdometry(const TwistStamped&);
  bool latestOdometry(TwistStamped&, double time_tolerance_s) const;
  // ---- ROS 桥 seam(pull-through + 回馈) ----
  using TfProvider = std::function<bool(const std::string& target,
      const std::string& source, const Timestamp&, double timeout_s, Eigen::Isometry3d&)>;
  void setTfProvider(TfProvider provider);   // ayanami_ros 安装;测试装桩
  void clearTfProvider();
};
```

**查询路径**(满足"需要查询 tf 时由 ayanami_ros 调 ros tf2 查询并经单例回馈"):

1. `lookupTransform` → 若已装 provider:在调用者线程调 ROS tf2 查询(有界 timeout);
2. 成功:结果经 `setTransform` **回馈进核心缓存**(stamped 窗口),并返回——核心其它
   线程随后的 stamp 查询直接命中缓存;
3. 失败/provider 未装:回退内部 stamped 缓存查询(保留现有 stamp 窗口语义,
   bounded-wait 与 latest fallback 行为不变)。

ayanami_ros 侧:`TF2Listener` 保留为 ROS 订阅体;`syncTransformsToCore` 保留低频
push 预热(离线韧性 + 首帧覆盖),但传感器 stamp 预同步可简化(查询即拉取)。

### 5.2 三步迁移(每步可独立合入、ctest 全绿)

1. **外观步**:新建 TransformFrameListener,内部暂时转发到 RobotStateController +
   TransformStore;全仓库调用点(context.robotPoseInFrame、server 的
   lookupWithBoundedWait、controller/recovery 执行层速度源、AyanamiROS
   setTransform/setOdometry/configureCacheDuration)迁到新单例。行为零变化。
2. **seam 步**:加 `setTfProvider`;ayanami_ros 安装 provider(tf2 查询 + 成功回馈);
   测试装静态桩。保持 push 预热。
3. **收拢步**:删除 `robot_state_controller.{hpp,cpp}`、`transform_store.{hpp,cpp}`
   与旧缓存实现,缓存 + odom 收进新单例;
   `robot_state_controller_cross_dso_test` → `transform_frame_listener_cross_dso_test`,
   `transform_store_test`/`robot_state_controller_test` 合并重写;
   `headless_e2e_test` 等改用桩 provider/setTransform。

### 5.3 明确保留的语义(回归护栏)

- stamp 窗口查找、future-stamp 钳制(ayanami_ros 侧)、
  "先解析位姿再拿地图锁"的锁序纪律、TF 缺边 2s/5s 节流诊断、
  controller 的 `velocity_timeout` 位姿丢失宽限。

---

## Phase 6 配置/文档收尾

- `config_sentry/` 全量镜像同步(配置一致性测试守护)。
- 文档修正:`architecture.md`(删除不存在的 composePlanningOverlay 决策描述,补
  快照插槽/层插件化/TransformFrameListener 新决策)、`map-and-costmap.md`、
  `behavior-tree.md`(FailureContext 端口协议、OscillationGuard)、
  `ros-integration.md`、`troubleshooting.md`。
- 新增测试汇总:costmap 层插件加载测试、global 动态标记/清除测试、
  on_costmap_raw 快照语义测试、smoother 无锁订阅测试、OscillationGuard 单测、
  FailureContext 端口协议测试。

---

## 实施顺序与验证门

| 顺序 | 阶段 | 依赖 | 验证门 |
|---|---|---|---|
| 1 | Phase 1.4 → 1.2 → 1.3 → 1.1 | 无 | ayanami 52 项 ctest 全绿 + 新增用例 |
| 2 | Phase 2(2.1 → 2.2 → 2.3) | 无(独立) | 同上 + smoother 测试重写 |
| 3 | Phase 3(3.1 → 3.2) | 无(独立) | planner 测试更名后全绿 |
| 4 | Phase 4(4.1 → 4.2 → 4.3 → 4.4) | 无(4.3 姿态面暂用现单例) | BT 契约测试全绿 |
| 5 | Phase 5(三步) | 建议在 1-4 稳定后 | 全量 ctest + ayanami_ros 5 项(容器内 roscore,注意 11311 占用时 `roscore -p 11313` + `export ROS_MASTER_URI`) |
| 6 | Phase 6 | 全部 | config_consistency_test + 文档走查 |

常用命令(容器内):

```bash
cd /share/nav_ws && catkin build ayanami ayanami_ros
cd build/ayanami && ctest --output-on-failure
cd ../ayanami_ros && ctest --output-on-failure   # 需要 roscore
```

## 主要风险

1. **全局动态标记改变既有测试基线**:planner/IsPathValid 行为变"敏感",局部窗口外
   障碍会触发重规划与恢复阶梯——`headless_e2e`/`recovery_vs_obstacle` 场景需按新语义
   修订而非放宽。
2. **smoother 无条件推进版本**(决策 5)在 10Hz 重平滑下放大 controller
   updatePlan 频次:确认可接受;后续议题 = epoch 门控重平滑。
3. **Phase 5 单例迁移**:严格按三步走,第 1 步零行为变化是安全网;跨 DSO 单例必须
   定义在 libayanami .cpp。
4. **层插件化后配置向后兼容**:不做旧配置自动迁移,缺 `layers` 键 = 启动报错,
   配置显式升级(config_sentry 同步改)。
