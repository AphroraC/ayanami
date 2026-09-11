# ayanami 第二轮优化计划

日期:2026-09-11。基于第一轮分析报告与负责人决策逐条落地。所有构建/测试在 docker 容器 dynamicx 内执行(`docker exec -it dynamicx bash`),禁止在宿主机安装任何依赖。

## 0. 范围基线

### 本轮落地(负责人已确认)

| # | 工作包 | 决策 |
|---|---|---|
| W1 | robot_radius 单一来源 / skipSubmit 移除 / Smac→Astar 更名 / RecoverySequence XML 精简 | 按分析结论执行 |
| W2 | 行为树参数从 config 注入 | 树参数可配置 |
| W3 | failure_context 从共享黑板槽改为端口传递 | 端口化 |
| W4 | smoother 经全局回调插槽取图 + pathEquals 移除(无条件跟踪新路径) | 插槽化 |
| W5 | 导航级 OSCILLATING 触发 | 新增 |
| W6 | costmap 层插件化(so+json)、观测过滤下沉、server 只留"唯一写者调度 + TF" | 插件化 |
| W7 | 全局图 = static + obstacle + inflation;clear_costmap 清 global | 对齐 nav2 |
| W8 | TransformBridge 重构:删 RobotStateController / TransformStore / fast_tf | 彻底重构 |
| W9 | planner 短锁拷贝(不再全程持锁搜索) | 拷贝搜索 |
| W10 | 文档同步 | 收尾 |

### 明确暂不优化(防止范围蔓延)

- 抽象地图层的 map-free 宿主(PID 仍被迫持 map 指针)。
- 静态图运行时热更新 seam。
- 版本/revision 机器(session/plan_version/goal_revision/raw_path_revision)保持现状,仅移除死代码 skipSubmit。
- 默认控制器仍为 costmap_pid_controller(切换 DWB 另行决策)。

---

## W1 清理与改名(先行,互不依赖)

### W1a robot_radius 单一来源
- 现状:`costmap.robot_radius=0.12`、`costmap_global.inscribed_radius`、`costmap_local.inscribed_radius` 三处手写同一物理量。
- 改动:
  - `configureCoreMaps`(src/costmap/components/costmap_navigation_server.cpp)只读 `costmap.robot_radius` 构造圆形 footprint,内切/外接半径由 `LayeredCostmap::setFootprint` 推导(InflationLayer 已通过 `onFootprintChanged` 拿 master 的内切半径,无需额外改)。
  - 删除 `costmap_global.inscribed_radius`、`costmap_local.inscribed_radius` 配置键;`costmap.robot_radius` 成为唯一入口。
  - 同步 config_sentry 变体与 `config_consistency_test`。
- 验收:删掉 inscribed_radius 键后构建/测试全绿;unusedKeys 审计不再报 robot_radius。

### W1b 移除 skipSubmit 死钩子
- `include/ayanami/behavior_tree/action_nodes.hpp`:删除 `skipSubmit` 虚钩子、"SmoothPath signature skip" 注释及 `onStart` 中的调用点(无任何派生类 override)。

### W1c CostmapSmacPlanner → CostmapAstarPlanner
- 目录 `modules/costmap_smac_planner/` → `modules/costmap_astar_planner/`;类名 `CostmapSmacPlanner` → `CostmapAstarPlanner`;Param 结构体 `CostmapAstarPlannerParam`;库名 `libayanami_costmap_astar_planner.so`。
- 配置:`config/modules/costmap_smac_planner.json` → `costmap_astar_planner.json`(section 同名),`costmap_navigation.json::navigation.config_planner` 路径更新。
- CMake 目标、`tests/costmap_smac_planner_test.cpp` → `costmap_astar_planner_test.cpp`、docs/modules.md 同步。行为不变(本就是普通 A*,去名不符实)。

### W1d RecoverySequence 直挂 Recovery
- `config/BT/navigation.xml` 与 `config_sentry/BT/navigation.xml`:删除每个 recovery 外层的单子节点 `<Sequence>`,`<RecoverySequence>` 直接挂 `<Recovery recovery="{...}"/>`。
- `production_tree_recovery_test` 适配。

---

## W2 行为树参数从 config 注入

- 原则:默认树由 BT extension 拥有,参数也由 extension 从 config 读取后写入黑板;核心协议(blackboard_protocol.hpp)不膨胀。
- 改动:
  - `navigation_behavior_tree.cpp::provideBlackboardEntries` 增读 `config_navigation.json::navigation`:
    - `recovery_retries`(默认 6)→ 黑板 `nav_retries`
    - `planner_retry_budget`(默认 1)→ `planner_retries`
    - `follow_retry_budget`(默认 1)→ `follow_retries`
    - `oscillation_timeout`(默认 5.0,≤0 关闭)/`oscillation_distance`(默认 0.1)/`oscillation_angle`(默认 3.14)→ 同名黑板键(供 W5 使用)
  - navigation.xml:`number_of_retries="{nav_retries}"`、`number_of_retries="{planner_retries}"`、`number_of_retries="{follow_retries}"`;`requiredBlackboardEntries` 补登新键。
  - pilot 的 configure 缓存只保留核心协议频率三键,不再新增。
- 验收:改 config 中 retries 数值 → 行为树重试预算变化;config 缺键时用默认值并有 info 日志。

---

## W3 failure_context 端口化(替换共享黑板槽)

- 现状问题:失败原因走单一黑板键 `failure_context`,靠"失败节点写 → RecoveryNode 读 → RecoveryActionNode 读并清 → PipelineSequence 成功清"的隐式轮转协议,跨分支残留风险由各节点自觉维护。
- 目标设计(BT.CPP v4 端口):
  1. `FailureContext` 注册为 BT 端口类型(若 v4 要求则 `BT::RegisterType<FailureContext>()`)。
  2. 写侧:四个 NavigationActionNode 派生节点与 `IsPathValid`、W5 的 `OscillationGuard` 均声明 `BT::OutputPort<FailureContext>("failure_context")`;`writeFailureContext` 改为 `setOutput("failure_context", ctx)`(未绑定端口时为 no-op,第三方树不受影响),保留 spdlog 告警行。删除黑板写。
  3. 读侧:`RecoveryNode` 声明 `BT::InputPort<FailureContext>("failure_context")`,`childFailureRecoverable()` 改读端口;端口未绑定/缺省 → 保守视为可恢复。
  4. `RecoveryActionNode`:新增 `BT::InputPort<FailureContext>("cause")` 替代黑板读+清;删除消费后清空逻辑。
  5. `PipelineSequence`:删除成功路径清黑板槽的补丁(共享槽已不存在)。
  6. 默认树按分支绑定独立键,天然消除跨分支残留:
     ```xml
     <RecoveryNode name="ComputePath" failure_context="{compute_path_failure}" ...>
       <Sequence>
         <ComputePathToPose goal="{goal}" path="{raw_path}" failure_context="{compute_path_failure}"/>
         <IsPathValid path="{raw_path}" failure_context="{compute_path_failure}"/>
       </Sequence>
       <Recovery recovery="{clear_costmap_global_id}" cause="{compute_path_failure}"/>
     </RecoveryNode>
     ```
- 删除项:`blackboard_protocol.hpp` 的 `k_failure_context` 及 `requiredBlackboardEntries` 中对应项。
- 测试:`production_tree_recovery_test`、`recovery_exception_test`、`pipeline_sequence_scenarios_test` 适配;新增"跨分支残留隔离"用例(A 分支失败后 B 分支 RecoveryNode 不读到 A 的 context)。

---

## W4 smoother 经回调插槽取图 + controller 无条件跟踪新路径

### W4a 数据通道:订阅全局回调插槽
- 现状:`CostmapSavitzkySmoother` 在 `readGlobal{ readLocal{ 平滑+碰撞检查 } }` 内同时压住两张图的共享锁,违背"<10ms 持锁"契约;10Hz 定时重跑进一步放大大盘饥饿。
- 改动:
  1. 模块构造/configure 时订阅 `NavigationCallbacks::on_costmap_visualization`(全量网格)+ `on_costmap_visualization_update`(增量窗口),按地图名维护自己的快照缓存(全量打底 + 应用增量窗口;无缓存的增量按契约丢弃)。订阅回调在宿主 spinOnce 线程执行:锁内拷贝/应用,锁外释放。
  2. `smoothPath` 在自己的 execution 线程对快照做碰撞检查,零地图锁;`configure(context, map)` 家族注入点保留(参数注入用),但不再持有/使用 map 指针读图。路径变换到各网格系所需的 TF 改经 TransformBridge(W8 之后可用)。
  3. 无任何快照(从未收到全量)时 `smoothPath` 返回结构化失败(不静默跳过碰撞检查)。
  4. 宿主 `CostmapNavigation` 的发布节奏(`costmap_local.visualization_rate` 等)成为 smoother 数据新鲜度上界:配置注释明确"local 流速率 ≥ smoother 碰撞检查需求";本轮不改发布机制。
- 契约修订:`map-and-costmap.md` 中 smoother 的家族读契约从"GLOBAL+LOCAL 直接读"改为"快照订阅"。

### W4b pathEquals 移除:smoother 有输出,controller 无条件跟踪
- `SmoothPathNode`:删除 `last_smoothed/has_last_smoothed/pathEquals` 及内容签名跳过;`onSuccessImpl` 无条件 `setOutput("path", ...)` + `incrementPlanVersion()` → FollowPath 每轮平滑产物都经 in-place updatePlan 下发。
- 连带:`skipSubmit` 已在 W1b 删除,注释同步。
- 配置建议:`smoother_frequency` 默认从 10.0 调整为 2.0(每轮必发版本,10Hz 无意义重平滑会白白推动控制器 setPlan;数值可调,写入配置注释)。
- 可选后续(P2,不本轮):smoother RateController 的 reset_token 绑定地图 revision(需 AbstractMapContext 暴露地图版本 seam),实现"图变了才重平滑"。

---

## W5 导航级 OSCILLATING 触发

- 目标:补 MBF 式导航级振荡检测——控制器周期性"成功"但机器人在窗口期内位移/旋转均低于阈值且超时,应触发 recovery ladder(控制器内部 progress checker 覆盖不到此类整体打转)。
- 设计:
  1. 新增核心 BT 条件节点 `OscillationGuard`(注册进 `registerNavigationBehaviorTreeNodes`,任何树可用):
     - 端口:`timeout`(黑板 `{oscillation_timeout}`)、`distance`、`angle`、`failure_context`(输出,W3)。
     - 语义:每次 tick 采样机器人位姿(TransformBridge 查 global←robot 最新);维护 (t,x,y,yaw) 窗口;当窗口跨度 ≥ timeout 且位移 < distance 且旋转 < angle → 写 FailureContext{OSCILLATION} 并返回 FAILURE,否则 SUCCESS。
     - `timeout ≤ 0` = 关闭(恒 SUCCESS);到位后的原地旋转调向窗口内可能低位移,默认 timeout=5s + angle 阈值放宽,现场可调(计划内注明调参注意事项)。
  2. 默认树把 FollowPath 包进 ReactiveSequence,振荡即打断管线进入 recovery:
     ```xml
     <RecoveryNode name="FollowPath" number_of_retries="{follow_retries}" failure_context="{follow_path_failure}" ...>
       <ReactiveSequence name="FollowPathGuarded">
         <OscillationGuard timeout="{oscillation_timeout}" distance="{oscillation_distance}"
                           angle="{oscillation_angle}" failure_context="{follow_path_failure}"/>
         <FollowPath path="{smoothed_path}" failure_context="{follow_path_failure}"/>
       </ReactiveSequence>
       <Recovery recovery="{clear_costmap_local_id}" cause="{follow_path_failure}"/>
     </RecoveryNode>
     ```
  3. 依赖:W8(位姿查询)、W3(失败上下文)、W2(参数下发)。
- 测试:新 `oscillation_guard_test`(时间注入,位移/旋转/关闭三分支)+ 生产树恢复链路用例。

---

## W6 costmap 层插件化 + 观测过滤下沉

### W6a 层组合可配置(so+json 模式)
- 目标:层列表配置化,沿用 modules/ 的装载姿势;`libayanami` 本体仍零 costmap 符号(`map_family_abstraction_test` 红线不动),层管理器放在 costmap 侧(libayanami_costmap_navigation)。
- 设计:
  1. 新接口 `ayanami::costmap::CostmapLayerPlugin : public virtual ExtensionModule`(kApiVersion=1),几何行为适配现有 `costmap_2d::Layer`(updateBounds/updateCosts/reset/matchSize/onFootprintChanged),参数经 `configureParameters(ModuleContext)` 读入自己的 `XxxLayerParam`。
  2. 内置三层(StaticLayer/SimpleObstacleLayer/InflationLayer)在本 .so 内经注册表自登记(DWB critics 的 registration.cpp 姿势),config 用 `type` 短名;自定义层支持 `so_name` dlopen(工厂符号统一 `create_extension_module`,走 ModuleManager 的 cast + apiVersion + 身份检查 + unusedKeys 审计)。
  3. 配置格式(过渡期向后兼容:缺 `layers` 键 = 维持现硬编码栈,下轮再删默认):
     ```json
     "costmap_global": {
       "layers": [
         {"name": "static",    "type": "StaticLayer",         "params": {"static_map": "config/assets/nav.pgm"}},
         {"name": "obstacles", "type": "SimpleObstacleLayer", "params": {"marking": true, "clearing": true, "obstacle_max_range": 4.0, "raytrace_max_range": 5.0, "min_obstacle_height": 0.0, "max_obstacle_height": 1.0}},
         {"name": "inflation", "type": "InflationLayer",      "params": {"inflation_radius": 0.5, "cost_scaling_factor": 10.0}}
       ]
     }
     ```
  4. 观测过滤参数下沉:`SimpleObstacleLayerParam` 承接 heights/ranges/marking/clearing/源选择(`sources`,缺省=全部);写者只做 TF(点云变换到各图 own frame)+ 交付原始点批,过滤由层自施。`ObservationSourceConfig` 收缩为传输信封 {name, kind, expected_update_rate}(活跃度检查属观测流,留在 server)。
  5. server 收缩为:观测 intake(转换/排队)、唯一写者循环(TF + 活跃度 + 逐层交付 + updateMap)、clear 命令执行。删除 server 内的过滤字段与 `prev_obstacles_local` 等层状态(下沉到 SimpleObstacleLayer)。
- 测试:层列表配置装配/未知 type 报错/自定义 .so 装载;过滤行为下沉后与现有 `costmap_layers_test`、`costmap_navigation_server_test` 对齐。

---

## W7 全局图动态障碍通道 + clear 清 global(对齐 nav2)

> 依赖 W6a 的层机制;与 W6b 是同一批改动,合并为一个 PR 序列。

- 全局图 = static + obstacle + inflation(默认配置即显式三层列表);local 图 = obstacle + inflation(滚动窗)不变。
- 写者管线扩展:`processObservation` 对 global 与 local 各做一次 TF(sensor→map / sensor→odom),分别交付两个障碍层(global 层配置独立的 marking/clearing/range 窗口,通常更大;global 层 `clear_to_free=false` 保持静态底图恢复语义)。全局图随观测触发 updateMap;全局 rolling 仍关闭。
- clear 语义:`ClearCostmapCommand` 对 global 不再退化为 no-op——清除 global 障碍层的动态标记后由 LayeredCostmap 重建窗口(static+inflation 自动恢复)。
  - 利用"同 .so 多逻辑名"机制(身份规则已支持)在 config_recovery 增加两个变体:`clear_global`(affected_maps:"global", role: clear_costmap_global)与 `clear_local`(affected_maps:"local", role: clear_costmap_local)。
  - 默认树对齐 nav2 形状:ComputePath 的 RecoveryNode 挂 `clear_costmap_global_id`,FollowPath 的 RecoveryNode 挂 `clear_costmap_local_id`(见 W3/W5 的 XML 片段);BT extension 的 role 表与 requiredBlackboardEntries 同步。
- 连带自动修复(无需改代码,回归确认):
  - `IsPathValid` 读 global 网格,动态障碍从此真实 invalidate 路径;
  - planner 重规划读到动态障碍(对齐 nav2 语义,行为变化是本项目的);
  - `recovery_vs_obstacle_test` 等场景按新语义更新。
- 配置:`costmap_global` 增加障碍层参数;`costmap_local.observation_sources` 保持声明,过滤窗口移入各层 params(W6a)。
- 测试:全局图打标/清除恢复静态/IsPathValid 动态失效/规划绕行动端到端;`headless_e2e_test` 补"走廊动态障碍 → 重规划绕行"用例。

---

## W8 TransformBridge 重构(删 RobotStateController / TransformStore / fast_tf)

- 目标形态:核心唯一的 TF 查询面 `ayanami::TransformBridge` 全局单例——精简、轻量、**内部零 buffer**,接口向 ayanami_ros 的 TF2Listener 对齐;ayanami_ros 把 ROS tf2_ros::Buffer 的能力经 Backend 适配器接入,核心按需查询时直通 tf_listener。
- 接口草案(include/ayanami/interfaces/transform_bridge.hpp;符号定义在 libayanami 的 .cpp,跨 .so 单例纪律同 NavigationCallbacks):
  ```cpp
  namespace ayanami {
  class TransformBridge {
  public:
    class Backend {                       // ayanami_ros 实现,包 TF2Listener
    public:
      virtual ~Backend() = default;
      virtual bool canTransform(const std::string& target, const std::string& source,
                                Timestamp stamp, double timeout_s) = 0;
      virtual bool lookupTransform(const std::string& target, const std::string& source,
                                   Timestamp stamp, Eigen::Isometry3d& out,
                                   double timeout_s, Timestamp* resolved_stamp = nullptr) = 0;
      virtual std::string framesAsString() { return {}; }   // 诊断直通,可缺省
    };
    static TransformBridge& instance();
    void installBackend(std::shared_ptr<Backend> backend);   // 启动时装一次;重复 install 记 warning 并替换
    bool canTransform(const std::string& target, const std::string& source,
                      Timestamp stamp, double timeout_s);
    bool lookupTransform(const std::string& target, const std::string& source,
                         Timestamp stamp, Eigen::Isometry3d& out,
                         double timeout_s, Timestamp* resolved_stamp = nullptr);
  };
  }
  ```
  - `Timestamp::latest()` = latest 语义,ROS 适配器映射 `ros::Time(0)`;timeout 语义与 TF2Listener 一致(0 = 不等待)。
  - 无 backend 时查询返回 false(无头/单测安全默认)。
- 速度状态的新家:`RobotStateController` 拆出的另一职责(最新里程计 + velocity_timeout 超龄清零)收敛为最小单例 `ayanami::RobotOdometry`(同样定义在 libayanami .cpp):`setOdometry(TwistStamped)` + `odometry(max_age)`(超龄返回零速)。ayanami_ros odomCallback 直写;控制器/恢复执行层消费。命名可在实施时再定,原则:与 TF 无关,不进 TransformBridge。
- ayanami_ros 侧:
  - 新增 `RosTransformBackend`(src/ayanami_ros/)包现有 `TF2Listener`,AyanamiROS 构造时、`loadNavigationModule` 之前 `TransformBridge::instance().installBackend(...)`。
  - **删除** 50Hz `syncTransformsToCore` 整套采样副本机制:edge 限频、future-stamp 钳制、stamp 窗口 fallback 计数、`tf_missing_since` 诊断——这些全是"核心持副本"模型的对症药,直通 tf2 后不复存在(tf2 原生支持 stamp 查询 + 插值 + latest)。
  - config_ros.json:删 `tf_sync_edges`/`tf_static_edges`/`tf_sync_rate`;`tf_cache_duration` 语义变为 tf2 Buffer 的 cache time(构造 TF2Listener 时传入);保留 map/odom/robot_frame(`checkTfHealth` 与一致性校验仍用)。`checkTfHealth` 保留,经 Backend 直查。
- 核心侧迁移点(grep `RobotStateController` 全量替换):
  - `CostmapMapContext::robotPoseInFrame` / `buildReader`(锁前解析位姿,改调 TransformBridge,锁序纪律不变);
  - planner/controller/recovery/smoother 执行层的位姿+速度读取(位姿=桥,速度=RobotOdometry);
  - `navigation_server_base.cpp`、`costmap_navigation_server.cpp` 中的引用。
- 删除项:`robot_state_controller.{hpp,cpp}`、`transform_store.{hpp,cpp}`、`thirdparty/fast_tf/` 及全部 CMake 引用。
- 测试:
  - 新 `tests/modules/stub_tf_backend.{hpp,cpp}`(内存 map:setTransform/按 stamp 查/latest=最新;不做插值)供全部旧 TF 注入型测试迁移;
  - 新 `transform_bridge_test`(install/未 install/跨 .so 单例性);
  - 删 `transform_store_test`、`robot_state_controller_test`、`robot_state_controller_cross_dso_test`;
  - 其余位姿注入型测试(headless_e2e、local_window_rolling、continuous_goal、controller_session、costmap 系)统一改 stub backend。
- 实施顺序(单分支分步提交):① 引入桥 + RobotOdometry + ROS Backend(旧件并存)→ ② 全量迁移调用点与测试 → ③ 删除三旧件与 fast_tf → ④ config/文档清理。

---

## W9 planner 短锁拷贝搜索

- 目标:A* 不再在共享锁内全程搜索;`readGlobal` 回调内只做一次 ROI 拷贝,锁释放后在副本上搜索。全局图引入动态层(W7)后写者更忙,此项同时保护写者。
- 设计:
  - `makePlan`:`readGlobal` 内按 start/goal 包围盒 + margin(≥ 膨胀半径,缺省整图 clamp)拷贝 `CostmapGridView {std::vector<unsigned char> costs; uint32_t size_x,y; double resolution, origin_x, origin_y}`(整图 1000×1000 ≈ 1MB,memcpy 亚毫秒级);ROI 超过整图 ~80% 或整图 > 64MB 时退回锁内搜索(契约写进头文件注释)。
  - `makePlanWithMap` 拆为 `copyView(readGlobal 内)` + `searchOnView(锁外)`,A* 逻辑本体不变;worldToMap 由视图自持元数据完成。
  - `planner_writer_concurrency_test` 更新:规划期间写者最大等待显著低于现阈值(拷贝语义断言)。
- 验收:基准 200×200 图上写者阻塞从 O(搜索时长) 降到 O(拷贝) 量级;规划结果与拷贝前逐路径一致(确定性测试)。

---

## W10 文档同步(收尾统一校对)

- `architecture.md`:删除不存在的 `composePlanningOverlay()` 表述;TF 一节改写为 TransformBridge 直通模型;层插件机制与"唯一写者"职责表更新;线程模型表(50Hz 同步循环删除后的 ROS 线程职责)。
- `map-and-costmap.md`:global 三层、clear 语义、robot_radius 单源、smoother 快照契约。
- `behavior-tree.md`:failure_context 端口协议、OscillationGuard、config 注入参数清单。
- `ros-integration.md`:Backend 接入、config_ros.json 键变更。
- `configuration.md`/`modules.md`/`README.md`:全部新键与更名。

---

## 执行顺序与依赖

```
Phase 0  W1a W1b W1c W1d          (独立小改,先清场)
Phase 1  W8 TransformBridge        (面宽,尽早合入,避免与后续包反复 rebase)
Phase 2  W3 → W2 → W5 → W4 → W9    (W5 依赖 W3/W2/W8;W4 依赖 W8;W9 依赖 W1c)
Phase 3  W6a → W7                  (层机制先于全局动态层)
Phase 4  W10 文档 + 全量回归
```

说明:W9 依赖 W1c(更名后实施);W5 依赖 W3(失败上下文端口)+W2(参数下发)+W8(位姿查询);W7 依赖 W6a(层配置机制);W4a 依赖 W8(路径跨系变换)。Phase 2 内部各包可并行分支。

## 验证纪律(每个 Phase 收尾,容器内执行)

```bash
docker exec -it dynamicx bash
cd /share/nav_ws && catkin build ayanami ayanami_ros
catkin test ayanami ayanami_ros        # 或 ctest --output-on-failure
```

- 全量回归基线:现有 52 项 gtest + ayanami_ros 测试 + `headless_e2e_test`。
- W8 落地后跑一次仿真链路(TF→stamping→规划→控制)人工冒烟,确认 tf2 直通语义与旧副本模型等价。

## 风险与回滚

| 风险 | 缓解 |
|---|---|
| W8 触及测试面最广 | 分步提交(引入→迁移→删除),stub backend 最早就位;旧件并存期随时可停 |
| W7 后 planner 行为变化(动态障碍进全局图) | 属预期变化;端到端用例先行定义,旧场景断言逐条迁移 |
| W4a smoother 新鲜度受宿主发布节奏约束 | 配置注释显式声明耦合;无快照时结构化失败,不静默放行 |
| W5 原地调向误触发 | timeout/angle 阈值可调,timeout≤0 关闭;默认值经 headless 仿真标定 |
| BT.CPP v4 自定义端口类型限制 | W3 首个提交先做类型注册验证,再铺开写侧/读侧改造 |
