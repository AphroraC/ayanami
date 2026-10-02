# 配置系统

## 文件结构与单一来源纪律

配置为多层 JSON，**每个键只有一个权威来源**：

```
config/
├── config.json               # 根索引：global 段（文件指针）+ logging 段
├── config_ros.json           # 仅 ayanami_ros 使用：topic / 帧名
├── config_extensions.json    # 扩展清单 + 每扩展参数段
├── routes/                   # 导航路由（每份 = 一个完整导航栈的描述）
│   ├── fsm_rose_navigation.json      # 默认：costmap + 派生 ESDF + 混合族模块
│   └── fsm_costmap_navigation.json   # 纯 costmap 栈 + costmap 族模块
├── assets/                   # nav.pgm + nav.json（静态地图元数据）
└── modules/                  # 每模块一个文件：<family> 段 + <name> 参数段
    ├── abstract_nothing_recovery.json
    ├── abstract_pid_controller.json
    ├── costmap_astar_planner.json
    ├── costmap_minsnap_smoother.json
    ├── costmap_mppi_controller.json
    ├── costmap_pid_controller.json
    ├── esdfmap_minco_smoother.json
    ├── simple_goal_checker.json
    └── simple_progress_checker.json
```

- `config.json::global` 只提供文件指针：`config_path`（目录）、
  `config_navigation`（**路由 json**，默认 `routes/fsm_rose_navigation.json`）、
  `config_ros`、`config_extensions`。不提供多实例语义（单机器人单导航）。
- 路由 json 的 `navigation.so_name` 是**宿主 so 的唯一来源**（ayanami_ros
  dlopen 它；`config_ros.json` 不再有 `navigation_so_name` 键）。换导航栈 =
  改这一个指针（`fsm_rose_navigation` ↔ `fsm_costmap_navigation`）。
- 模块文件指针统一放在路由 json 的 `navigation` 段
  （`config_planner/config_smoother/config_controller/config_goal_checker/
  config_progress_checker` 各为**单字符串**；`config_recovery` 为**有序
  字符串数组**，顺序即恢复轮转序）。FSM 架构下每族单模块、每族单 Execution。
- 模块 json 的 `<family>` 段（如 `planner: {name, so_name}`）是**模块 id 的
  单一来源**：name 即模块逻辑 id（无独立 `*_id` 配置键）。
- 层配置**内嵌**在路由 json 的 `costmap_global/costmap_local` 段——层已
  静态组合，`layers` 列表与逐层 `so_name` 键不再是配置的一部分。
- 日志配置（logging 段）位于根 `config.json`，没有独立的 config_logging 文件。

## logging 段（根 config.json）

日志按框架层拆分为命名 logger，每行输出携带 `[名字]` 标签
（`[%Y-%m-%d %H:%M:%S.%e] [名字] [级别] 消息`）：

| logger 名 | 负责层 |
|---|---|
| `ros` | ROS 适配层（ayanami_ros 边界、扩展模块） |
| `planner` / `smoother` / `controller` / `recovery` | 导航四组件（各自 execution 与插件模块；goal/progress checker 归 controller） |
| `BT` | 编排层——名字沿用 BT 时代（`utility/logging.hpp` 的 `kBtLogger`），现在承载 FsmDriver 的状态迁移日志 |
| `costmap` | 代价地图（CostmapNavigationServer、层、LayeredCostmap/CostmapWrapper/InflationLayer） |
| （默认 logger `ayanami`） | 框架基础设施（config/插件加载等未分层消息） |

| 键 | 默认值 | 说明 |
|---|---|---|
| `logging_level` | info | 全局主开关：所有 logger 的级别基准 |
| `console_level` | info | 控制台 sink 独立级别 |
| `console_output` / `file_output` | true | 控制台 / 文件输出开关 |
| `logging_dir` | logs | 日志目录（绝对或相对 cwd；costmap PGM dump 也落这里） |
| `rotate_logs` / `max_file_size_kb` / `max_files` | true / 8192 / 10 | 轮转参数（文件固定为 `<dir>/ayanami.log`） |

调试时可选择性写入 `logging.logger_levels`（如 `{"costmap": "debug"}`）
按层覆盖，出厂配置不含该键。

## navigation 段（路由 json）

| 键 | 出厂配置值 | 说明（代码默认值见文末） |
|---|---|---|
| `so_name` | libayanami_rose_navigation_fsm.so（rose 路由）/ libayanami_costmap_navigation_fsm.so（costmap 路由） | 宿主 so；ayanami_ros dlopen 的唯一来源 |
| `config_planner` 等 | — | 五族模块文件指针（见上） |
| `fsm_frequency` | 20.0 | FSM PLAN 线程 tick 频率（Hz） |
| `planner_frequency` | 5.0 | kFollow 节流重规划（Hz；≤0 = 仅路径失效时重规划） |
| `smoother_frequency` | 10.0 | kFollow 迭代重平滑节流（Hz；≤0 = 每条 raw path 只平滑一次） |
| `smoother_timeout` | 2.0 | 单次平滑建议时间预算（s；→ FsmDriverConfig::max_smooth_duration） |
| `controller_frequency` | 50.0 | controller execution 周期 |
| `max_nav_retries` | 6 | 每次尝试的恢复预算（goal 修订复位） |
| `oscillation_distance` / `oscillation_timeout` | 0.1 / 5.0 | 振荡门（位移 m / 时间窗 s） |
| `controller_patience` | 1.0 | nav2 failure_tolerance 语义（秒，<0=无限容忍，0=立即失败，>0=容忍窗口；覆盖"无有效命令"与"地图持续过期"MAP_STALE） |
| `recovery_patience` | 0.0 | 单次恢复的框架级超时（秒，≤0=关） |
| `max_vel_x / max_vel_y / max_vel_theta` | rose 2.5/2.5/3.0；costmap 2.0/2.0/2.0 | **速度上限单一来源**：execution 层发布前校验，超限清零 |
| `robot_frame` | base_link | 机器人根帧。**机器人帧的唯一权威**；`config_ros.robot_frame` 只是镜像，不一致时 ROS 层启动即抛异常 |
| `velocity_timeout` | 0.25 | 速度样本过期阈值（RSC） |

## esdfmap 段（仅 rose 路由）

| 键 | 出厂值 | 说明 |
|---|---|---|
| `max_distance` | 3.0 | 有符号场的正负钳位（m） |
| `update_rate` | 20.0 | EsdfWriter 重建频率上限（Hz；generation 门控下不变源直接跳过） |
| `stale_timeout` | 0.0 | 快照年龄门（s；0 = 有快照即 current；execution 层安全检查消费） |

## costmap 段

| 段/键 | 值 | 说明 |
|---|---|---|
| `costmap.dump_frequency` | 0.1 | PGM dump 频率（Hz，写者线程；0 关闭） |
| `costmap.global_costmap_frame` | map | 全局图帧（兼容旧键 `global_frame` / `map_frame`） |
| `costmap.local_costmap_frame` | odom | 局部图帧（nav2 语义；兼容旧键 `local_frame`；`config_ros.odom_frame` 应与其一致） |
| `costmap.resolution` | 0.05 | |
| `costmap.robot_radius` | 0.3 | 无 footprint 多边形时的圆近似来源 |
| `costmap_global.rolling_window` | false | 全局图固定 |
| `costmap_global.track_unknown` | false | |
| `costmap_global.static_layer.static_map` | config/assets/nav.pgm | StaticLayer 输入；元数据同名 `.json`（map.resolution/origin）；相对路径相对部署根解析 |
| `costmap_global.inflation_layer.{inflation_radius, inscribed_radius, cost_scaling_factor}` | 0.5 / 0.3 / 3.0 | InflationLayer |
| `costmap_local.size_x/size_y` | 120 / 120 | 6m×6m 滚动窗 |
| `costmap_local.origin` | [0,0,0] | 初始原点（rolling 后跟随机器人） |
| `costmap_local.rolling_window` / `track_unknown` | true / false | 局部图跟随机器人 |
| `costmap_local.transform_tolerance` | 0.1 | 观测 TF 有界等待（写者线程内轮询，不阻塞他人） |
| `costmap_local.update_frequency` | 50.0 | 滚动更新频率上限（写者循环驱动） |
| `costmap_local.observation_keep_time` | 0.5 | 观测邮箱保留窗 |
| `costmap_local.filter_static_marks` | true | **静态区标记过滤**：落在全局阻挡栅格内的局部点云命中不落图（静态墙不进局部动态层） |
| `costmap_local.inflation_layer.*` | 0.55 / 0.3 / 3.0 | InflationLayer |

`costmap_local.obstacle_layer` 子段（层参数内嵌，无 so_name）：
`combination`（overwrite/none/maximum）、`footprint_clearing`、
`decay_time`（占用衰减秒数，0=关闭）、`decay_sweep_interval`、
`observation_sources` + 逐源子块（`type` pointcloud/laserscan、
marking/clearing、高度窗、距离窗、`expected_update_rate` 传感器活性
——静默超率的源把地图标记为 not-current，下游表现为 MAP_STALE）。
一种传感器类型只允许一个源（重复即配置错误）。

## ayanami_ros 段（config_ros.json）

| 键 | 说明 |
|---|---|
| `pointcloud_topic / laserscan_topic / odom_topic / goal_topic` | 输入 topic（出厂：/nagisa/obstacle、空、/asuka/odometry、/move_base_simple/goal） |
| `map_frame / odom_frame / robot_frame` | map / odom / base_link |

TF 同步采样、`navigation_so_name`、`localization_clear_*` 等键均已随架构
演进而删：TF 改为按需查询桥；宿主 so 由路由 json 的 `navigation.so_name`
声明；全局图已是纯静态（static+inflation），不存在需要定位跳变清理的
全局动态层。注意 `odom_frame` 只是 ROS 侧帧名镜像；**局部地图帧的单一
来源是 `costmap.local_costmap_frame`**（启动时有两者一致性校验）。

## config_extensions.json

| 段 | 内容 |
|---|---|
| `extensions.loading_extension_modules` | 扩展 so 清单（出厂：libayanami_rviz_viewer.so、libayanami_rose_viewer.so、libayanami_cmd_chassis_publisher.so） |
| `rviz_viewer` | 12 个输出 topic 名（planner/smoother/controller/recovery × goal/feedback/status + navigation_status 等） |
| `rose_viewer` | 4 个 latched 地图 topic + 每工件重发频率；ESDF 场/占用 topic + `esdf_rate`/`esdf_stride` |
| `cmd_chassis_publisher` | /cmd_chassis topic + mode/加速度 ramp/功率限制（执行在底盘固件） |

## modules/*.json 模块文件模式

```json
{
  "planner": { "name": "costmap_astar_planner", "so_name": "libayanami_costmap_astar_planner.so" },
  "costmap_astar_planner": { "use_diagonal": true, "tolerance": 0.35, ... }
}
```

- 第一段是**家族描述**（family key：planner/smoother/controller/recovery/
  goal_checker/progress_checker）：`name` 为逻辑 id，`so_name` 为 dlopen
  目标。BT 时代的 `role` 字段已删——恢复顺序就是 `config_recovery` 数组
  的顺序，FsmDriver 轮转游标直接按序消费。
- 第二段起为**模块参数**，由模块在 `configureParameters(ModuleContext)` 中
  读取（加载成功后由 loader 注入），段名 = 模块名。

## 加载顺序

```
AyanamiROS 构造
 ├─ resolveConfigPath(param → env AYANAMI_CONFIG_PATH → ros::package)
 ├─ GlobalConfig::instance(config_path)          # config.json global 段
 ├─ configureLogging()                           # logging 段
 ├─ 读 route json（config_navigation 指针）→ navigation.so_name
 ├─ loadExtensionModules(config_extensions)      # 扩展先注册旁路槽
 ├─ dlopen 宿主 so → AbstractNavigation::initialize(config_path)
 │    ├─ bootstrap()（config root + logging）
 │    ├─ CostmapNavigationServer::configureCoreMaps(path)   # 双图 + 层静态组合
 │    ├─ [rose] EsdfMap + EsdfWriter（读路由 json esdfmap 段；mask 读者注入）
 │    └─ loadModules(path)                       # 6 族 dlopen + 虚 initialize*Module
 │         └─ createActions()                    # Execution 装配 + FsmDriver + path validator
 ├─ subscribeTopics()
 └─ start()：宿主 start（写者线程 → 模块 activate → fsm_->start()）
```

## 代码默认值 vs 出厂配置值

上表右列是**出厂配置**里的值，不等于代码里的默认值：代码默认值写在各个
`XxxParam` 结构体的字段初始化里（模块）、`FsmDriverConfig` 字段初始化
（FSM）或 `param(..., default)` 的字面量里（核心）。省略某个键时取的是
**代码默认值**，可能与出厂值不同。要确认实际生效的值，看启动日志里的参数
读取记录，或直接查结构体字段初值。

参数键打错不会被静默吞掉：加载模块时 loader 比对该段「声明了但没人读」的
键并告警（`global.strict_params = true` 时改为启动失败）。先审计、后严格，
避免把既有配置里的冗余键一次性变成启动错误。

## 模块身份约定

一个模块只有一个名字：**模块 json 的 `name` 字段 = 参数段名 = 被加载的
`.so`**（`libayanami_<name>.so`）。loader 在加载时校验前两者一致且参数段
存在，不一致即拒绝加载。`.so` 基名不参与校验——同一个 `.so` 挂多个逻辑名
（各自一套参数）是合法的别名用法。

## inflation.inscribed_radius 语义（2026-09-29）

`costmap_global/costmap_local` 的 `inflation_layer.inscribed_radius` **必须
等于 robot_radius**（0.3）：253（INSCRIBED）致死带 = 机器人真实足印区。
此前配 0.12 时规划器合法产出"中心线距障碍 0.15~0.3m"的路径（车体壳层蹭墙），
近距点云随即把墙沿标记为 lethal，路径中心线落入 253 带内 → isPathValid
失败 → 恢复级联 → 停车。配套：`costmap_astar_planner.tolerance`（0.35）须
大于该值，使落在 253 带内的目标点仍能在带外找到近似可达点。
