# 配置系统

## 文件结构与单一来源纪律

配置为多层 JSON，**每个键只有一个权威来源**：

```
config/
├── config.json               # 根索引：global 段（文件指针）+ logging 段
├── config_ros.json           # 仅 ayanami_ros 使用：topic / 帧 / TF 同步边 / 阈值
├── costmap_navigation.json   # 导航主配置：navigation 段 + costmap 三段
├── assets/                   # nav.pgm + nav.json（静态地图元数据）、rmuc26.*
└── modules/                  # 每模块一个文件：<family> 段 + <name> 参数段
    ├── costmap_astar_planner.json
    ├── costmap_minsnap_smoother.json
    ├── costmap_dwb_controller.json
    ├── costmap_pid_controller.json
    ├── costmap_clear_costmap_recovery.json   # recovery.role = clear_costmap
    ├── costmap_spin_recovery.json            # recovery.role = spin
    ├── costmap_wait_recovery.json            # recovery.role = wait
    ├── costmap_back_up_recovery.json         # recovery.role = backup
    ├── simple_goal_checker.json
    └── simple_progress_checker.json
```

- `config.json::global` 只提供文件指针：`config_path`（目录）、
  `config_navigation`、`config_ros`。不提供多实例语义（单机器人单导航）。
- 模块文件指针统一放在 `costmap_navigation.json::navigation` 段
  （`config_planner/config_smoother/config_controller/config_goal_checker/
  config_progress_checker` 各为**单字符串**；`config_recovery` 为**有序
  字符串数组**，顺序即恢复声明序）。
- 模块 json 的 `<family>` 段（如 `planner: {name, so_name}`）是**模块 id 的
  单一来源**：name 即模块逻辑 id（无独立 `*_id` 配置键）。
- 日志配置（logging 段）位于根 `config.json`，没有独立的 config_logging 文件。

## logging 段（根 config.json）

日志按框架层拆分为命名 logger，每行输出携带 `[名字]` 标签
（`[%Y-%m-%d %H:%M:%S.%e] [名字] [级别] 消息`）：

| logger 名 | 负责层 |
|---|---|
| `ros` | ROS 适配层（ayanami_ros 边界、rviz_viewer / cmd_chassis_publisher 扩展） |
| `planner` / `smoother` / `controller` / `recovery` | 导航四组件（各自 execution/action 层与插件模块；goal/progress checker 归 controller） |
| `BT` | 行为树层（pilot、BT 节点、navigation_behavior_tree 扩展） |
| `costmap` | 代价地图（costmap 导航服务器、层插件、LayeredCostmap/CostmapWrapper/InflationLayer） |
| （默认 logger `ayanami`） | 框架基础设施（config/插件加载等未分层消息） |

日志等级统一由 `logging_level` 一个开关控制（`console_level` 单独控制控制台）；调试时可选择性写入 `logging.logger_levels`（如 `{"costmap": "debug"}`）按层覆盖，出厂配置不含该键。

| 键 | 默认值 | 说明 |
|---|---|---|
| `logging_level` | info | 全局主开关：所有 logger 的级别基准 |
| `console_level` | info | 控制台 sink 独立级别 |
| `console_output` / `file_output` | true | 控制台 / 文件输出开关 |
| `logging_dir` | logs | 日志目录（绝对或相对 cwd；costmap PGM dump 也落这里） |
| `rotate_logs` / `max_file_size_kb` / `max_files` | true / 8192 / 10 | 轮转参数（文件固定为 `<dir>/ayanami.log`） |

## navigation 段（costmap_navigation.json）

| 键 | 出厂配置值 | 说明（代码默认值见文末） |
|---|---|---|
| `behavior_tree_extension` | libayanami_navigation_behavior_tree.so | BT 扩展 .so（单串或列表），负责注册节点与默认树 |
| `behavior_tree_id` | NavigateWithReplanningAndSmoothing | 树 id；空则用扩展 defaultTreeId |
| `behavior_tree_xml` | BT/navigation.xml | 树 XML（相对 config_path）。**必需**：没有内嵌默认树，配置为空或文件不存在即启动报错 |
| `config_planner` 等 | — | 见上 |
| `planner_frequency` | 1.0 | >0：RateController 限频重规划（nav2 语义）；<=0：单发缓存 |
| `smoother_frequency` | 10.0 | >0：RateController 限频重平滑，reset_token={raw_path_revision}（新的原始路径落地即重平滑）；<=0：单发缓存 |
| `controller_frequency` | 20.0 | controller execution 周期 |
| `tick_frequency` | 100.0 | pilot BT tick 频率 |
| `max_vel_x / max_vel_y / max_vel_theta` | 0.5 / 0.0 / 1.5 | **速度上限单一来源**：execution 层发布前校验，超限清零 |
| `robot_frame` | base_link | 机器人根帧。**机器人帧的唯一权威**；`config_ros.robot_frame` 只是镜像，不一致时 ROS 层启动即抛异常 |
| `velocity_timeout` | 0.25 | 速度样本过期阈值（RSC） |

## costmap 段

| 段/键 | 值 | 说明 |
|---|---|---|
| `costmap.global_costmap_frame` | map | 全局图帧（兼容旧键 `global_frame` / `map_frame`） |
| `costmap.local_costmap_frame` | odom | 局部图帧（nav2 语义；兼容旧键 `local_frame`；`config_ros.odom_frame` 应与其一致） |
| `costmap.resolution` | 0.05 | |
| `costmap.robot_radius` | 0.12 | 无 footprint 多边形时的圆近似来源；有 `costmap.footprint` 则优先 |
| `costmap_global.rolling_window` | false | 全局图固定 |
| `costmap_global.static_map` | config/assets/nav.pgm | StaticLayer 输入；元数据同名 `.json`（map.resolution/origin） |
| `costmap_global.inflation_radius / inscribed_radius / cost_scaling_factor` | 0.5 / 0.12 / 10 | InflationLayer |
| `costmap_local.rolling_window` | true | 局部图跟随机器人（updateOrigin 居中） |
| `costmap_local.size_x/size_y` | 60 / 60 | 3m×3m 窗口 |
| `costmap_local.inflation_radius` 等 | 0.2 / 0.12 / 3 | |
| `costmap_local.obstacle_range / raytrace_range` | 3.0 / 3.5 | 落图标记/射线清除半径 |
| `costmap_local.update_frequency` | 10.0 | 滚动窗口解耦更新频率（spinOnce 驱动，与传感器流无关） |
| `costmap_global.update_frequency` | 5.0 | 同上（仅 global 启用 rolling_window 时生效） |

`costmap_{global,local}.layers` 为**必需**键（层模块 json 列表，顺序即
组合顺序）；障碍层参数：`combination`（overwrite/maximum）、
`clear_margin`（前窗清除外扩）、`footprint_clearing`、`decay_time`
（占用衰减秒数，0=关闭；全局层建议启用）、`observation_sources` +
逐源子块（过滤参数归层所有）。

## ayanami_ros 段（config_ros.json）

| 键 | 说明 |
|---|---|
| `laserscan_topic / pointcloud_topic / odom_topic / goal_topic` | 输入 topic（/scan、/velodyne_points、/odom、/move_base_simple/goal） |
| `map_frame / odom_frame / robot_frame` | map / odom / base_link |
| ~~`tf_sync_rate` / `tf_sync_edges` / `tf_static_edges` / `tf_cache_duration`~~ | 已移除：TF 改为按需查询桥（核心经注入源直达 tf2 buffer），采样配置不复存在 |
| `localization_clear_yaw_threshold / localization_clear_pos_threshold` | 全局动态层重建触发：累计旋转 1.0 rad / 累计平移 2.0 m |
| `navigation_so_name` | libayanami_costmap_navigation.so |
| `extension_modules` | [libayanami_rviz_viewer.so] |
| 其余 `*_topic` | 旁路可视化输出（由 rviz_viewer 发布） |

注意：`odom_frame` 只是 ROS TF 同步边配置；**局部地图的帧单一来源是
`costmap.local_frame`**（启动时有两者一致性校验，不一致打 error）。

## modules/*.json 模块文件模式

```json
{
  "planner": { "name": "costmap_astar", "so_name": "libayanami_costmap_astar_planner.so" },
  "costmap_astar_planner": { "use_diagonal": true, "tolerance": 0.0, ... }
}
```

- 第一段是**家族描述**（family key：planner/smoother/controller/recovery/
  goal_checker/progress_checker）：`name` 为逻辑 id（黑板引用/分派用），
  `so_name` 为 dlopen 目标；recovery 额外有 `role` 字段
  （clear_costmap/spin/wait/backup）供 pilot 把角色映射到黑板 id。
- 第二段起为**模块参数**，由模块在 `configureParameters(ModuleContext)` 中读取
  （加载成功后由 loader 注入），段名 = 模块名。DWB critic 的子参数是
  `<critic>.scale` 形式的**扁平键**，不是嵌套。

## 加载顺序

```
AyanamiROS 构造
 ├─ resolveConfigPath(param → env AYANAMI_CONFIG_PATH → ros::package)
 ├─ GlobalConfig::instance(config_path)          # config.json global 段
 ├─ configureLogging()                           # logging 段
 ├─ loadExtensionModules(config_ros)             # rviz_viewer 等先注册旁路
 ├─ loadNavigationModule(config_ros)             # dlopen 宿主模块
 │    └─ CostmapNavigation::initialize
 │         ├─ configureCoreMaps(path, MapFamilyFactory)   # 地图族工厂
 │         ├─ loadModules(path)                  # 6 族 dlopen + 虚 initialize*Module
 │         ├─ bt_pilot.configure(path)           # 频率/恢复角色映射
 │         └─ bt_pilot.loadFromFile("", tree_id) # 建工厂（扩展注册树；实例按尝试新建）
 └─ subscribeTopics()
```

## Patience 与频率（navigation 段）

- `planner_frequency`：-1 = 每 goal 只规划一次（goal 更新经
  goal_revision reset_token 触发重规划）。
- `smoother_frequency`：10.0（出厂值；每条新路径落地即重新平滑）。
- `controller_frequency`：20.0。
- `controller_patience`：对标 nav2 failure_tolerance 语义（秒，<0=无限容忍，0=立即失败，>0=容忍窗口）。覆盖"无有效命令"与"地图持续过期"（MAP_STALE 终态，交 BT 恢复分支）。无有效
  速度命令持续超过该时长才终态上报；重试决策仍归 BT。
- `recovery_patience`：单次恢复的框架级超时（秒，0=关），防止自限缺失的
  恢复模块无限 RUNNING。

## 代码默认值 vs 出厂配置值

上表右列是**出厂配置**里的值，不等于代码里的默认值：代码默认值写在各个
`XxxParam` 结构体的字段初始化里（模块）或 `param(..., default)` 的字面量里
（核心）。省略某个键时取的是**代码默认值**，可能与出厂值不同。要确认实际生效
的值，看启动日志里的参数读取记录，或直接查结构体字段初值。

参数键打错不会被静默吞掉：加载模块时 loader 比对该段「声明了但没人读」的键并
告警（`global.strict_params = true` 时改为启动失败）。先审计、后严格，避免把既有
配置里的冗余键一次性变成启动错误。

## 模块身份约定

一个模块只有一个名字：**模块 json 的 `name` 字段 = 参数段名 = 被加载的 `.so`**
（`libayanami_<name>.so`）。loader 在加载时校验前两者一致且参数段存在，不一致即
拒绝加载。`.so` 基名不参与校验——同一个 `.so` 挂多个逻辑名（各自一套参数）是合法
的别名用法，recovery 的 role 列表正是这么用的。


## inflation.inscribed_radius 语义（2026-09-29）

`costmap_global/costmap_local` 的 `inflation_layer.inscribed_radius` **必须等于
robot_radius**（0.3）：253（INSCRIBED）致死带 = 机器人真实足印区。此前配 0.12 时
规划器合法产出"中心线距障碍 0.15~0.3m"的路径（车体壳层蹭墙），近距点云随即把墙沿
标记为 lethal，路径中心线落入 253 带内 → IsPathValid 失败 → 恢复级联 → 停车。
配套：`costmap_astar_planner.tolerance`（0.35）须大于该值，使落在 253 带内的目标点
仍能在带外找到近似可达点。
