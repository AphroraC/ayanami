# 配置系统

## 文件结构与单一来源纪律

配置为多层 JSON，**每个键只有一个权威来源**：

```
config/
├── config.json               # 根索引：global 段（文件指针）+ logging 段
├── config_ros.json           # 仅 ayanami_ros 使用：topic / 帧 / TF 同步边 / 阈值
├── costmap_navigation.json   # 导航主配置：navigation 段 + costmap 三段
├── assets/                   # nav.pgm + nav.json（静态地图元数据）、rmuc26.*
└── modules/                  # 每插件一个文件：<family> 段 + <plugin> 参数段
    ├── costmap_smac_planner.json
    ├── costmap_savitzky_smoother.json
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
- 插件文件指针统一放在 `costmap_navigation.json::navigation` 段
  （`config_planner/config_smoother/config_controller/config_goal_checker/
  config_progress_checker` 各为**单字符串**；`config_recovery` 为**有序
  字符串数组**，顺序即恢复声明序）。
- 模块 json 的 `<family>` 段（如 `planner: {name, so_name}`）是**模块 id 的
  单一来源**：name 即插件逻辑 id（无独立 `*_id` 配置键）。
- 日志配置（logging 段）位于根 `config.json`，没有独立的 config_logging 文件。

## navigation 段（costmap_navigation.json）

| 键 | 默认 | 说明 |
|---|---|---|
| `behavior_tree_extension` | libayanami_navigation_behavior_tree.so | BT 扩展 .so（单串或列表），负责注册节点与默认树 |
| `behavior_tree_id` | NavigateWithReplanningAndSmoothing | 树 id；空则用扩展 defaultTreeId |
| `behavior_tree_xml` | 空 | 可选外部树 XML（相对 config_path）；生产为空 = 用扩展内嵌默认树 |
| `config_planner` 等 | — | 见上 |
| `planner_frequency` | 1.0 | >0：RateController 限频重规划（nav2 语义）；<=0：单发缓存 |
| `smoother_frequency` | -1.0 | -1：每条新 raw_path 只平滑一次（reset_token={path_id} 仍会触发） |
| `controller_frequency` | 20.0 | controller execution 周期 |
| `tick_frequency` | 100.0 | pilot BT tick 频率 |
| `max_vel_x / max_vel_y / max_vel_theta` | 0.5 / 0.0 / 1.5 | **速度上限单一来源**：execution 层发布前校验，超限清零 |
| `robot_frame` | base_footprint | 机器人根帧（与 config_ros.robot_frame 一致性校验） |
| `velocity_timeout` | 0.25 | 速度样本过期阈值（RSC） |

## costmap 段

| 段/键 | 值 | 说明 |
|---|---|---|
| `costmap.global_frame` | map | 全局图帧 |
| `costmap.local_frame` | odom | 局部图帧（nav2 语义；`config_ros.odom_frame` 应与其一致） |
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

组合语义：全局障碍层 `MAXIMUM` + `setClearToFree(false)`（静态层先恢复，
动态标记只做 cost>old 叠加）；局部障碍层 `OVERWRITE`。

## ayanami_ros 段（config_ros.json）

| 键 | 说明 |
|---|---|
| `laserscan_topic / pointcloud_topic / odom_topic / goal_topic` | 输入 topic（/scan、/velodyne_points、/odom、/move_base_simple/goal） |
| `map_frame / odom_frame / robot_frame` | map / odom / base_footprint |
| `tf_sync_rate` | 50.0 Hz 同步循环 |
| `tf_sync_edges` | `[[target, source], ...]`：map←odom、odom←base_footprint、base_footprint←base_link、base_link←support、support←laser |
| `tf_static_edges` | base_footprint←base_link 等静态边（fast_tf is_static 分类） |
| `localization_clear_yaw_threshold / localization_clear_pos_threshold` | 全局动态层重建触发：累计旋转 1.0 rad / 累计平移 2.0 m |
| `navigation_so_name` | libayanami_costmap_navigation.so |
| `extension_modules` | [libayanami_rviz_viewer.so] |
| 其余 `*_topic` | 旁路可视化输出（由 rviz_viewer 发布） |

注意：`odom_frame` 只是 ROS TF 同步边配置；**局部地图的帧单一来源是
`costmap.local_frame`**（启动时有两者一致性校验，不一致打 error）。

## modules/*.json 插件文件模式

```json
{
  "planner": { "name": "costmap_smac", "so_name": "libayanami_costmap_smac_planner.so" },
  "costmap_smac_planner": { "use_diagonal": true, "tolerance": 0.0, ... }
}
```

- 第一段是**家族描述**（family key：planner/smoother/controller/recovery/
  goal_checker/progress_checker）：`name` 为逻辑 id（黑板引用/分派用），
  `so_name` 为 dlopen 目标；recovery 额外有 `role` 字段
  （clear_costmap/spin/wait/backup）供 pilot 把角色映射到黑板 id。
- 第二段起为**插件参数**，由插件在 `configure(PluginContext)` 中读取，
  键即插件名段（如 `costmap_dwb_controller.RotateToGoal.scale` 点分键）。

## 加载顺序

```
AyanamiROS 构造
 ├─ resolveConfigPath(param → env AYANAMI_CONFIG_PATH → ros::package)
 ├─ GlobalConfig::instance(config_path)          # config.json global 段
 ├─ configureLogging()                           # logging 段
 ├─ loadExtensionModules(config_ros)             # rviz_viewer 等先注册旁路
 ├─ loadNavigationPlugin(config_ros)             # dlopen 宿主插件
 │    └─ CostmapNavigation::initialize
 │         ├─ configureCoreMaps(path, MapFamilyFactory)   # 地图族工厂
 │         ├─ loadPlugins(path)                  # 6 族 dlopen + 虚 initialize*Plugin
 │         ├─ bt_pilot.configure(path)           # 频率/恢复角色映射
 │         └─ bt_pilot.loadFromFile("", tree_id) # 建常驻树（扩展注册树）
 └─ subscribeTopics()
```

## Patience 与频率（navigation 段）

- `planner_frequency`：-1 = 每 goal 只规划一次（goal 更新经
  goal_revision reset_token 触发重规划）。
- `smoother_frequency`：10.0（生产默认；每条新路径落地即重新平滑）。
- `controller_frequency`：20.0。
- `controller_patience`：对标 nav2 failure_tolerance（秒，0=关）。无有效
  速度命令持续超过该时长才终态上报；重试决策仍归 BT。
- `recovery_patience`：单次恢复的框架级超时（秒，0=关），防止自限缺失的
  恢复插件无限 RUNNING。
