# ROS 集成层（ayanami_ros）

ayanami_ros 是整个系统**唯一接触 ROS 的地方**。职责：把 ROS 数据转成
ayanami 消息送入核心、以固定频率驱动核心、承载 ROS 侧扩展。核心侧红线：
零 ROS 头文件。

## AyanamiROS（ayanami_ros.hpp / ayanami_ros.cpp）

对外接口（供宿主/测试调用，与 ROS 回调共用）：

| 方法 | 语义 |
|---|---|
| `installTfSource()`（构造时） | 把 tf2 backed 查询源注入核心 `RobotStateController`（按需查询桥） |
| `setOdometry(TwistStamped)` | 直写核心 `RobotStateController::setOdometry`（控制器读最新速度） |
| `submitSensor(PointCloud2 / LaserScan)` | 直递宿主 `setPointcloud/setLaserScan`（传感器时刻变换由核心按需经 tf2 源查询） |
| `submitGoal(PoseStamped)` | goal 邮箱覆写（latest-wins，不排队） |
| `drainPendingGoal() / hasPendingGoal()` | loop 线程每周期投递最新一条 |
| `start() / stop()` | loop 线程生命周期（50Hz） |

loop 线程每周期：`checkTfHealth()`（1Hz）→
`navigation_module->spinOnce()`（costmap 可视化 1Hz）→ `drainPendingGoal()`
→ `ros::spinOnce()`（触发 scan/odom/goal 回调）→ WallRate sleep。

扩展加载顺序（M2-P1）：扩展（rviz_viewer，构造期注册旁路订阅）**先于**
navigation_module->start()（首波可视化 emit 不丢）。

### 定位修正累计漂移 → 清全局动态层

loop 的 TF 同步中对 `odom←base_footprint` 边累计机器人旋转/平移（自上次
清空起），超过阈值（`localization_clear_yaw_threshold=1.0 rad` /
`localization_clear_pos_threshold=2.0 m`）调用宿主
`onLocalizationJump()` 清空全局动态层。

背景（实测）：原地旋转时轮式里程计打滑欠报转角 ~6%，AMCL 修正渐进追赶，
期间 map 系 scan 持续偏转、全局动态标记整体错位。注意信号必须是**机器人
运动量**而非修正增量——修正渐进且可能长期不收敛，增量恒为小值。

### TF 桥（按需查询，installTfSource）

- 构造时把 tf2 backed 查询源注入核心 `RobotStateController`
  （`setTransformSource`）：核心线程的 `lookupTransform / canTransform /
  lookupPose` **按需直达 tf2 buffer**——插值、静态边、stamp 时间查询全部
  由 tf2 负责，无周期采样、无核心侧变换缓存。
- 超时语义：TF2Listener 内部用 **steady_clock 轮询 canTransform**（5ms
  粒度），绝不用 tf2 自带的带时长等待——后者以 ros::Time（sim time）计时，
  /clock 停摆时会无限阻塞（实测事故）。缺边由 `checkTfHealth`（1Hz，附
  完整帧树）低频诊断兜底。
- 障碍落图的时间约定：传感器变换按 scan 戳查询（见 map-and-costmap.md），
  机器人位姿/滚动居中用 latest——两者不要混用。

## goal 链路与 AMCL

```
/move_base_simple/goal ──► submitGoal（邮箱）──► loop drain ──► setGoal
                                                                   └─► BT 邮箱
AMCL (map←odom) ──► tf2 buffer ◄── installTfSource ◄── 全部核心 TF 查询唯一入口
```

AMCL 集成要点（`nav_demo/launch/amcl_diff.launch`，参数均已源码验证）：

| 参数 | 值 | 原因 |
|---|---|---|
| `odom_model_type` | diff | |
| `odom_alpha1` | **1.0** | 源码 bug 补偿：DIFF 模型把方差当 sigma 传给 pf_ran_gaussian（缺 sqrt），0.2 时纯旋转粒子角噪声过小，滤波器跟随打滑里程计 |
| `laser_max_beams` | **60** | 344° FOV 航向分辨率 5.7°（30 束仅 11.5°） |
| `recovery_alpha_slow / fast` | **0.001 / 0.1** | 0/0 会永久禁用随机粒子注入（pf.c: w_diff=1-w_fast/w_slow 恒 0），滤波器塌缩后无法恢复 |
| `min_particles / max_particles` | **1000 / 10000** | 估计方差 ≈ sigma/√N_eff：粒子越多估计越平滑（少晃动）、假设覆盖越密（快收敛）。**注意 AMCL.cfg 的重配置范围 [0,1000] 会把 min_particles 钳到 1000**（roslaunch 设 1500 实际生效 1000） |
| `resample_interval` | **2** | 隔次重采样：降低每次更新注入的重采样噪声（估计抖动来源之一）。**launch 同名 param 后值覆盖前值（roslaunch 语义），曾因此中招——改参数后必须 `rosparam get` 核实 live 值** |
| 调参实测 | — | 粒子 1000-10000 + 100 束 + 隔次重采样（1.2 rad/s 旋转 / 0.3 m/s 直行）：位置误差 max 0.052→**0.015 m**（3×），旋转中 yaw 误差 p2p 0.042→0.037 rad，运动后收敛 ~0.85 s（受 scan 速率 5.5 Hz 门控） |
| `initial_cov_aa` | **0.15** | 打滑旋转后真值可偏离初值 0.5+ rad，紧先验锁死粒子群 |
| `initial_pose_x/y/a` | ≈(0,0,0) | 必须用 `initial_pose_a`（不是 _yaw，否则内部 NaN，不发布 map→odom） |
| `update_min_d/a` | 0.05 / **0.05** | 触发滤波更新的最小运动。**AMCL 是运动门控滤波器**（amcl_node.cpp: `update = |Δx|>d || |Δy|>d || |Δyaw|>a`）——静止时跳过量测更新，估计**冻结在停下那一刻的值**，scan 不会"静止收敛到贴墙"；恢复运动（>阈值）即继续修正（实测冻结 0.05 rad 的航向误差在前进 0.5m 后修正到 0.007 rad）。快速旋转（1.2 rad/s）中更新频率受 scan 速率（5.5Hz）限制，降低阈值主要改善慢速旋转时的更新密度 |
| `transform_tolerance` | 0.5 | |

map_server 加载 `nav_demo/map/ayanami.yaml` → 指向
`ayanami/config/assets/nav.pgm`（GIMP 精确裁剪的 SLAM 图，与 ayanami 静态层
同一张图、同一原点，逐像素一致）。

## costmap/field 输出扩展（costmap_viewer / rosemap_viewer）

- rviz_viewer 只发布路径/状态/`cmd_vel`；**全部地图发布由独立扩展承担**。
- `costmap_viewer`：订阅三个每层工件槽
  `on_global_costmap_static` / `on_global_costmap_inflation` /
  `on_local_costmap_inflation`（server 唯一写者在内容变更时无条件发射，
  generation 门控；静态 global 可能只发射一次），**长期持有缓存**并在独立
  线程按 `config_extensions.json::costmap_viewer` 的频率重发（global 0.1Hz
  边沿触发 + 周期重发，local 5Hz）；latched 话题
  `/ayanami/global_costmap/static`、`/ayanami/global_costmap/inflation`、
  `/ayanami/local_costmap/obstacle`、`/ayanami/local_costmap/inflation`。
- `rosemap_viewer`：订阅 `on_rosemap_field`（rosemap server 写者按
  sequence 门控无条件发射），同样的缓存 + 独立线程 + 可配频率
  （`config_extensions.json::rosemap_viewer`，默认 5Hz）；latched 话题
  `/ayanami/rosemap_field`。
- 转换统一走 `ros_msg_converter.hpp`（lethal→100、NO_INFORMATION→-1、
  线性映射其余；帧由核心单一来源填充）。
- 两个扩展都导出 `extern "C"` 发布计数器（costmapViewer*/rosemapViewer*）
  供测试 dlsym 验证加载顺序与发布行为。

## rviz_viewer（可视化扩展）

- 构造期注册旁路 `CallbackSubscription`（RAII）：12 个 action 流槽
  （goal/feedback/result × 4 族）+ `on_velocity_command`。
- 发布 ROS topic：`/ayanami/planner/path`、`/ayanami/smoother/path`、
  `/ayanami/controller/path|status`、`/ayanami/recovery/goal|status`、
  `/cmd_vel` 等；不再发布任何地图。

## 消息转换（ros_msg_converter.hpp）

纯函数转换：fromRosPose / fromRosTwist / fromRosPointCloud2 / fromRosLaserScan
/ toRosPath / toRosOccupancyGrid；`ros::Time(0) ↔ Timestamp::latest()`。
scan/pointcloud 的 frame_id 原样保留，落图时由核心经注入的 tf2 源解析。

## 输出边界（rviz_viewer / cmd_chassis_publisher）

速度输出是**双通道互补分工**，不是重复输出：

- `rviz_viewer`：可视化扩展 + **速度本体通道**——订阅全局回调槽发布路径/
  costmap/goal/feedback/status/**navigation_status**，并把
  `on_velocity_command` 的速度本体转发为 `geometry_msgs/Twist` 发布到
  `/cmd_vel`（rm_chassis_controllers ChassisBase 的输入）。话题参数在
  `config_extensions.json::rviz_viewer`。
- `cmd_chassis_publisher`：**命令参数通道**——每收到一条核心速度指令发布一条
  `rm_msgs/ChassisCmd`（mode / ramp 加速度 / power_limit）。加速度 ramp 与
  功率限制的**执行在底盘固件**（ChassisBase），不在本层、也不在导航核心；
  导航核心只通过控制器周期契约（`setCyclePeriod`）保证模块内部积分使用的
  dt 与真实控制周期一致。参数在 `config_extensions.json::cmd_chassis_publisher`。
- ayanami_ros 本体只做 ROS 输入（订阅 + TF 同步 + goal 邮箱）。
- **停机整形归属**：核心的终态/取消只发"恰一次零脉冲"（速度阶跃），减速曲线
  由下游执行——RM 部署链路中 ChassisCmd 携带 ramp 加速度参数、由底盘固件限幅；
  `/cmd_vel` 链路中由 rm_chassis_controllers 自身限幅。**非 rm_chassis 的第三方
  消费者（含无头部署直连 on_velocity_command）需自行做速度整形**，核心不内嵌
  减速 ramp（保持精简，见 2026-09-28 决策）。
