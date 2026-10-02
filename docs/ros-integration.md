# ROS 集成层（ayanami_ros）

ayanami_ros 是整个系统**唯一接触 ROS 的地方**。职责：把 ROS 数据转成
ayanami 消息送入核心、以固定频率驱动 loop、承载 ROS 侧扩展。核心侧红线：
零 ROS 头文件。

## 宿主 so 的路由选择（BT 时代 config_ros.navigation_so_name 的继任者）

```
config.json::global.config_navigation  →  routes/fsm_rose_navigation.json（默认）
                                         或 routes/fsm_costmap_navigation.json
route json::navigation.so_name         →  dlopen 目标（宿主模块）
```

AyanamiROS 构造期读路由 json 的 `navigation.so_name` 并
`dlopen`（`libayanami_rose_navigation_fsm.so` / 
`libayanami_costmap_navigation_fsm.so`），经 `AbstractNavigation` 接口使用：
`initialize / start / stop / spinOnce / setGoal / setPointcloud /
setLaserScan`。换导航栈 = 换 `config.json::global.config_navigation` 一个
指针；`config_ros.json` 不再声明宿主 so，也没有第二份导航配置。

## AyanamiROS（ayanami_ros.hpp / ayanami_ros.cpp）

对外接口（供宿主/测试调用，与 ROS 回调共用）：

| 方法 | 语义 |
|---|---|
| `installTfSource()`（构造时） | 把 tf2 backed 查询源注入核心 `RobotStateController`（按需查询桥） |
| `setOdometry(TwistStamped)` | 直写核心 `RobotStateController::setOdometry`（控制器读最新速度） |
| `submitSensor(PointCloud2 / LaserScan)` | 直递宿主 `setPointcloud/setLaserScan`（传感器时刻变换由写者线程按需经 tf2 源查询） |
| `submitGoal(PoseStamped)` | goal 邮箱覆写（latest-wins，不排队） |
| `drainPendingGoal() / hasPendingGoal()` | loop 线程每周期投递最新一条 |
| `start() / stop()` | loop 线程生命周期（50Hz） |

loop 线程每周期：`checkTfHealth()`（1Hz）→
`navigation_module->spinOnce()`（FSM 宿主为空实现，保留接缝）→
`drainPendingGoal()` → `ros::spinOnce()`（触发 scan/odom/goal 回调）→
WallRate sleep。滚动窗口更新已由 costmap 写者线程自驱动，不再经 loop。

扩展加载顺序：扩展（构造期注册旁路订阅）**先于** navigation_module->start()
（首波可视化 emit 不丢）。

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
                                                                  └─► FsmDriver 邮箱
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
| `update_min_d/a` | 0.05 / **0.05** | 触发滤波更新的最小运动。**AMCL 是运动门控滤波器**——静止时跳过量测更新，估计**冻结在停下那一刻的值**；恢复运动（>阈值）即继续修正。降低阈值主要改善慢速旋转时的更新密度 |
| `transform_tolerance` | 0.5 | |

map_server 加载 `nav_demo/map/ayanami.yaml` → 指向
`ayanami/config/assets/nav.pgm`（GIMP 精确裁剪的 SLAM 图，与 ayanami 静态层
同一张图、同一原点，逐像素一致）。

## 地图输出扩展（rose_viewer）

- `rose_viewer`（libayanami_rose_viewer.so，costmap_viewer 的继任者，以
  RoseNavigationFsm 命名）：一切数据经核心全局回调槽进入，发布为 ROS
  消息供 rviz 可视化——
- 四个 costmap 每层工件槽
  （`on_global_costmap_{static,inflation}` / `on_local_costmap_{obstacle,
  inflation}`）→ `nav_msgs/OccupancyGrid` latched 话题；长期缓存 + 独立
  发布线程按 `config_extensions.json::rose_viewer` 的各自频率重发
  （global 0.1Hz 边沿触发、local 5Hz 纯频率驱动；一次性发射的静态全局
  图由 viewer 长期持有重发）；
- ESDF 场槽 `on_esdfmap_field`（宿主经 EsdfWriter 快照监听器发射，
  sequence 门控）→ `sensor_msgs/PointCloud2` `/ayanami/esdf/field`
  （x/y 为格心、z=0、intensity=**有符号距离**（米），按 `esdf_stride`
  降采样，rose_navigation esdf_out 风格；rviz 用 Intensity 色彩渲染
  距离场）+ `nav_msgs/OccupancyGrid` `/ayanami/esdf/occupancy`
  （occupied 二值面 100/0），共用 `esdf_rate`（默认 2Hz）。

## rviz_viewer（可视化扩展）

- 构造期注册旁路 `CallbackSubscription`（RAII）：12 个 action 流槽
  （goal/feedback/result × 4 族）+ `on_velocity_command`。
- 发布 ROS topic：`/ayanami/planner/path`、`/ayanami/smoother/path`、
  `/ayanami/controller/status`、`/ayanami/recovery/goal|status`、
  `/cmd_vel` 等；不发布任何地图。

## 消息转换（ros_msg_converter.hpp）

纯函数转换：fromRosPose / fromRosTwist / fromRosPointCloud2 / fromRosLaserScan
/ toRosPath / toRosOccupancyGrid；`ros::Time(0) ↔ Timestamp::latest()`。
scan/pointcloud 的 frame_id 原样保留，落图时由核心经注入的 tf2 源解析。

## 输出边界（rviz_viewer / cmd_chassis_publisher）

速度输出是**双通道互补分工**，不是重复输出：

- `rviz_viewer`：可视化扩展 + **速度本体通道**——订阅全局回调槽发布路径/
  goal/feedback/status/**navigation_status**，并把 `on_velocity_command`
  的速度本体转发为 `geometry_msgs/Twist` 发布到 `/cmd_vel`
  （rm_chassis_controllers ChassisBase 的输入）。话题参数在
  `config_extensions.json::rviz_viewer`。
- `cmd_chassis_publisher`：**命令参数通道**——每收到一条核心速度指令发布一条
  `rm_msgs/ChassisCmd`（mode / ramp 加速度 / power_limit）。加速度 ramp 与
  功率限制的**执行在底盘固件**（ChassisBase），不在本层、也不在导航核心。
  参数在 `config_extensions.json::cmd_chassis_publisher`。
- ayanami_ros 本体只做 ROS 输入（订阅 + TF 桥 + goal 邮箱）。
- **停机整形归属**：核心的终态/取消只发"恰一次零脉冲"（速度阶跃），减速曲线
  由下游执行——RM 部署链路中 ChassisCmd 携带 ramp 加速度参数、由底盘固件限幅；
  `/cmd_vel` 链路中由 rm_chassis_controllers 自身限幅。**非 rm_chassis 的第三方
  消费者（含无头部署直连 on_velocity_command）需自行做速度整形**，核心不内嵌
  减速 ramp（保持精简，见 2026-09-28 决策）。

## 扩展清单（3 个）

| 扩展 | 状态 | 职责 |
|---|---|---|
| `libayanami_rviz_viewer.so` | 出厂启用 | 路径/状态可视化 + `/cmd_vel` 速度本体 |
| `libayanami_cmd_chassis_publisher.so` | 出厂启用 | `rm_msgs/ChassisCmd` 命令参数通道 |
| `libayanami_rose_viewer.so` | 是 | 4 层代价图 OccupancyGrid + ESDF 距离场点云/占用栅格（latched） |

（rosemap_viewer 已随 rosemap 族删除。）
