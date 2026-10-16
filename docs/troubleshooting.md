# 故障排查与已知坑

本文收录三类内容：运行故障的排查路径、源码/环境层面的已知坑、以及真实
卡死案例的完整复盘（它们塑造了当前的多条设计纪律）。

## 快速排查路径

### 症状：机器人不动 / 中途卡住

1. `rostopic hz /scan`（或 `/nagisa/obstacle`）—— 有数据？无 → 检查传感器
   仿真/驱动是否初始化（llvmpipe 下首启可能延迟数十秒）。
2. `rosnode list` —— 五件套齐全？（amcl / ayanami_ros / gazebo /
   map_server / robot_state_publisher）
3. TF 链：`rosrun tf tf_echo map odom` 与 `odom base_link`。
   缺 map←odom → AMCL（看 ayanami 日志的 "TF health check" 诊断，
   每 5s 附完整帧树）。
4. FSM 状态：日志 `[FSM]` 标签（文件 `ayanami_FSM.log`）下有完整状态迁移（"FSM: navigation attempt
   started / goal revised in place / planning failed / follow failed /
   recovery done / attempt finished"）。终态原因（"retry budget exhausted"
   / "non-recoverable failure"）直接写在 attempt finished 行。
5. `rostopic echo -n1 /ayanami/controller/status` —— 最近一次控制器终态
   （NoProgress / TRANSFORM_UNAVAILABLE / TIMEOUT ...）。
6. `rostopic hz /ayanami/local_costmap/inflation` —— 可视化停更 =
   ayanami_ros loop 线程被阻塞/饿死（或 rose_viewer 扩展未启用，见
   config_extensions.json）。
7. RViz 里局部代价图是否跟随机器人（窗口钉死 = 落图管线被阻塞，见案例 B）。

### 症状：FSM 状态机不前进（停在 kPlan/kFollow 无迁移）

- kPlan 反复失败：看 `planning failed (code N)` 的 ErrorCode——
  PATH_NOT_FOUND 多为目标不可达或 inscribed 带内目标（见
  configuration.md 的 inscribed_radius/tolerance 配套注记）；
  TRANSFORM_UNAVAILABLE 为 TF 缺边。
- kFollow 无速度输出：controller execution 静默失败语义（nav2 对齐）下，
  无效命令周期**不发布**——看 `on_exe_path_feedback` 是否还在发；
  `controller_patience`（出厂 1.0s）耗尽才会终态上报交恢复。
- 振荡门误触发（NO_PROGRESS + "oscillation gate" 日志）：机器人确在
  `oscillation_distance` 内驻留 `oscillation_timeout` 秒；静止目标/测试
  环境应放宽或关闭振荡门。

### 症状：恢复预算耗尽（"retry budget exhausted"）

每次尝试预算 `max_nav_retries`（出厂 6），恢复成功→重规划→再失败才消耗；
恢复**失败本身**直接 ABORTED 不消耗。连续耗尽说明根因在恢复解决不了的
层面（目标在障碍体内、定位持续漂移、地图 stale）。注意出厂恢复列表只有
`abstract_nothing_recovery` 占位——它一拍即成功，所以预算耗尽等于"重规划
了 6 次仍失败"；换真恢复行为是换插件（见 modules.md）。

### 症状：laserscan 在 map 系"大幅旋转"

三层叠加因果链（均已实测定位）：

1. **物理层**：轮式里程计原地旋转打滑欠报转角（实测 ~6%，一圈 0.5-0.8
   rad）——URDF/物理属性，无法参数消除。
2. **定位层（AMCL）**：`recovery_alpha_slow/fast=0/0` 永久禁用随机粒子
   注入（pf.c: w_diff = 1 - w_fast/w_slow 恒 0，滤波器塌缩后永不恢复）；
   `odom_alpha1` 的源码 bug（amcl_odom.cpp DIFF 分支把方差当 sigma 传
   pf_ran_gaussian，缺 sqrt）使纯旋转粒子角噪声过小。修复：0.001/0.1 +
   alpha1=1.0 + laser_max_beams=60。修复后旋转中定位误差 0.63 rad →
   **0.081 rad 有界**，且 map←odom 修正确实包含旋转分量。
3. **落图时序层（ayanami）**：markObstacles 曾用 latest-TF 摆放周期前
   采集的 scan——旋转时 ω×(now-T_scan) 的歪斜被烙进代价地图。修复：
   传感器变换按 scan 戳查询（有界等待，不齐整帧丢弃），机器人位姿
   （滚动居中/清图中心）仍用 latest（见 map-and-costmap.md）。
- 现状兜底：动态标记只存在于**局部滚动图**（odom 系，对定位跳变免疫）；
  全局图纯静态（static+inflation），不存在会积累错位动态标记的全局层。
  局部残留靠滚动窗自然淘汰 + `decay_time` 时间衰减。

### 症状：机器人停下后 scan 不"收敛到贴墙"

**这是 AMCL 的设计行为**：它是运动门控的贝叶斯滤波器（amcl_node.cpp:
`update = |Δodom| > update_min_d/a`），静止时跳过量测更新，位姿估计
**冻结在停下那一刻**——不会像配准优化器那样静止时精修对齐。实测
（1.2 rad/s 旋转 12s 后静止 30s）：amcl 估计逐比特不变，scan-墙偏差
维持 ~0.05 rad；恢复运动 0.5m 后修正到 0.007 rad。缓解：
`update_min_a=0.05`（降低慢速旋转的更新门限）；残余冻结误差 = 旋转跟踪
滞后，随 alpha1/scan 速率改善。

### 调参口径：收敛速度 vs 晃动（A/B 方法论）

- **晃动度量** = 定位误差（amcl_pose − 真值）的 p2p/std——不要用原始
  yaw 拟合残差（跨 ±π 解卷绕 + 二次拟合病态会产出无意义数字）。
- **快收敛与少晃动的共同杠杆是"每次更新的信息量"**：粒子数（KLD 在
  min/max 间自适应）、laser_max_beams——而不是 alpha 系数（alpha 大 =
  覆盖漂移快但估计散，跷跷板；alpha1=1.0 因源码 sqrt-bug 补偿而设，
  不回调）。
- 实测 A/B（1.2 rad/s 旋转 / 0.3 m/s 直行）：粒子 1000-10000 + 100 束 +
  resample_interval=2 → 位置误差 max 0.052→0.015 m（3×），旋转 yaw p2p
  0.042→0.037 rad。
- 两个 AMCL 钳制坑：min_particles 上限 1000（AMCL.cfg 重配置范围
  [0,1000]）；launch 同名 param 后值覆盖前值（resample_interval 曾中招）
  ——改参数后务必 `rosparam get` 核实 live 值。

### 症状：代价地图有"幽灵墙"/错位障碍

- 局部图：确认 `costmap.local_costmap_frame`（odom）与 TF 帧名一致；
  InflationLayer 只做 cost>old 覆盖（Ruling #7），障碍移走依赖
  raytrace/clearArea + `decay_time`，管线被阻塞时会有 stale 带残留。
- 全局图：纯静态（PGM + 膨胀），"幽灵墙"只可能来自静态图本身或膨胀参数
  （inscribed_radius ≠ robot_radius 会让车体壳层蹭墙，见
  configuration.md 注记）。
- `costmap_local.filter_static_marks`（默认 true）会把落在全局阻挡栅格内
  的局部命中滤掉——怀疑"动态障碍没进局部图"时先核对该开关与静态图内容。

### 日志在哪

- roslaunch 标准输出（无头运行重定向到 /tmp/*.log）；
- spdlog 同时按 logging 段轮转写 `logs/`；
- 每行带分层 logger 标签：`[ros] / [planner] / [smoother] / [controller] /
  [recovery] / [FSM] / [costmap]`（框架基础设施为 `[ayanami]`；每个 layer 写独立文件 `ayanami_<layer>.log`），级别统一由 `logging_level`
  控制；
- ctest 输出在 build/<pkg>/Testing/Temporary/LastTest.log。

## 已知坑（源码/环境级）

| # | 坑 | 位置 | 状态 |
|---|---|---|---|
| P4 | tf2 带超时等待以 ros::Time（sim time）计时，/clock 停摆时无限阻塞 | ayanami_ros sync | 已修：timeout=0（诊断兜底） |
| P5 | gazebo_ros_diff_drive 轮距/打滑 → 原地旋转里程计欠报 ~6% | nav_demo (car.xacro) | 环境已知；AMCL 参数补偿兜底 |
| P6 | AMCL alpha_slow/fast=0 → 恢复注入永久禁用（w_diff≡0） | amcl_diff.launch / pf.c:408 | 已修 0.001/0.1 |
| P7 | amcl_odom DIFF 分支把方差当 sigma 传 pf_ran_gaussian（缺 sqrt）→ 纯旋转噪声 ~12× 过小 | amcl_odom.cpp:168-173 | 已修 alpha1=1.0 参数补偿 |
| P8 | AMCL 参数须用 `initial_pose_a`（非 _yaw），否则内部 NaN 不发布 map→odom | amcl_diff.launch | 已修 |
| P9 | 多个残留 roslaunch/gzserver 同发同名 TF → 帧缺失/乱序（ayanami 日志会提示） | 环境 | 重启前彻底 pkill |
| P10 | 二次 configureCoreMaps 会替换 context 使模块指针悬挂 | costmap_navigation_server.cpp | 已加守护（warn + return） |
| P11 | 持地图锁做 TF 查询（map 锁 → robot-state 锁） | costmap_map_context buildReader | 已修：机器人位姿在**加锁之前**解析。无回归测试（timeout=0 的查询不可观测），靠代码审查保证 |
| P12 | 平滑器只看得见局部表达（ESDF/costmap），其产物此前没有规划视图校验 → 平滑可以把路径推进障碍里 | smoother 模块 + FsmDriver | 已修：FSM 对平滑产物做 `isPathValid` 校验，非法回退 raw path（DegradeToRawPath）；平滑器自身的碰撞回滚仍是 SUCCESS（优雅降级，不是失败） |
| P13 | 到达容差两处独立配置，出厂值过宽使朝向检查失效 | simple_goal_checker.json + 控制器 | 已修机制：容差**单一来源**（checker 经执行层下推给控制器，控制器视其为上界）；启动日志会点名过宽配置 |
| P14 | 模块 json 的参数写在**家族段**里 → 模块读自己的段读不到，静默跑默认值 | 多处夹具 | 已加校验：加载时要求存在以模块名命名的参数段，缺失即拒绝；另有「未消费键」审计 |
| P15 | Execution 复用模型下，重复同 goal 的新尝试不触发 run loop 的 checker reset → 上一尝试的基线泄漏 | ControllerExecution + FsmDriver | 已修：`startAttempt` 显式 `controller_->resetCheckers()`（fsm_test 的 second-attempt 场景锁定） |

## 案例复盘（塑造当前设计的真实事故）

### 案例 E：`catkin_add_gtest` 的测试二进制不会随 `catkin build` 重建

`catkin_add_gtest` 生成的可执行文件带 `EXCLUDE_FROM_ALL`，因此普通
`catkin build ayanami_ros` **从不重建它们**——改完测试源码跑 `ctest`，跑的是旧
二进制，绿灯毫无意义。已在 `ayanami_ros/CMakeLists.txt` 里把每个测试目标拉回
`all`。核心包用的是 `add_executable`，本来就在 `all` 里，所以只有 ROS 包有这个问题。

**教训**：验证「测试通过」之前，先确认二进制比源码新。

### 案例 F：跑 ROS 测试时主机没有 master，测试全部超时

`~/.bashrc` 里 `ROS_MASTER_URI` 指向 11311，而那个端口上没有 master（它原被 VS Code
占用）。需要 ROS 初始化的测试会站在 master 重试上（表现为「超时」而不是失败）。
按 AGENTS.md 的约定自起一个：

```bash
ROS_MASTER_URI=http://localhost:11313 roscore -p 11313 &
ROS_MASTER_URI=http://localhost:11313 ctest --timeout 120
```

顺带说明：有 master 时这批测试只需几秒，无 master 时要 ~55 秒——耗时本身就是
诊断信号。

### 案例 A：恢复风暴（28 次恢复、FollowPath 零重入、原地转圈）

- 现象：导航中途卡住，RViz 里 laserscan 随机器人持续自转"大幅旋转"。
- 取证：rosbag 显示 28 次恢复（1 backup + 27 spin），controller path/status
  恒 1 条——主任务从未重入；BT.CPP v4 的 halt 语义缺陷导致树生命终端无法
  重置，planner 永不重规划。
- 修复：**BT 架构整体退役**是本轮重构的最终答案——单体 FSM 的重试预算 +
  恢复轮转 + 每尝试干净开局从构造上消除了"编排状态跨生命期泄漏"整类问题
  （fsm_test 的预算/轮转/二次尝试场景锁定）。

### 案例 B：机器人行驶 ~1.4m 后静止抖动

- 现象：两次实验都在 x≈-1.47 停住——恰好是局部窗口（钉死在原点的 3×3m）
  西边缘；前进轨迹全部出界无效。
- 根因：控制器模块把持有 RAII 读锁的读视图存成成员 → 局部图写者
  （落图/滚动/清图）永久阻塞 → 窗口永不滚动。
- 修复：视图改函数局部（锁生命周期 = 单次调用）+ 按值返回视图的接口删除；
  local_window_rolling_test 锁定。

### 案例 C+：到达目标后 CPU 空转（plan_version 以 60/s 膨胀）

- 现象：导航 SUCCEEDED 后 `/ayanami/planner/path` 以 ~100Hz 持续发布，
  planner/smoother/controller 空转重跑"规划→平滑→立即到达"循环。
- 根因：编排器无"导航完成空闲"状态——每次 tick 立即开启全新尝试。
- 修复：**终态驻留**——finishAttempt 后 FSM 停在 kSucceeded/kAborted/
  kCanceled，不 tick、不规划，下一个 goal 才开新尝试（fsm_e2e_test 的
  consecutiveAttempts 场景锁定）。

### 案例 C：controller 恢复间隙漏发旧速度

- 现象：recovery 启动瞬间可能漏出一条旧速度命令。
- 修复：模块调用返回后、发布前复查 `cancel_requested/should_exit`；
  controller 抢占（恢复入口与终态）的 cancel 内同步发**恰一次零脉冲**；
  Execution 无内部重试（重试归 FSM 预算），patience 保留
  （对标 nav2 failure_tolerance）。

## 环境备注（dynamicx 容器）

- VS Code 进程曾占用 11311 → roscore 无法绑定，ROS 侧测试超时。
  换端口：`roscore -p 11313` + `export ROS_MASTER_URI=http://localhost:11313`。
- 无 GPU（llvmpipe）软件渲染：gazebo 启动慢，TF 建立有几十秒延迟
  （ayanami 有启动期 TF 健康诊断，属正常）；物理步进可能停顿。
- 无头运行：`roslaunch nav_demo ayanami.launch rviz:=false`。
- 清理：`pkill` 模式注意别匹配到自身命令行；建议按 ps+awk+kill 两步走。
