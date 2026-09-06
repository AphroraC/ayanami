# 故障排查与已知坑

本文收录三类内容：运行故障的排查路径、源码/环境层面的已知坑、以及三次
真实卡死案例的完整复盘（它们塑造了当前的多条设计纪律）。

## 快速排查路径

### 症状：机器人不动 / 中途卡住

1. `rostopic hz /scan` —— 有数据？无 → 检查 gazebo 激光插件是否初始化
   （llvmpipe 下首启可能延迟数十秒）。
2. `rosnode list` —— 五件套齐全？（amcl / ayanami_ros / gazebo /
   map_server / robot_state_publisher）
3. TF 链：`rosrun tf tf_echo map odom` 与 `odom base_footprint`。
   缺 map←odom → AMCL（看 /tmp 日志中 "No laser scan received" 或
   "TF 健康诊断"）。ayanami 侧日志（spdlog）有同义诊断：
   `TF 健康诊断: 边不可用 [...]`、`markObstacles: robot pose FAIL`。
4. `rostopic echo -n1 /ayanami/controller/status` —— 最近一次控制器终态
   （NoProgress / TRANSFORM_UNAVAILABLE / RESOURCE_BUSY ...）。
5. `rostopic hz /ayanami/local_costmap` —— 可视化停更 = ayanami_ros loop
   线程被阻塞/饿死（见坑 T4）。
6. RViz 里局部代价图是否跟随机器人（窗口钉死 = 落图管线被阻塞，见案例 B）。

### 症状：laserscan 在 map 系"大幅旋转"

三层叠加因果链（均已实测定位）：

1. **物理层**：轮式里程计原地旋转打滑欠报转角（实测 ~6%，一圈 0.5-0.8
   rad）——URDF/物理属性，无法参数消除。
2. **定位层（AMCL）**：`recovery_alpha_slow/fast=0/0` 永久禁用随机粒子
   注入（pf.c: w_diff = 1 - w_fast/w_slow 恒 0，滤波器塌缩后永不恢复）；
   `odom_alpha1` 的源码 bug（amcl_odom.cpp DIFF 分支把方差当 sigma 传
   pf_ran_gaussian，缺 sqrt）使纯旋转粒子角噪声过小。修复：0.001/0.1 +
   alpha1=1.0 + laser_max_beams=60。修复后旋转中定位误差 0.63 rad →
   **0.081 rad 有界**，且 map←odom 修正确实包含旋转分量（实测 dyaw 随
   漂移增长到 1.39 rad）。
3. **落图时序层（ayanami）**：markObstacles 曾用 latest-TF 摆放 5.5Hz
   周期前采集的 scan——旋转时 ω×(now-T_scan) ≈ 0.2 rad 的歪斜被烙进代价
   地图（RViz 里 ayanami 代价地图"点云大幅旋转"的直接来源；ROS TF/RViz
   本身按 scan 戳插值，raw scan 显示是稳定的——实测 scan-stamp 语义下
   旋转中点云漂移仅 0.01~0.04m）。修复：传感器变换按 scan 戳查询，
   失败回退 latest（见 map-and-costmap.md）。修复后旋转中局部代价图
   漂移 ≤1 cell。
- 框架兜底：累计旋转 1.0 rad / 平移 2.0 m 自动清空全局动态层
  （日志 `clearing global dynamic costmap`），错位标记不积累。

### 症状：机器人停下后 scan 不"收敛到贴墙"

**这是 AMCL 的设计行为**：它是运动门控的贝叶斯滤波器（amcl_node.cpp:
`update = |Δodom| > update_min_d/a`），静止时跳过量测更新，位姿估计
**冻结在停下那一刻**——不会像配准优化器那样静止时精修对齐。实测
（1.2 rad/s 旋转 12s 后静止 30s）：amcl 估计逐比特不变，scan-墙偏差
维持 ~0.05 rad（3m 处 ~15cm）；恢复运动 0.5m 后修正到 0.007 rad
（scan 贴墙至 9mm）。缓解：`update_min_a=0.05`（降低慢速旋转的更新
门限）；残余冻结误差 = 旋转跟踪滞后，随 alpha1/scan 速率改善。

### 调参口径：收敛速度 vs 晃动（A/B 方法论）

- **晃动度量** = 定位误差（amcl_pose − 真值）的 p2p/std——不要用原始
  yaw 拟合残差（跨 ±π 解卷绕 + 二次拟合病态会产出无意义数字）。
- **快收敛与少晃动的共同杠杆是"每次更新的信息量"**：粒子数（KLD 在
  min/max 间自适应）、laser_max_beams——而不是 alpha 系数（alpha 大 =
  覆盖漂移快但估计散，跷跷板；alpha1=1.0 因源码 sqrt-bug 补偿而设，
  不回调）。
- 实测 A/B（1.2 rad/s 旋转 / 0.3 m/s 直行）：粒子 1000-10000 + 100 束 +
  resample_interval=2 → 位置误差 max 0.052→0.015 m（3×），旋转 yaw p2p
  0.042→0.037 rad，帧间跳变 ≤0.035 rad（1.2 rad/s 硬旋转场景）。
- 两个 AMCL 钳制坑：min_particles 上限 1000（AMCL.cfg 重配置范围
  [0,1000]）；launch 同名 param 后值覆盖前值（resample_interval 曾中招）
  ——改参数后务必 `rosparam get` 核实 live 值。

### 症状：代价地图有"幽灵墙"/错位障碍

- 全局图：定位跳变/漂移期间落图的标记错位 → 框架自动清层已兜底；
  也可手动触发 clear_costmap 恢复。
- 局部图：确认 `costmap.local_frame`（odom）与 TF 同步边一致；
  InflationLayer 只做 cost>old 覆盖（Ruling #7），障碍移走依赖
  raytrace/clearArea，管线被阻塞时会有 stale 带残留。

### 日志在哪

- roslaunch 标准输出（无头运行重定向到 /tmp/*.log）；
- spdlog 同时按 logging 段轮转写 `logs/`；
- ctest 输出在 build/<pkg>/Testing/Temporary/LastTest.log。

## 已知坑（源码/环境级）

| # | 坑 | 位置 | 状态 |
|---|---|---|---|
| P1 | BT.CPP v4 `haltChildren` 只对 RUNNING child 生效，终态缓存子树不重置 → 常驻树下 planner 永不重规划 + 恢复风暴 | BT.CPP v4 语义 + behavior_tree/* | 已修：resetState() + resetNavigationTreeState（pilot 终端调用）；直驱树的代码必须复刻该契约 |
| P2 | fast_tf 动态边同时间戳 set 不覆盖（emplace 语义） | thirdparty/fast_tf | 已知；测试代码需用递增时间戳 |
| P3 | fast_tf "新父帧+已存在子帧"直接 set 抛 cannot merge unconnected trees | thirdparty/fast_tf | 按同步边序挂载（map←odom 先行） |
| P4 | tf2 带超时等待以 ros::Time（sim time）计时，/clock 停摆时无限阻塞 | ayanami_ros sync | 已修：timeout=0（诊断兜底） |
| P5 | gazebo_ros_diff_drive 轮距/打滑 → 原地旋转里程计欠报 ~6% | urdf_gazebo_demo | 环境已知；AMCL 参数补偿 + 框架自动清图兜底 |
| P6 | AMCL alpha_slow/fast=0 → 恢复注入永久禁用（w_diff≡0） | amcl_diff.launch / pf.c:408 | 已修 0.001/0.1 |
| P7 | amcl_odom DIFF 分支把方差当 sigma 传 pf_ran_gaussian（缺 sqrt）→ 纯旋转噪声 ~12× 过小 | amcl_odom.cpp:168-173 | 已修 alpha1=1.0 参数补偿 |
| P8 | AMCL 参数须用 `initial_pose_a`（非 _yaw），否则内部 NaN 不发布 map→odom | amcl_diff.launch | 已修 |
| P9 | 多个残留 roslaunch/gzserver 同发同名 TF → 帧缺失/乱序（ayanami 日志会提示） | 环境 | 重启前彻底 pkill |
| P10 | 二次 configureCoreMaps 会替换 context 使插件指针悬挂 | costmap_navigation_server.cpp | 已加守护（warn + return） |

## 案例复盘（塑造当前设计的真实事故）

### 案例 A：恢复风暴（28 次恢复、FollowPath 零重入、原地转圈）

- 现象：导航中途卡住，RViz 里 laserscan 随机器人持续自转"大幅旋转"。
- 取证：rosbag 显示 28 次恢复（1 backup + 27 spin），controller path/status
  恒 1 条——主任务从未重入；P1（v4 halt）导致树生命终端无法重置，
  planner 永不重规划。
- 修复：P1（resetState + pilot 终端重置）+ planner_frequency 1.0 +
  production_tree_recovery_test 锁定。

### 案例 B：机器人行驶 ~1.4m 后静止抖动

- 现象：两次实验都在 x≈-1.47 停住——恰好是局部窗口（钉死在原点的 3×3m）
  西边缘；DWB 前进轨迹全部出界无效。
- 根因：dwb 插件把持有 RAII 读锁的 `CostmapView` 存成成员 → 局部图写者
  （落图/滚动/清图）永久阻塞 → 窗口永不滚动。
- 修复：视图改函数局部（锁生命周期 = 单次规划周期）；
  local_window_rolling_test 锁定。

### 案例 C+：到达目标后 CPU 空转（plan_version 以 60/s 膨胀）

- 现象：导航 SUCCEEDED 后 `/ayanami/planner/path` 以 ~100Hz 持续发布
  （plan=5382），planner/smoother/controller 空转重跑
  "规划→平滑→立即到达"循环。
- 根因：pilot 无"导航完成空闲"状态——树 SUCCESS → 终端复位 → 下一 tick
  立即全新树生命 → 立即再次 SUCCESS → 循环。
- 修复：树返回终端状态后 pilot 进入空闲（消费 halt/kill，等待新 goal）；
  second-goal 场景验证空闲→恢复正常。见 behavior-tree.md §2b。

### 案例 D：机器人楔入障碍 inscribed 膨胀带后 DWB 全轨迹非法

- 现象：目标点选在墙体区域内时，机器人"成功"抵达（checker 容差 0.35m
  大于目标点到墙的距离）并楔入墙体；下一个目标 DWB 报
  "No valid trajectories out of 440"——机器人中心处于 inscribed(253) 带,
  每条轨迹的首个位姿即非法。BT 按设计跑完恢复（backup/spin 移出 1.1m）
  后优雅 FAILURE。
- 要点：这不是缺陷而是 checker 容差 + 环境的组合结果; 选目标点应避开
  障碍体内部。DWB 逐轨迹评分可经 `AYANAMI_DWB_DEBUG=1` 观测（1Hz 输出
  twists/best/velocity）。

### 案例 C：controller 恢复间隙漏发旧速度

- 现象：recovery 启动瞬间可能漏出一条旧速度命令。
- 修复：插件调用返回后、发布前复查 `cancel_requested/should_exit`；
  `setMaxRetries` 删除（重试归 BT），patience 保留（对标 nav2
  failure_tolerance）。

## 环境备注（dynamicx 容器）

- VS Code 进程曾占用 11311 → roscore 无法绑定，ROS 侧测试超时。
  换端口：`roscore -p 11313` + `export ROS_MASTER_URI=http://localhost:11313`。
- 无 GPU（llvmpipe）软件渲染：gazebo 启动慢，TF 建立有几十秒延迟
  （ayanami 有启动期 TF 健康诊断，属正常）；物理步进可能停顿。
- 无头运行：`roslaunch nav_demo ayanami.launch rviz:=false`。
- 清理：`pkill` 模式注意别匹配到自身命令行；建议按 ps+awk+kill 两步走。
