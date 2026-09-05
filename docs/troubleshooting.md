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

- 本质是 map←odom 修正在旋转期间/之后偏转：轮式里程计原地旋转打滑欠报
  转角（实测 ~6%，一圈 0.5-0.8 rad），AMCL 修正渐进追赶。
- 已做的修复：AMCL `recovery_alpha_slow/fast` 0/0 → 0.001/0.1（0/0 会
  永久禁用随机粒子注入，pf.c: w_diff = 1 - w_fast/w_slow 恒 0）；
  `odom_alpha1` 0.2 → 1.0（源码 bug 补偿，见下）；`laser_max_beams` 60。
- 框架兜底：累计旋转 1.0 rad / 平移 2.0 m 自动清空全局动态层
  （日志 `clearing global dynamic costmap`），错位标记不积累。

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
