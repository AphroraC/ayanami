# 测试指南

## 套件概览

- **ayanami**：52 个 ctest（`build/ayanami`），纯 C++，无 ROS 依赖。
- **ayanami_ros**：5 个 ctest（`build/ayanami_ros`），需要 roscore
  （容器内注意 ROS_MASTER_URI；VS Code 可能占用 11311 端口导致 roscore 起不来，
  可改用 `roscore -p 11313` 并 `export ROS_MASTER_URI=http://localhost:11313`）。

关键回归测试（改对应模块前先看）：

| 测试 | 守护的行为 |
|---|---|
| production_tree_recovery_test | **生产默认树 + MockRuntime**：恢复后 FollowPath 必须重入（exe≥2）、planner 必须重规划（path≥2）、每树生命恢复次数 ≤ 预算（历史：28 次恢复零重入的恢复风暴） |
| local_window_rolling_test | 局部滚动窗口跟随机器人（历史：窗口钉死原点导致机器人卡在窗口边缘）；定位跳变清空全局动态层 |
| goal_update_follow_test | 导航中重设 goal → planner 重规划 + smoother 重平滑 + controller in-place 跟随（RViz 抢占场景回归，60s） |
| pipeline_sequence_scenarios_test | S1-S6 场景（planner/controller 并行、FAILURE 短路、重规划不中断跟随、goal 更新）+ RateController/RoundRobin/RecoverySequence reset_token |
| latest_wins_test / continuous_goal_test | goal 邮箱与连续目标修订语义 |
| recovery_exhaustion_test / recovery_exception_test | 恢复预算耗尽、插件异常兜底（k_aborted）后互斥释放 |
| cancel_responsiveness_test | 取消及时性 |
| snapshot_stability / planner_writer_concurrency / concurrent_controller_obstacle / recovery_vs_obstacle | CostmapView 零拷贝读 + 唯一写者并发 |
| headless_e2e_test | 无 ROS 全栈在进程内跑通（config → 插件 → 树 → 目标 → 到达） |
| map_family_abstraction_test | Grid 假族经同一虚钩子装配，核心零 costmap 依赖 |
| robot_state_controller_cross_dso_test | 跨 DSO 单例解析一致性 |
| legacy 契约六件套 | action_protocol / cancel_responsiveness / continuous_goal / controller_session / latest_wins / recovery_exhaustion |

## 写测试的方法论

### 1) 单元级：直驱类 + mock

Execution/Action/BT 层测试不 dlopen，直接在进程内构造。轻量
`MockRuntime : NavigationRuntime` 模式（见 production_tree_recovery_test）：

- `request*` 把 pending 项（age + settle lambda）入队；
- `pump()` 推进年龄并按策略结算（可编程成功/失败/延迟）；
- 计数器（path/smooth/exe/recovery requests + per-id）做断言；
- BT 驱动契约：`tickTree()` = `tickExactlyOnce()` + 终端状态时
  `resetNavigationTreeState(tree)`（与 pilot 一致）；
- 调试：环境变量 `AYANAMI_BT_TRACE=1` 挂 `BT::StdCoutLogger` 看逐节点迁移。

### 2) 集成级：真实插件 + 临时 config 目录

`makeConfigDir` 模式：写 config.json / navigation.json / 模块 json / PGM 到
`std::filesystem::temp_directory_path()` 子目录，`configureCoreMaps(dir)`
后直调 `handleObstacleLaserScan` 等公共入口断言地图状态
（参考 local_window_rolling_test / costmap_navigation_server_test）。
真插件 so 由 CMake 用 `$<TARGET_FILE:...>` 传入（宏 `AYANAMI_TEST_*`）。

### 3) 跨 DSO：真实 dlopen

插件目标在 CMake 中声明为 SHARED（`tests/plugins/`），测试经
`ExtensionModule::load_module` 加载并回收（plugin_teardown_no_ub_test
验证析构顺序无 UB）。

### 4) 无头 E2E

headless_e2e_test：全栈进程内（宿主插件 + BT + 执行层 + 假地图 +
假 TF），发目标、等待到达断言——不依赖 roscore/gazebo。

## 断言纪律

- 先写失败断言再修（本次恢复风暴修复即 TDD：先红 production_tree_recovery，
  后绿）。
- 失败消息里带上观测计数（`<< "path=" << n`），方便从 ctest 输出直接定位。
- 时序敏感断言用事件驱动（TaskHandle::onComplete / waitForStateUpdate），
  避免裸 sleep 竞态。

## 构建状态陷阱（重要）

容器内 catkin/make 的增量判断在"跨包头文件变更"上不可靠：
- 改 `abstract/*.hpp` 后，**测试二进制与插件可能没有重编**（.o/.so 时间戳
  不变），出现"单跑通过、连跑失败 / 段错误（rip=0x0，新旧类布局混用）"。
- 处理：`touch` 对应头文件 + `catkin build`；仍不行就删除具体 .o/二进制
  后 `cd build/<pkg> && make <target>`。
- 验证产物一致性：比对 `devel/lib/libayanami.so`、
  `devel/lib/libayanami_costmap_navigation.so`、测试二进制的 mtime 是否都
  晚于最后一次头文件修改。
