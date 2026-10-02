# 测试指南

## 套件概览

- **ayanami**：32 个 ctest（`build/ayanami`），纯 C++，无 ROS 依赖。
- **ayanami_ros**：7 个 ctest（`build/ayanami_ros`），需要 roscore
  （容器内注意 ROS_MASTER_URI；VS Code 可能占用 11311 端口导致 roscore 起不来，
  可改用 `roscore -p 11313` 并 `export ROS_MASTER_URI=http://localhost:11313`）。

## ayanami 测试清单（32 个，按类分组）

| 组 | 测试 |
|---|---|
| 基础设施 | config_test、logging_test、callback_slot_test、callbacks_test（跨 .so 槽单实例）、messages_test |
| 机器人状态 | robot_state_controller_test、robot_state_controller_bench（TF 查询基准：10 边 @500Hz，成功率 >98%、p99 <3ms）、robot_state_controller_cross_dso_test（跨 DSO 单例） |
| 插件机制 | load_module_test、module_manager_test（含 stale api 版本拒绝）、module_teardown_no_ub_test |
| 抽象组件 | core_interfaces_test、execution_test |
| costmap 包 | costmap_2d_test、costmap_layers_test、costmap_server_test |
| costmap 组件 | costmap_context_test、costmap_components_test、concurrent_controller_obstacle_test（控制器短锁不被落图饿死）、snapshot_stability_test、planner_writer_concurrency_test、local_window_rolling_test |
| costmap 层静态组合 | costmap_layers_static_test（buildGlobalLayers/buildLocalLayers 解析、重复源类型拒绝、applier 过滤与足印清除） |
| 生产模块 | costmap_astar_planner_test、costmap_minsnap_smoother_test、costmap_pid_controller_test、costmap_mppi_controller_test、minor_carry_wirings_test（astar+pid 小装配） |
| ESDF | esdfmap_test（重建发布/generation 门控/静态 mask 经变换融合/插值与梯度/路径校验探针/新鲜度门） |
| **FSM** | **fsm_test**（进程内 mock 直驱状态机语义）、**fsm_e2e_test**（rose 宿主无头 E2E）、**fsm_costmap_e2e_test**（costmap 宿主无头 E2E） |

期自动跳过该测试（不注册进 ctest）。

## 关键新测试（改 FSM/地图前先看）

| 测试 | 守护的行为 |
|---|---|
| fsm_test | 状态机语义七场景：正向流水线到达、goal 原地修订（预算复位 + 原地重规划后成功）、恢复轮转烧尽预算后 ABORTED、不可恢复失败短路恢复、平滑失败 DegradeToRawPath、取消结算 CANCELED、成功后二次尝试干净开局 |
| fsm_e2e_test | **rose FSM 无头 E2E**：按生产方式 dlopen libayanami_rose_navigation_fsm.so（costmap + EsdfWriter + FsmDriver + astar → esdfmap minco → abstract pid → nothing recovery），路由 json 无任何 BT 键；场景：跟随流水线发速度命令、同 goal 连续两次尝试都 SUCCEEDED |
| fsm_costmap_e2e_test | **costmap FSM 无头 E2E**：同一套宿主装配协议跑纯 costmap 栈（astar → minsnap → costmap pid → nothing recovery） |
| esdfmap_test | ESDF 派生流水线全语义（见上表） |
| costmap_layers_static_test | 层静态组合配置解析（层无 so_name 后的配置契约） |
| local_window_rolling_test | 局部滚动窗口跟随机器人（历史：窗口钉死原点导致机器人卡在窗口边缘） |
| snapshot_stability / planner_writer_concurrency / concurrent_controller_obstacle | CostmapReader 回调作用域零拷贝读 + 唯一写者并发 |

## 写测试的方法论

### 1) 单元级：FSM 直驱 + mock Execution

`fsm_test.cpp` 模式：不 dlopen，进程内构造 `FsmDriver`，注入测试用的
mock Execution（可控成功/失败/延迟的最小 `runBlocking` 实现），

- 直接驱动 `beginNavigation` / `cancelNavigation`，用 `state()` /
  `attemptActive()` / `goalRevision()` / `planVersion()` 原子观测点断言
  状态迁移与预算/游标行为；
- 用受控时钟/短频率配置避免裸 sleep 竞态；
- 恢复列表注入多个 mock recovery 验证**轮转游标**与**预算耗尽**。

Execution 层自身的状态机语义仍由 execution_test 直驱覆盖。

### 2) 集成级：真实模块 + 临时 config 目录

`makeConfigDir` 模式：写 config.json / 路由 json / 模块 json / PGM 到
`std::filesystem::temp_directory_path()` 子目录，`configureCoreMaps(dir)`
后直调 `handleObstaclePointCloud` 等公共入口断言地图状态
（参考 local_window_rolling_test / costmap_components_test）。
真模块 so 由 CMake 用 `$<TARGET_FILE:...>` 传入（宏 `AYANAMI_TEST_*`）。

### 3) 跨 DSO：真实 dlopen

模块目标在 CMake 中声明为 SHARED（`tests/modules/` 的
`ayanami_add_test_plugin`），测试经 `ExtensionModule::loadModule` 加载并
回收（module_teardown_no_ub_test 验证析构顺序无 UB）。

### 4) 无头 E2E

fsm_e2e_test / fsm_costmap_e2e_test：全栈进程内按生产加载方式组装
（dlopen 宿主 so + 6 个模块 so + tests/assets 无头地图），发目标、轮询
`on_navigation_result` 断言到达——不依赖 roscore/gazebo。静态测试机器人
场景要把振荡门关到足够宽（位置历史不动会触发 NO_PROGRESS）。

## 断言纪律

- 先写失败断言再修（TDD）。
- 失败消息里带上观测计数（`<< "path=" << n`），方便从 ctest 输出直接定位。
- 时序敏感断言用事件驱动（条件变量 / waitForStateUpdate），避免裸 sleep
  竞态。

## 构建状态陷阱（重要）

容器内 catkin/make 的增量判断在"跨包头文件变更"上不可靠：
- 改 `abstract/*.hpp` 后，**测试二进制与模块可能没有重编**（.o/.so 时间戳
  不变），出现"单跑通过、连跑失败 / 段错误（rip=0x0，新旧类布局混用）"。
- 处理：`touch` 对应头文件 + `catkin build`；仍不行就删除具体 .o/二进制
  后 `cd build/<pkg> && make <target>`。
- 验证产物一致性：比对 `devel/lib/libayanami.so`、
  `devel/lib/libayanami_rose_navigation_fsm.so` 等模块 so、测试二进制的
  mtime 是否都晚于最后一次头文件修改。
