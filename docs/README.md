# Ayanami Wiki

Ayanami 是一个**纯 C++、不依赖 ROS** 的单机器人导航框架，专注于**同一时间内的一次导航任务**。
它借鉴了 move_base_flex 的"抽象地图 + 依赖地图"模块分层与 nav2 的
planner / smoother / controller / recovery 四组件划分，编排采用**单体 FSM**
（`FsmDriver`，行为树的语义继承者），并针对稳健、精简、高效、可扩展的目标
做了多项结构决策。

- 仓库布局：`src/ayanami`（纯 C++ 核心 + 模块，零 ROS 头文件）与
  `src/ayanami_ros`（唯一的 ROS1 边界层）。
- 算法模块通过 dlopen（`RTLD_LAZY | RTLD_GLOBAL | RTLD_NODELETE`）加载，
  以 `create_extension_module` 工厂符号暴露。
- 单体 FSM（`FsmDriver`）：goal latest-wins 邮箱 + 常驻 PLAN/CONTROL 双线程；
  重试预算、恢复轮转、不可恢复短路、原地修订、节流重规划等策略全部是
  状态机里的普通代码（BT 构造 → 代码的逐项对照见 [fsm.md](fsm.md)）。

## 快速开始（docker 环境 dynamicx）

```bash
# 进入容器（项目挂载于 /share/nav_ws）
docker exec -it dynamicx bash

# 构建（catkin workspace）
source /opt/ros/noetic/setup.bash
cd /share/nav_ws
catkin build ayanami ayanami_ros

# 无头仿真运行完整导航栈（Gazebo + map_server + AMCL + ayanami_ros）
roslaunch nav_demo ayanami.launch rviz:=false

# 下发导航目标
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: -7.0, y: -3.0}, orientation: {w: 1.0}}}'

# 单元测试
cd /share/nav_ws/build/ayanami && ctest --output-on-failure
cd /share/nav_ws/build/ayanami_ros && ctest --timeout 120
```

## 文档目录

| 页面 | 内容 |
|---|---|
| [architecture.md](architecture.md) | 总体架构、分层、线程模型、目录结构 |
| [fsm.md](fsm.md) | 单体 FSM：状态机、线程模型、BT 语义平移对照、FsmDriverConfig、宿主注入 |
| [configuration.md](configuration.md) | 配置系统：全部文件、键、默认值与单一来源纪律 |
| [navigation-flow.md](navigation-flow.md) | 一次导航的完整生命周期（goal → plan → follow → recover → 终态） |
| [execution-runtime.md](execution-runtime.md) | 执行运行时：Execution 直驱/错误码体系/patience 与零脉冲语义 |
| [map-and-costmap.md](map-and-costmap.md) | 地图族抽象、costmap 合包与层静态组合、ESDF 派生流水线、唯一写者纪律 |
| [modules.md](modules.md) | 模块开发指南：接口、工厂、加载后参数注入、Param 结构体、api_version、CMake 集成 |
| [ros-integration.md](ros-integration.md) | ayanami_ros 边界：AyanamiROS、TF 同步、AMCL 集成、扩展 |
| [testing.md](testing.md) | 测试套件、关键回归测试、写测试的方法论 |
| [troubleshooting.md](troubleshooting.md) | 故障排查与已知坑（含真实事故案例的复盘） |

## 一分钟架构图

```
        ROS 侧 (ayanami_ros)                        纯 C++ 核心 (ayanami)
/scan /odom /tf ──► AyanamiROS ──setOdometry──► RobotStateController (TF/速度)
 /tf (AMCL)   │        │  submitSensor              │
              │        └──setPointcloud──┐           ▼
/goal ────────┼──► submitGoal(邮箱)      │   宿主 so（routes json 选择）
              │                         │   rose_navigation_fsm / costmap_navigation_fsm
rviz_viewer ◄─┘ (旁路可视化)            │   ├─ CostmapNavigationServer（唯一写者）
                                        │   ├─ [rose: EsdfWriter（派生有符号距离场）]
                                        │   └─ planner/smoother/controller/recovery 模块
                                        ▼
                                   FsmDriver（PLAN 线程 tick + CONTROL 线程）
                                        │ 注入三族 Execution + recovery 列表
                                        ▼
                              Execution::runBlocking ──► 模块
                                        │
                              cmd_vel ──► 机器人（Gazebo 差速）
```

## 文档约定

- 所有成员变量使用无下划缀的 snake_case；函数 lower camel case；类/结构体
  upper camel case（Google 风格基础 + 120 列）。
- 代码中的 `spec §x` / `Ruling #n` 注释引用内部设计决议；关键决议已收录到
  [troubleshooting.md](troubleshooting.md) 与对应页面的"设计纪律"小节。
