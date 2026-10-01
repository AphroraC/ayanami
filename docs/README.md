# Ayanami Wiki

Ayanami 是一个**纯 C++、不依赖 ROS** 的单机器人导航框架，专注于**同一时间内的一次导航任务**。
它借鉴了 move_base_flex 的"抽象地图 + 依赖地图"模块分层与 nav2 的
planner / smoother / controller / recovery 四组件和行为树组织方式，并针对
稳健、精简、高效、可扩展的目标做了多项结构决策。

- 仓库布局：`src/ayanami`（纯 C++ 核心 + 模块，零 ROS 头文件）与
  `src/ayanami_ros`（唯一的 ROS1 边界层）。
- 算法模块通过 dlopen（`RTLD_LAZY | RTLD_GLOBAL | RTLD_NODELETE`）加载，
  以 `create_extension_module` 工厂符号暴露。
- 行为树（BehaviorTree.CPP v4，vendored）：工厂与树实例都只建一次（**常驻树**：
  首个 goal 建树，每尝试终态 `haltTree()` 归 IDLE 后原树再武装）；
  目标更新走 latest-wins 邮箱（in-flight 时为原地 goal 修订）；恢复调度完全由行为树决定。

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
| [configuration.md](configuration.md) | 配置系统：全部文件、键、默认值与单一来源纪律 |
| [navigation-flow.md](navigation-flow.md) | 一次导航的完整生命周期（goal → plan → smooth → follow → recover） |
| [behavior-tree.md](behavior-tree.md) | 行为树层：节点清单、树实例模型、建树期端口/角色校验、reset_token |
| [execution-runtime.md](execution-runtime.md) | 执行运行时：Execution/Action/并发槽/资源协议/错误码体系 |
| [map-and-costmap.md](map-and-costmap.md) | 地图族抽象、CostmapReader 回调作用域零拷贝读、唯一写者纪律、清图 |
| [modules.md](modules.md) | 模块开发指南：接口、工厂、加载后参数注入、Param 结构体、api_version、CMake 集成 |
| [ros-integration.md](ros-integration.md) | ayanami_ros 边界：AyanamiROS、TF 同步、AMCL 集成、rviz_viewer |
| [testing.md](testing.md) | 测试套件、关键回归测试、写测试的方法论 |
| [troubleshooting.md](troubleshooting.md) | 故障排查与已知坑（含三次真实卡死案例的复盘） |

## 一分钟架构图

```
        ROS 侧 (ayanami_ros)                        纯 C++ 核心 (ayanami)
/scan /odom /tf ──► AyanamiROS ──setOdometry──► RobotStateController (TF/速度)
 /tf (AMCL)   │        │  submitSensor              │
              │        └──setLaserScan──┐           ▼
/goal ────────┼──► submitGoal(邮箱)     │   CostmapNavigationServer
              │                         │   ├─ CostmapMapContext (global/local)
rviz_viewer ◄─┘ (旁路可视化)            │   ├─ planner/smoother/controller/
                                        │   │  recovery/checker 模块
                                        ▼   ▼
                                   BehaviorTreePilot（工厂常驻；每次尝试一棵新树）
                                        │ request* (TaskHandle 直连)
                                        ▼
                              ActionBase 并发槽 ──► Execution 线程 ──► 模块
                                        │
                              cmd_vel ──► 机器人（Gazebo 差速）
```

## 文档约定

- 所有成员变量使用无下划缀的 snake_case；函数 lower camel case；类/结构体
  upper camel case（Google 风格基础 + 120 列）。
- 代码中的 `spec §x` / `Ruling #n` 注释引用内部设计决议；关键决议已收录到
  [troubleshooting.md](troubleshooting.md) 与对应页面的"设计纪律"小节。
