// Map Family 抽象测试 : 证明 NavigationServerBase 装配路径零 costmap
// 依赖 —— 一个非 costmap 的 fake 地图族 (Grid 族) 经同一组虚钩子
// (initialize*Plugin + PluginContext 注入) 完成插件配置, 地图上下文经
// AbstractMapContext 接口提供 identity/frame/版本/能力/写命令/机器人状态。
//
// 本测试不 dlopen 任何 .so (装配缝在钩子层, 与加载机制正交)。

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <string>
#include <utility>

#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/map/abstract_map_context.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/config.hpp>

namespace {

using ayanami::ErrorCode;
using ayanami::map::AbstractMapContext;
using ayanami::map::MapCapability;
using ayanami::map::MapCommandPort;

// ---- fake "grid" 地图族上下文 (与 costmap 族平行的最小实现) ----

struct GridWriteCommand {
  double clear_radius = 0.5;
};

class GridCommandPort : public MapCommandPort {
 public:
  using Ptr = std::shared_ptr<GridCommandPort>;

  bool submit(const GridWriteCommand& command) {
    ++submissions;
    last_radius = command.clear_radius;
    return accepted;
  }

  std::atomic<int> submissions{0};
  std::atomic<double> last_radius{0.0};
  std::atomic<bool> accepted{true};
};

class GridMapContext final : public AbstractMapContext {
 public:
  GridMapContext(std::string global_frame, std::string robot_frame)
      : global_frame(std::move(global_frame)),
        robot_frame(std::move(robot_frame)) {}

  std::string mapFamily() const override { return "grid"; }
  std::string globalFrame() const override { return global_frame; }
  std::string localFrame() const override { return global_frame; }
  std::string robotFrame() const override { return robot_frame; }
  bool isCurrent() const override { return is_current.load(); }
  std::uint64_t version() const override { return context_version.load(); }
  std::uint32_t capabilities() const override {
    return static_cast<std::uint32_t>(MapCapability::GlobalPlanning) |
           static_cast<std::uint32_t>(MapCapability::ClearRegion);
  }
  MapCommandPort::Ptr commandPort() const override { return command_port; }
  bool getRobotState(ayanami::geometry_msgs::PoseStamped& pose,
                     ayanami::geometry_msgs::TwistStamped&) const override {
    pose.header.frame_id = global_frame;
    pose.pose.position.x() = robot_x;
    pose.pose.position.y() = robot_y;
    return true;
  }

  void setCommandPort(GridCommandPort::Ptr port) { command_port = std::move(port); }
  void setCurrent(bool value) { is_current = value; }
  void bumpVersion() { ++context_version; }

 private:
  std::string global_frame;
  std::string robot_frame;
  std::atomic<bool> is_current{true};
  std::atomic<std::uint64_t> context_version{1};
  double robot_x{1.0};
  double robot_y{2.0};
  GridCommandPort::Ptr command_port;
};

// ---- fake 族的族级插件层 (抽象层零地图, 族层绑定 GridMapContext) ----

class GridPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  virtual void configure(const ayanami::PluginContext& context,
                         std::shared_ptr<const GridMapContext> map) = 0;

 protected:
  std::shared_ptr<const GridMapContext> map_context;
  std::string configured_name;
};

class StraightLineGridPlanner final : public GridPlanner {
 public:
  void configure(const ayanami::PluginContext& context,
                 std::shared_ptr<const GridMapContext> map) override {
    configured_name = context.name;
    map_context = std::move(map);
    configured = true;
  }

  ayanami::PlannerResultT makePlan(
      const ayanami::geometry_msgs::PoseStamped& start,
      const ayanami::geometry_msgs::PoseStamped& goal,
      double /*tolerance*/) override {
    if (!configured) {
      return ayanami::PlannerResultT::failure(
          ErrorCode::INTERNAL_ERROR, "grid planner not configured");
    }
    ayanami::nav_msgs::Path path;
    path.header = goal.header;
    path.poses.push_back(start);
    path.poses.push_back(goal);
    return ayanami::PlannerResultT::success(std::move(path));
  }

  bool configured = false;
};

class GridRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  virtual void configure(const ayanami::PluginContext& context,
                         std::shared_ptr<const GridMapContext> map) = 0;

 protected:
  std::shared_ptr<const GridMapContext> map_context;
};

class GridClearRecovery final : public GridRecovery {
 public:
  void configure(const ayanami::PluginContext& context,
                 std::shared_ptr<const GridMapContext> map) override {
    configured_name = context.name;
    map_context = std::move(map);
    command_port = std::dynamic_pointer_cast<GridCommandPort>(map_context->commandPort());
  }

  ayanami::RecoveryResultT tick() override {
    if (!command_port) {
      return ayanami::RecoveryResultT::failure(
          ErrorCode::INTERNAL_ERROR, "grid recovery has no command port");
    }
    GridWriteCommand command;
    command.clear_radius = 0.75;
    if (!command_port->submit(command)) {
      return ayanami::RecoveryResultT::failure(
          ErrorCode::RECOVERY_FAILED, "grid write command rejected");
    }
    ayanami::RecoveryCycleData data;
    data.status = ayanami::RecoveryStatus::SUCCESS;
    return ayanami::RecoveryResultT::success(std::move(data));
  }

  std::string configured_name;

 private:
  GridCommandPort::Ptr command_port;
};

// ---- fake 族的派生 server: 与 CostmapNavigationServer 平行的注入层 ----

class GridNavigationServer final : public ayanami::components::NavigationServerBase {
 public:
  bool initializePlannerPlugin(const std::string& name,
                               const ayanami::Config& module_config,
                               ayanami::abstract::AbstractPlanner::Ptr ptr) override {
    auto cast = std::dynamic_pointer_cast<GridPlanner>(ptr);
    if (!cast || !grid_map) {
      return false;
    }
    cast->configure(ayanami::PluginContext{name, module_config}, grid_map);
    return true;
  }

  bool initializeRecoveryPlugin(const std::string& name,
                                const ayanami::Config& module_config,
                                ayanami::abstract::AbstractRecovery::Ptr ptr) override {
    auto cast = std::dynamic_pointer_cast<GridRecovery>(ptr);
    if (!cast || !grid_map) {
      return false;
    }
    cast->configure(ayanami::PluginContext{name, module_config}, grid_map);
    return true;
  }

  std::shared_ptr<const GridMapContext> grid_map;
};

}  // namespace

// ---- 地图族经抽象接口完整表达 identity/frame/版本/能力/写命令/机器人状态 ----

TEST(MapFamilyAbstractionTest, gridContextSatisfiesAbstractInterface) {
  auto map = std::make_shared<GridMapContext>("world", "base");
  auto port = std::make_shared<GridCommandPort>();
  map->setCommandPort(port);

  AbstractMapContext::ConstPtr abstract = map;
  EXPECT_EQ(abstract->mapFamily(), "grid");
  EXPECT_EQ(abstract->globalFrame(), "world");
  EXPECT_EQ(abstract->localFrame(), "world");
  EXPECT_EQ(abstract->robotFrame(), "base");
  EXPECT_TRUE(abstract->isCurrent());
  EXPECT_EQ(abstract->version(), 1u);

  map->setCurrent(false);
  EXPECT_FALSE(abstract->isCurrent());
  map->setCurrent(true);
  map->bumpVersion();
  EXPECT_EQ(abstract->version(), 2u);

  const std::uint32_t caps = abstract->capabilities();
  EXPECT_NE(caps & static_cast<std::uint32_t>(MapCapability::GlobalPlanning), 0u);
  EXPECT_NE(caps & static_cast<std::uint32_t>(MapCapability::ClearRegion), 0u);
  EXPECT_EQ(caps & static_cast<std::uint32_t>(MapCapability::CollisionQuery), 0u);

  ayanami::geometry_msgs::PoseStamped pose;
  ayanami::geometry_msgs::TwistStamped velocity;
  EXPECT_TRUE(abstract->getRobotState(pose, velocity));
  EXPECT_EQ(pose.header.frame_id, "world");
  EXPECT_DOUBLE_EQ(pose.pose.position.x(), 1.0);

  EXPECT_EQ(abstract->commandPort(), port);
}

// ---- 派生 server 钩子把 fake 地图族上下文注入族级插件 (零 costmap 依赖) ----

TEST(MapFamilyAbstractionTest, gridServerInjectsContextViaFamilyHooks) {
  GridNavigationServer server;
  auto port = std::make_shared<GridCommandPort>();
  auto map = std::make_shared<GridMapContext>("world", "base");
  map->setCommandPort(port);
  server.grid_map = map;

  auto planner = std::make_shared<StraightLineGridPlanner>();
  ayanami::abstract::AbstractPlanner::Ptr planner_base = planner;
  EXPECT_TRUE(server.initializePlannerPlugin(
      "grid_planner", ayanami::Config(""), planner_base));
  EXPECT_TRUE(planner->configured);

  // 插件经注入的地图上下文工作 (族层 configure + 抽象层 makePlan)。
  ayanami::geometry_msgs::PoseStamped start;
  ayanami::geometry_msgs::PoseStamped goal;
  start.header.frame_id = "world";
  goal.header.frame_id = "world";
  goal.pose.position.x() = 3.0;
  const auto result = planner_base->makePlan(start, goal, 0.1);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.data.poses.size(), 2u);

  // recovery 插件经 commandPort 提交族内写命令 (写命令接口闭环)。
  auto recovery = std::make_shared<GridClearRecovery>();
  ayanami::abstract::AbstractRecovery::Ptr recovery_base = recovery;
  EXPECT_TRUE(server.initializeRecoveryPlugin(
      "grid_clear", ayanami::Config(""), recovery_base));
  EXPECT_EQ(recovery->configured_name, "grid_clear");

  const auto outcome = recovery_base->tick();
  ASSERT_TRUE(outcome.ok) << outcome.message;
  EXPECT_EQ(port->submissions.load(), 1);
  EXPECT_DOUBLE_EQ(port->last_radius.load(), 0.75);

  // 非 fake 族插件 (普通 AbstractPlanner) 被族钩子拒绝 (类型化约束)。
  class PlainPlanner : public ayanami::abstract::AbstractPlanner {
   public:
    ayanami::PlannerResultT makePlan(const ayanami::geometry_msgs::PoseStamped&,
                                     const ayanami::geometry_msgs::PoseStamped&,
                                     double) override {
      return ayanami::PlannerResultT::failure(ErrorCode::INTERNAL_ERROR, "unused");
    }
  };
  EXPECT_FALSE(server.initializePlannerPlugin(
      "plain", ayanami::Config(""), std::make_shared<PlainPlanner>()));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
