// Map Family abstraction test: proves the NavigationServerBase assembly path has zero costmap
// dependency -- a non-costmap fake map family (the Grid family) completes module configuration through the same set of virtual hooks
// (initialize*Module + ModuleContext injection), with the map context provided via the
// AbstractMapContext interface: identity/frame/version/capability/write commands/robot state.
//
// This test dlopens no .so at all (the assembly seam is at the hook layer, orthogonal to the loading mechanism).

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
using ayanami::map::MapCommandPort;

// ---- Fake "grid" map-family context (a minimal implementation parallel to the costmap family) ----

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
  GridMapContext() = default;

  std::string mapFamily() const override { return "grid"; }
  MapCommandPort::Ptr commandPort() const override { return command_port; }
  void setCommandPort(GridCommandPort::Ptr port) { command_port = std::move(port); }

 private:
  GridCommandPort::Ptr command_port;
};

// ---- Fake-family family-level module layer (the abstract layer has zero map knowledge; the family layer binds GridMapContext) ----

class GridPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  virtual void configure(const ayanami::ModuleContext& context,
                         std::shared_ptr<const GridMapContext> map) = 0;

 protected:
  std::shared_ptr<const GridMapContext> map_context;
  std::string configured_name;
};

class StraightLineGridPlanner final : public GridPlanner {
 public:
  void configure(const ayanami::ModuleContext& context,
                 std::shared_ptr<const GridMapContext> map) override {
    configured_name = context.name;
    map_context = std::move(map);
    configured = true;
  }

  ayanami::ResultT<void> makePlan(
      const ayanami::geometry_msgs::PoseStamped& start,
      const ayanami::geometry_msgs::PoseStamped& goal,
      double /*tolerance*/, ayanami::nav_msgs::Path& plan_out) override {
    if (!configured) {
      return ayanami::ResultT<void>::failure(
          ErrorCode::INTERNAL_ERROR, "grid planner not configured");
    }
    ayanami::nav_msgs::Path path;
    path.header = goal.header;
    path.poses.push_back(start);
    path.poses.push_back(goal);
    plan_out = std::move(path);
    return ayanami::ResultT<void>::success();
  }

  bool configured = false;
};

class GridRecovery : public ayanami::abstract::AbstractRecovery {
 public:
  virtual void configure(const ayanami::ModuleContext& context,
                         std::shared_ptr<const GridMapContext> map) = 0;

 protected:
  std::shared_ptr<const GridMapContext> map_context;
};

class GridClearRecovery final : public GridRecovery {
 public:
  void configure(const ayanami::ModuleContext& context,
                 std::shared_ptr<const GridMapContext> map) override {
    configured_name = context.name;
    map_context = std::move(map);
    command_port = std::dynamic_pointer_cast<GridCommandPort>(map_context->commandPort());
  }

  ayanami::ResultT<void> tick(ayanami::RecoveryCycleData& cycle) override {
    if (!command_port) {
      return ayanami::ResultT<void>::failure(
          ErrorCode::INTERNAL_ERROR, "grid recovery has no command port");
    }
    GridWriteCommand command;
    command.clear_radius = 0.75;
    if (!command_port->submit(command)) {
      return ayanami::ResultT<void>::failure(
          ErrorCode::RECOVERY_FAILED, "grid write command rejected");
    }
    ayanami::RecoveryCycleData data;
    data.status = ayanami::RecoveryStatus::SUCCESS;
    cycle = std::move(data);
    return ayanami::ResultT<void>::success();
  }

  std::string configured_name;

 private:
  GridCommandPort::Ptr command_port;
};

// ---- Fake-family derived server: an injection layer parallel to CostmapNavigationServer ----

class GridNavigationServer final : public ayanami::components::NavigationServerBase {
 public:
  bool initializePlannerModule(const std::string& name,
                               const ayanami::Config& module_config,
                               ayanami::abstract::AbstractPlanner::Ptr ptr) override {
    auto cast = std::dynamic_pointer_cast<GridPlanner>(ptr);
    if (!cast || !grid_map) {
      return false;
    }
    cast->configure(ayanami::ModuleContext{name, module_config}, grid_map);
    return true;
  }

  bool initializeRecoveryModule(const std::string& name,
                                const ayanami::Config& module_config,
                                ayanami::abstract::AbstractRecovery::Ptr ptr) override {
    auto cast = std::dynamic_pointer_cast<GridRecovery>(ptr);
    if (!cast || !grid_map) {
      return false;
    }
    cast->configure(ayanami::ModuleContext{name, module_config}, grid_map);
    return true;
  }

  std::shared_ptr<const GridMapContext> grid_map;
};

}  // namespace

// ---- The map family is fully expressed through the abstract interface: identity/frame/version/capability/write commands/robot state ----

TEST(MapFamilyAbstractionTest, gridContextSatisfiesAbstractInterface) {
  auto map = std::make_shared<GridMapContext>();
  auto port = std::make_shared<GridCommandPort>();
  map->setCommandPort(port);

  AbstractMapContext::ConstPtr abstract = map;
  EXPECT_EQ(abstract->mapFamily(), "grid");
  EXPECT_EQ(abstract->commandPort(), port);
}

// ---- The derived server hooks inject the fake map-family context into family-level modules (zero costmap dependency) ----

TEST(MapFamilyAbstractionTest, gridServerInjectsContextViaFamilyHooks) {
  GridNavigationServer server;
  auto port = std::make_shared<GridCommandPort>();
  auto map = std::make_shared<GridMapContext>();
  map->setCommandPort(port);
  server.grid_map = map;

  auto planner = std::make_shared<StraightLineGridPlanner>();
  ayanami::abstract::AbstractPlanner::Ptr planner_base = planner;
  EXPECT_TRUE(server.initializePlannerModule(
      "grid_planner", ayanami::Config(""), planner_base));
  EXPECT_TRUE(planner->configured);

  // The modules work through the injected map context (family-level configure + abstract-layer makePlan).
  ayanami::geometry_msgs::PoseStamped start;
  ayanami::geometry_msgs::PoseStamped goal;
  start.header.frame_id = "world";
  goal.header.frame_id = "world";
  goal.pose.position.x() = 3.0;
  ayanami::nav_msgs::Path plan;
  const auto result = planner_base->makePlan(start, goal, 0.1, plan);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(plan.poses.size(), 2u);

  // The recovery module submits family write commands via commandPort (the write-command interface closes the loop).
  auto recovery = std::make_shared<GridClearRecovery>();
  ayanami::abstract::AbstractRecovery::Ptr recovery_base = recovery;
  EXPECT_TRUE(server.initializeRecoveryModule(
      "grid_clear", ayanami::Config(""), recovery_base));
  EXPECT_EQ(recovery->configured_name, "grid_clear");

  ayanami::RecoveryCycleData cycle;
  const auto outcome = recovery_base->tick(cycle);
  ASSERT_TRUE(outcome.ok) << outcome.message;
  EXPECT_EQ(port->submissions.load(), 1);
  EXPECT_DOUBLE_EQ(port->last_radius.load(), 0.75);

  // A non-fake-family module (a plain AbstractPlanner) is rejected by the family hooks (typed constraint).
  class PlainPlanner : public ayanami::abstract::AbstractPlanner {
   public:
    ayanami::ResultT<void> makePlan(const ayanami::geometry_msgs::PoseStamped&,
                                     const ayanami::geometry_msgs::PoseStamped&,
                                     double, ayanami::nav_msgs::Path& plan_out) override {
      return ayanami::ResultT<void>::failure(ErrorCode::INTERNAL_ERROR, "unused");
    }
  };
  EXPECT_FALSE(server.initializePlannerModule(
      "plain", ayanami::Config(""), std::make_shared<PlainPlanner>()));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
