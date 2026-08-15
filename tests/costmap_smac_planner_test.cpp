// Task 21: modules/costmap_smac_planner (CostmapPlanner 族 A* 全局路径规划)。
//
// 覆盖:
// - makePlan 全 free 地图 start/goal → SUCCESS, path 非空, cost>0, 起点≈start, 终点≈goal
// - goal 在 obstacle → nearestValidGoal → SUCCESS (tolerance 覆盖) / NO_PATH_FOUND=58 (无 tolerance)
// - start 在 obstacle / 越界 → outcome >= 50 (规划段)
// - cancel() 中途打断 → outcome=68 (CANCELED, 规划段)
// - module.cpp 导出 create_costmap_planner_module 符号 (dlopen 验证)

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/static_layer.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/modules/costmap_smac_planner/costmap_smac_planner.hpp>
#include <ayanami/utility/load_module.hpp>

namespace {

using ayanami::costmap::CostmapContext;
using ayanami::modules::CostmapSmacPlanner;

std::string g_module_so;

// ---------------- 测试工具 ----------------

// 建一个 NxN @ res, origin=(0,0) 的 LayeredCostmap + StaticLayer 小地图。
// obstacle_fn 回调可标记障碍 (默认全 free)。
std::shared_ptr<costmap_2d::CostmapWrapper> makeWrapper(
    unsigned int size, double res,
    const std::function<void(costmap_2d::Costmap2D&)>& obstacle_fn = {}) {
  auto lm = std::make_shared<costmap_2d::LayeredCostmap>(
      "map", /*rolling_window=*/false, /*track_unknown=*/false);
  lm->resizeMap(size, size, res, 0.0, 0.0);

  costmap_2d::Costmap2D layer_map(size, size, res, 0.0, 0.0,
                                  costmap_2d::FREE_SPACE);
  if (obstacle_fn) {
    obstacle_fn(layer_map);
  }
  auto static_layer = std::make_shared<costmap_2d_layers::StaticLayer>();
  static_layer->setMap(layer_map);
  lm->addLayer(static_layer);
  lm->updateMap(0.0, 0.0, 0.0);
  return std::make_shared<costmap_2d::CostmapWrapper>(lm, "map");
}

std::shared_ptr<CostmapContext> makeContext(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper) {
  auto ctx = std::make_shared<CostmapContext>();
  ctx->global_costmap_ = wrapper;
  return ctx;
}

std::shared_ptr<CostmapSmacPlanner> makePlanner(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper) {
  auto planner = std::make_shared<CostmapSmacPlanner>();
  planner->initialize("test_smac", makeContext(wrapper));
  return planner;
}

ayanami::geometry_msgs::PoseStamped makePose(double x, double y) {
  auto p = ayanami::geometry_msgs::PoseStamped::fromIsometry(
      "map", Eigen::Isometry3d::Identity());
  p.pose.translation() = Eigen::Vector3d(x, y, 0.0);
  return p;
}

// cell (mx,my) 中心的世界坐标 (res=0.5, origin=0)。
ayanami::geometry_msgs::PoseStamped cellCenter(unsigned int mx, unsigned int my,
                                               double res = 0.5) {
  return makePose((static_cast<double>(mx) + 0.5) * res,
                  (static_cast<double>(my) + 0.5) * res);
}

}  // namespace

// ---------- makePlan 成功路径 ----------

TEST(CostmapSmacPlannerTest, MakePlanSuccessOnFreeMap) {
  auto wrapper = makeWrapper(20, 0.5);  // 20x20 @ 0.5m = 10m x 10m, 全 free
  auto planner = makePlanner(wrapper);

  auto start = cellCenter(2, 2);
  auto goal = cellCenter(17, 17);
  ayanami::nav_msgs::Path plan;
  double cost = 0.0;
  std::string message;
  const uint32_t outcome = planner->makePlan(start, goal, 0.1, plan, cost, message);

  EXPECT_EQ(outcome, ayanami::actions::GetPathResult::SUCCESS) << message;
  ASSERT_FALSE(plan.poses.empty());
  EXPECT_GT(cost, 0.0);
  EXPECT_NEAR(plan.poses.front().pose.translation().x(), start.pose.translation().x(), 1e-6);
  EXPECT_NEAR(plan.poses.front().pose.translation().y(), start.pose.translation().y(), 1e-6);
  EXPECT_NEAR(plan.poses.back().pose.translation().x(), goal.pose.translation().x(), 1e-6);
  EXPECT_NEAR(plan.poses.back().pose.translation().y(), goal.pose.translation().y(), 1e-6);
}

TEST(CostmapSmacPlannerTest, MakePlanRoutesAroundObstacle) {
  // 中央 4x4 障碍块 (cells 8..11), start/goal 在两侧。
  auto wrapper = makeWrapper(20, 0.5, [](costmap_2d::Costmap2D& m) {
    for (unsigned int y = 8; y <= 11; ++y) {
      for (unsigned int x = 8; x <= 11; ++x) {
        m.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
      }
    }
  });
  auto planner = makePlanner(wrapper);

  auto start = cellCenter(2, 2);
  auto goal = cellCenter(17, 17);
  ayanami::nav_msgs::Path plan;
  double cost = 0.0;
  std::string message;
  const uint32_t outcome = planner->makePlan(start, goal, 0.1, plan, cost, message);

  EXPECT_EQ(outcome, ayanami::actions::GetPathResult::SUCCESS) << message;
  ASSERT_FALSE(plan.poses.empty());
  EXPECT_GT(cost, 0.0);
}

// ---------- goal 在 obstacle (nearestValidGoal) ----------

TEST(CostmapSmacPlannerTest, GoalInObstacleWithToleranceFindsNearest) {
  // 中央 4x4 障碍块 (cells 8..11), goal 在块中心 (10,10)。
  auto wrapper = makeWrapper(20, 0.5, [](costmap_2d::Costmap2D& m) {
    for (unsigned int y = 8; y <= 11; ++y) {
      for (unsigned int x = 8; x <= 11; ++x) {
        m.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
      }
    }
  });
  auto planner = makePlanner(wrapper);

  auto start = cellCenter(2, 2);
  auto goal = cellCenter(10, 10);
  ayanami::nav_msgs::Path plan;
  double cost = 0.0;
  std::string message;
  // tolerance=1.0m → 2 cells, 覆盖块边缘附近 free cell (8,7) 等。
  const uint32_t outcome = planner->makePlan(start, goal, 1.0, plan, cost, message);

  EXPECT_EQ(outcome, ayanami::actions::GetPathResult::SUCCESS) << message;
  ASSERT_FALSE(plan.poses.empty());
  EXPECT_GT(cost, 0.0);
}

TEST(CostmapSmacPlannerTest, GoalInObstacleWithoutToleranceReturnsNoPath) {
  auto wrapper = makeWrapper(20, 0.5, [](costmap_2d::Costmap2D& m) {
    for (unsigned int y = 8; y <= 11; ++y) {
      for (unsigned int x = 8; x <= 11; ++x) {
        m.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
      }
    }
  });
  auto planner = makePlanner(wrapper);

  auto start = cellCenter(2, 2);
  auto goal = cellCenter(10, 10);
  ayanami::nav_msgs::Path plan;
  double cost = 0.0;
  std::string message;
  const uint32_t outcome = planner->makePlan(start, goal, 0.0, plan, cost, message);

  EXPECT_EQ(outcome, ayanami::actions::GetPathResult::NO_PATH_FOUND);
}

// ---------- start 异常 ----------

TEST(CostmapSmacPlannerTest, StartInObstacleReturnsErrorSegment) {
  auto wrapper = makeWrapper(20, 0.5, [](costmap_2d::Costmap2D& m) {
    for (unsigned int y = 8; y <= 11; ++y) {
      for (unsigned int x = 8; x <= 11; ++x) {
        m.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
      }
    }
  });
  auto planner = makePlanner(wrapper);

  auto start = cellCenter(10, 10);  // 障碍内
  auto goal = cellCenter(2, 2);
  ayanami::nav_msgs::Path plan;
  double cost = 0.0;
  std::string message;
  const uint32_t outcome = planner->makePlan(start, goal, 0.1, plan, cost, message);

  EXPECT_GE(outcome, 50u);  // INVALID_START/OUT_OF_MAP 等规划段
  EXPECT_TRUE(plan.poses.empty());
}

TEST(CostmapSmacPlannerTest, StartOutOfMapReturnsErrorSegment) {
  auto wrapper = makeWrapper(20, 0.5);
  auto planner = makePlanner(wrapper);

  auto start = makePose(-1.0, -1.0);  // 越界 (origin 0)
  auto goal = cellCenter(5, 5);
  ayanami::nav_msgs::Path plan;
  double cost = 0.0;
  std::string message;
  const uint32_t outcome = planner->makePlan(start, goal, 0.1, plan, cost, message);

  EXPECT_GE(outcome, 50u);
  EXPECT_TRUE(plan.poses.empty());
}

// ---------- cancel() 中途打断 ----------

TEST(CostmapSmacPlannerTest, CancelDuringPlanningReturnsCanceled) {
  // 800x800 @ 0.05 = 40m x 40m。goal 位于 lethal 环内 (ring 1 cell 厚):
  // A* 必须扩展整个可达区域才能判 NO_PATH (约 100ms), 给 cancel 留足窗口。
  auto wrapper = makeWrapper(800, 0.05, [](costmap_2d::Costmap2D& m) {
    const unsigned int c = 400;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx != 0 || dy != 0) {
          m.setCost(c + static_cast<unsigned int>(dx),
                    c + static_cast<unsigned int>(dy),
                    costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  });
  auto planner = makePlanner(wrapper);

  auto start = cellCenter(5, 5, 0.05);
  auto goal = cellCenter(400, 400, 0.05);
  ayanami::nav_msgs::Path plan;
  double cost = 0.0;
  std::string message;

  std::atomic<bool> done{false};
  std::atomic<uint32_t> outcome{0};
  std::thread worker([&] {
    outcome.store(planner->makePlan(start, goal, 0.0, plan, cost, message));
    done.store(true);
  });

  // 让搜索跑一小段时间再 cancel
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_TRUE(planner->cancel());
  worker.join();
  EXPECT_TRUE(done.load());
  EXPECT_EQ(outcome.load(), 68u);  // CANCELED (规划段 50+)
}

// ---------- module.cpp 工厂符号 ----------

TEST(CostmapSmacPlannerTest, ModuleFactoryCreatesPlanner) {
  ASSERT_FALSE(g_module_so.empty());
  void* handle = ayanami::open_so(g_module_so);
  ASSERT_NE(handle, nullptr);
  using Factory = ayanami::costmap::CostmapPlanner* (*)();
  void* symbol =
      ayanami::load_symbol(handle, "create_costmap_planner_module");
  ASSERT_NE(symbol, nullptr);
  auto factory = reinterpret_cast<Factory>(symbol);
  ayanami::costmap::CostmapPlanner* planner = factory();
  ASSERT_NE(planner, nullptr);
  delete planner;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_module_so = argv[1];
  }
  return RUN_ALL_TESTS();
}
