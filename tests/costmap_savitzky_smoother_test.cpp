// Task 22: modules/costmap_savitzky_smoother (CostmapSmoother 族 7 点
// Savitzky-Golay 平滑)。
//
// 覆盖:
// - L 形 zigzag input (10 poses) → smoothPath 后更平滑 (一阶差分长度和减小),
//   path size 不减小, 两端点固定不动
// - max_duration=0.001s 立即 SUCCESS (允许放过部分或全部)
// - 空 path / 点数 < 7 → SUCCESS, out=in
// - check_for_collision=true: 平滑后路径经过障碍 cell → 回滚到 original path,
//   msg 含 "collision"
// - module.cpp 导出 create_costmap_smoother_module 符号 (dlopen 验证)

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <costmap_2d/costmap_2d.hpp>
#include <costmap_2d/layered_costmap.hpp>
#include <costmap_2d_layers/static_layer.hpp>
#include <costmap_server/costmap_wrapper.hpp>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/messages/actions.hpp>
#include <ayanami/modules/costmap_savitzky_smoother/costmap_savitzky_smoother.hpp>
#include <ayanami/utility/load_module.hpp>

namespace {

using ayanami::costmap::CostmapContext;
using ayanami::modules::CostmapSavitzkySmoother;
using ayanami::nav_msgs::Path;

std::string g_module_so;

// ---------------- 测试工具 ----------------

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

std::shared_ptr<CostmapSavitzkySmoother> makeSmoother(
    const std::shared_ptr<costmap_2d::CostmapWrapper>& wrapper) {
  auto smoother = std::make_shared<CostmapSavitzkySmoother>();
  smoother->initialize("test_savitzky", makeContext(wrapper));
  return smoother;
}

ayanami::geometry_msgs::PoseStamped makePose(double x, double y) {
  auto p = ayanami::geometry_msgs::PoseStamped::fromIsometry(
      "map", Eigen::Isometry3d::Identity());
  p.pose.translation() = Eigen::Vector3d(x, y, 0.0);
  return p;
}

// L 形 zigzag path (10 poses): 5 沿 +x 再 5 沿 +y。
Path makeLPath() {
  Path p;
  p.header.frame_id = "map";
  for (int i = 0; i <= 4; ++i) {
    p.poses.push_back(makePose(static_cast<double>(i), 0.0));
  }
  for (int j = 1; j <= 5; ++j) {
    p.poses.push_back(makePose(5.0, static_cast<double>(j)));
  }
  return p;
}

// 一阶差分 (欧氏步长) 绝对值之和: 越小越平滑。
double pathLengthSum(const Path& p) {
  double sum = 0.0;
  for (std::size_t i = 1; i < p.poses.size(); ++i) {
    sum += (p.poses[i].pose.translation() - p.poses[i - 1].pose.translation())
               .norm();
  }
  return sum;
}

bool posesEqual(const Path& a, const Path& b) {
  if (a.poses.size() != b.poses.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.poses.size(); ++i) {
    if ((a.poses[i].pose.translation() - b.poses[i].pose.translation()).norm() >
        1e-9) {
      return false;
    }
  }
  return true;
}

}  // namespace

// ---------- 平滑有效性 ----------

TEST(CostmapSavitzkySmootherTest, SmoothsLShapedPath) {
  auto wrapper = makeWrapper(20, 0.5);
  auto smoother = makeSmoother(wrapper);

  const Path in = makeLPath();
  Path out;
  std::string message;
  const uint32_t outcome =
      smoother->smoothPath(in, out, 10.0, false, message);

  EXPECT_EQ(outcome, ayanami::actions::SmoothPathResult::SUCCESS) << message;
  EXPECT_EQ(out.poses.size(), in.poses.size());
  EXPECT_LT(pathLengthSum(out), pathLengthSum(in));
  // 两端点固定不动。
  EXPECT_NEAR(out.poses.front().pose.translation().x(),
              in.poses.front().pose.translation().x(), 1e-9);
  EXPECT_NEAR(out.poses.front().pose.translation().y(),
              in.poses.front().pose.translation().y(), 1e-9);
  EXPECT_NEAR(out.poses.back().pose.translation().x(),
              in.poses.back().pose.translation().x(), 1e-9);
  EXPECT_NEAR(out.poses.back().pose.translation().y(),
              in.poses.back().pose.translation().y(), 1e-9);
}

TEST(CostmapSavitzkySmootherTest, TinyMaxDurationReturnsSuccess) {
  auto wrapper = makeWrapper(20, 0.5);
  auto smoother = makeSmoother(wrapper);

  const Path in = makeLPath();
  Path out;
  std::string message;
  const uint32_t outcome =
      smoother->smoothPath(in, out, 0.001, false, message);

  EXPECT_EQ(outcome, ayanami::actions::SmoothPathResult::SUCCESS) << message;
  EXPECT_GE(out.poses.size(), in.poses.size());
  EXPECT_FALSE(out.poses.empty());
}

// ---------- 空 / 短 path ----------

TEST(CostmapSavitzkySmootherTest, EmptyPathReturnsSuccessWithEmptyOut) {
  auto wrapper = makeWrapper(20, 0.5);
  auto smoother = makeSmoother(wrapper);

  Path in;
  Path out;
  out.poses.push_back(makePose(1.0, 1.0));  // 先污染, 验证被覆盖
  std::string message;
  const uint32_t outcome =
      smoother->smoothPath(in, out, 10.0, false, message);

  EXPECT_EQ(outcome, ayanami::actions::SmoothPathResult::SUCCESS) << message;
  EXPECT_TRUE(out.poses.empty());
}

TEST(CostmapSavitzkySmootherTest, ShortPathReturnsSuccessWithSamePath) {
  auto wrapper = makeWrapper(20, 0.5);
  auto smoother = makeSmoother(wrapper);

  Path in;  // 3 < 7 个点
  for (int i = 0; i < 3; ++i) {
    in.poses.push_back(makePose(static_cast<double>(i), 0.0));
  }
  Path out;
  std::string message;
  const uint32_t outcome =
      smoother->smoothPath(in, out, 10.0, false, message);

  EXPECT_EQ(outcome, ayanami::actions::SmoothPathResult::SUCCESS) << message;
  EXPECT_TRUE(posesEqual(out, in));
}

// ---------- 碰撞检查回滚 ----------

TEST(CostmapSavitzkySmootherTest, CollisionCheckRollsBackToOriginal) {
  auto wrapper = makeWrapper(20, 0.5);
  auto smoother = makeSmoother(wrapper);
  costmap_2d::Costmap2D* cm = wrapper->get().getCostmap();
  ASSERT_NE(cm, nullptr);

  const Path in = makeLPath();
  Path smoothed;
  std::string message;
  ASSERT_EQ(smoother->smoothPath(in, smoothed, 10.0, false, message),
            ayanami::actions::SmoothPathResult::SUCCESS);
  ASSERT_FALSE(posesEqual(smoothed, in));  // 平滑确实移动了内部点

  // 找平滑后经过、但 original path 未经过的自由 cell, 置为 lethal。
  std::set<std::pair<unsigned int, unsigned int>> orig_cells;
  for (const auto& p : in.poses) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (cm->worldToMap(p.pose.translation().x(), p.pose.translation().y(), mx,
                       my)) {
      orig_cells.insert({mx, my});
    }
  }

  bool found = false;
  for (std::size_t i = 1; i + 1 < smoothed.poses.size(); ++i) {
    unsigned int mx = 0;
    unsigned int my = 0;
    const Eigen::Vector3d& t = smoothed.poses[i].pose.translation();
    if (!cm->worldToMap(t.x(), t.y(), mx, my)) {
      continue;
    }
    const auto cell = std::make_pair(mx, my);
    if (orig_cells.count(cell) == 0 &&
        cm->getCost(mx, my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      cm->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
      found = true;
      break;
    }
  }
  ASSERT_TRUE(found) << "expected a smoothed corner-cut cell to place obstacle";

  // 重平滑 + 碰撞检查 → 回滚到 original path。
  Path out;
  message.clear();
  const uint32_t outcome =
      smoother->smoothPath(in, out, 10.0, true, message);

  EXPECT_EQ(outcome, ayanami::actions::SmoothPathResult::SUCCESS) << message;
  EXPECT_TRUE(posesEqual(out, in));  // 回滚到 original
  EXPECT_NE(message.find("collision"), std::string::npos);
}

// ---------- module.cpp 工厂符号 ----------

TEST(CostmapSavitzkySmootherTest, ModuleFactoryCreatesSmoother) {
  ASSERT_FALSE(g_module_so.empty());
  void* handle = ayanami::open_so(g_module_so);
  ASSERT_NE(handle, nullptr);
  using Factory = ayanami::costmap::CostmapSmoother* (*)();
  void* symbol = ayanami::load_symbol(handle, "create_costmap_smoother_module");
  ASSERT_NE(symbol, nullptr);
  auto factory = reinterpret_cast<Factory>(symbol);
  ayanami::costmap::CostmapSmoother* smoother = factory();
  ASSERT_NE(smoother, nullptr);
  delete smoother;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_module_so = argv[1];
  }
  return RUN_ALL_TESTS();
}
