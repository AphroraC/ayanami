// Task 22: modules/costmap_savitzky_smoother — 7 点 Savitzky-Golay 路径平滑。
//
// 纯 C++ (零 ROS): 在入参不可变 CostmapSnapshot (global grid) 上做
// 碰撞检查。核心为 7 点 SG 卷积 (nav2 SavitzkyGolaySmoother 语义):
// - 系数 [-2, 3, 6, 7, 6, 3, -2] / 21, 首尾端点固定, 内部点按窗口卷积
// - 边界不足窗口处用重复填充 (clamp 到端点)
// - do_refinement: 对结果再应用 refinement_num 轮过滤 (受 max_duration 限时)
// - check_for_collision: 平滑后逐点查 costmap, 任一 cost >=
//   INSCRIBED_INFLATED_OBSTACLE (Ruling #6, 253 起 lethal) 即回滚 original

#include "costmap_savitzky_smoother.hpp"

#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace ayanami {
namespace modules {

namespace {

// 7 点 SG 滤波器系数 (nav2_smoother §2, 归一化 /21)。
constexpr std::array<double, 7> k_filter = {-2.0 / 21.0, 3.0 / 21.0, 6.0 / 21.0,
                                           7.0 / 21.0,  6.0 / 21.0, 3.0 / 21.0,
                                           -2.0 / 21.0};

// 重复填充窗口索引: 越界 clamp 到 [0, n-1]。
std::size_t clampIndex(std::size_t i, int k, std::size_t n) {
  const long long idx = static_cast<long long>(i) + k;
  return static_cast<std::size_t>(
      std::clamp(idx, 0LL, static_cast<long long>(n - 1)));
}

// 对内部点 (1..n-2) 应用一轮 SG 卷积, 端点固定。读原始值, 一次写入。
void applySmoothPass(std::vector<ayanami::geometry_msgs::PoseStamped>& poses) {
  const std::size_t n = poses.size();
  std::vector<Eigen::Vector3d> smoothed(n);
  smoothed.front() = poses.front().pose.position;
  smoothed.back() = poses.back().pose.position;

  for (std::size_t i = 1; i + 1 < n; ++i) {
    Eigen::Vector3d val = Eigen::Vector3d::Zero();
    for (int k = -3; k <= 3; ++k) {
      const std::size_t j = clampIndex(i, k, n);
      val += k_filter[static_cast<std::size_t>(k + 3)] *
             poses[j].pose.position;
    }
    smoothed[i] = val;
  }

  for (std::size_t i = 1; i + 1 < n; ++i) {
    poses[i].pose.position = smoothed[i];
  }
}

// 内部点朝向对齐到切线方向 (端点保持原始朝向)。
void updateOrientations(std::vector<ayanami::geometry_msgs::PoseStamped>& poses) {
  for (std::size_t i = 1; i + 1 < poses.size(); ++i) {
    const Eigen::Vector3d d = poses[i + 1].pose.position -
                              poses[i - 1].pose.position;
    const double yaw = std::atan2(d.y(), d.x());
    poses[i].pose.orientation =
        Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  }
}

}  // namespace


void CostmapSavitzkySmoother::configure(
    const PluginContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // 参数经 PluginContext 注入的自身配置读取 (插件不知道全局配置布局)。
  const Config& file = context.config;
  param.do_refinement =
      file.param<bool>("costmap_savitzky_smoother", "do_refinement",
                       param.do_refinement);
  param.refinement_num =
      file.param<int>("costmap_savitzky_smoother", "refinement_num",
                      param.refinement_num);
  map_ = std::move(map);
}

bool CostmapSavitzkySmoother::cancel() {
  canceled.store(true);
  return true;
}

SmootherResultT CostmapSavitzkySmoother::smoothPath(
    ayanami::nav_msgs::Path& path, double max_duration,
    bool check_for_collision) {
  // 碰撞检查需要读图: 需要时持 shared 锁 (整个平滑调用持有, <10ms), 零拷贝。
  std::optional<ayanami::costmap::CostmapView> costmap;
  if (check_for_collision) {
    if (!map_) {
      return SmootherResultT::failure(
          ErrorCode::MAP_UNAVAILABLE,
          "CostmapSavitzkySmoother has no costmap module");
    }
    costmap = map_->lockGlobal();
  }
  const ayanami::nav_msgs::Path original = path;
  canceled.store(false);

  // 空 path 或点数 < 7: 无窗口可平滑, 直接 SUCCESS, path 不变。
  if (path.poses.size() < 7) {
    auto result = SmootherResultT::success(path);
    result.message = "path too short to smooth";
    return result;
  }

  const auto t0 = std::chrono::steady_clock::now();
  const auto elapsed = [&t0]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
        .count();
  };
  // max_duration < 0 视为不限时。
  const bool timed = max_duration >= 0.0;

  // 预算耗尽仍未开始: SUCCESS + 部分平滑 (= 无)。
  if (timed && elapsed() > max_duration) {
    return SmootherResultT::success(path);
  }

  // 名义一轮过滤 (核心平滑)。
  applySmoothPass(path.poses);

  // 精化轮 (时间预算 / cancel 约束下提前停止, 已平滑部分保留)。
  if (param.do_refinement) {
    for (int r = 0; r < param.refinement_num; ++r) {
      if (canceled.load() || (timed && elapsed() >= max_duration)) {
        break;
      }
      applySmoothPass(path.poses);
    }
  }

  updateOrientations(path.poses);

  // 碰撞检查: 平滑后逐点查加锁读视图, 命中 lethal 回滚到 original。
  if (check_for_collision) {
    const costmap_2d::Costmap2D* cm = costmap ? costmap->grid() : nullptr;
    if (cm) {
      for (const auto& p : path.poses) {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!cm->worldToMap(p.pose.position.x(), p.pose.position.y(),
                            mx, my)) {
          continue;
        }
        if (cm->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          path = original;
          auto result = SmootherResultT::success(path);
          result.message = "collision detected; rolled back to original path";
          return result;
        }
      }
    }
  }

  // 与 original 完全一致时同样退化为 SUCCESS。
  return SmootherResultT::success(path);
}

}  // namespace modules
}  // namespace ayanami
