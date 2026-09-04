#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/core/costmap_smoother.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 7 点 Savitzky-Golay 路径平滑模块 (CostmapSmoother 族)。
 *
 * 系数 [-2, 3, 6, 7, 6, 3, -2] / 21。首尾端点固定, 内部点按窗口卷积
 * (边界用重复填充 clamp)。可选 refinement 轮次做二次平滑 (受 max_duration
 * 时间预算约束)。check_for_collision 时对地图模块加锁直读 (shared 锁) 逐点查询
 * global grid, 命中 inscribed-inflated (>=253, Ruling #6) 即回滚到 original
 * path。不持有 context / wrapper / 可变 grid。
 */
class CostmapSavitzkySmoother final : public ayanami::costmap::CostmapSmoother {
 public:
  CostmapSavitzkySmoother() = default;
  // Ruling #3: smoother 族基类无虚析构, 派生类补虚拟析构 (不加 override)。
  virtual ~CostmapSavitzkySmoother() = default;

  void configure(const PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  /// @brief in-place 平滑: 直接修改 path; 失败/回滚时恢复原路径。
  SmootherResultT smoothPath(ayanami::nav_msgs::Path& path,
                             double max_duration,
                             bool check_for_collision) override;

  bool cancel() override;

 private:
  struct CostmapSavitzkySmootherParam {
    bool do_refinement = true;
    int refinement_num = 2;
  };

  CostmapSavitzkySmootherParam param;

  ayanami::costmap::CostmapMapContext::ConstPtr map_;

  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
