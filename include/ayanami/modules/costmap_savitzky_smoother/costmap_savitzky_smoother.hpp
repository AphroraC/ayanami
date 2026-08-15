#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/components/costmap_context.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 7 点 Savitzky-Golay 路径平滑模块 (spec §5.5 CostmapSmoother 族,
 * 参考 nav2_smoother SavitzkyGolaySmoother 纯 C++ 重写)。
 *
 * 系数 [-2, 3, 6, 7, 6, 3, -2] / 21。首尾端点固定, 内部点按窗口卷积
 * (边界用重复填充 clamp)。可选 refinement 轮次做二次平滑 (受 max_duration
 * 时间预算约束)。check_for_collision 时逐点查询 global costmap, 命中
 * inscribed-inflated (>=253, Ruling #6) 即回滚到 original path。
 */
class CostmapSavitzkySmoother : public ayanami::costmap::CostmapSmoother {
 public:
  CostmapSavitzkySmoother();
  // Ruling #3: smoother 族基类无虚析构, 派生类补虚拟析构 (不加 override)。
  virtual ~CostmapSavitzkySmoother() = default;

  void initialize(const std::string& name,
                  ayanami::costmap::CostmapContext::Ptr ctx) override;

  uint32_t smoothPath(const ayanami::nav_msgs::Path& in,
                      ayanami::nav_msgs::Path& out, double max_duration,
                      bool check_for_collision, std::string& message) override;

  bool cancel() override;

 private:
  struct Config {
    bool do_refinement = true;
    int refinement_num = 2;
  };

  void loadConfig();

  ayanami::costmap::CostmapContext::Ptr costmap_context_;
  std::atomic<bool> canceled_{false};
  Config config_;
};

}  // namespace modules
}  // namespace ayanami
