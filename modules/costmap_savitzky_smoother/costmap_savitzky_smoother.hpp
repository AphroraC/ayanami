#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ayanami/costmap/core/costmap_smoother.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 7-point Savitzky-Golay path smoother module (CostmapSmoother family).
 *
 * Coefficients [-2, 3, 6, 7, 6, 3, -2] / 21. The end points are fixed; interior points are convolved over the window
 * (boundaries use replicated clamp padding). Optional refinement rounds perform a second smoothing pass (bounded by the
 * max_duration time budget). The local map is read under a shared lock and the grid is
 * queried point by point; any inscribed-inflated hit (>=253) rolls back to the original path.
 * Holds no context / wrapper / mutable grid.
 */
class CostmapSavitzkySmoother final : public ayanami::costmap::CostmapSmoother {
 public:
  CostmapSavitzkySmoother() = default;
  // The smoother family base class has no virtual destructor; the derived class adds one (without override).
  virtual ~CostmapSavitzkySmoother() = default;

  void configure(const PluginContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  /// @brief In-place smoothing: modifies path directly; restores the original path on failure/rollback.
  ResultT<void> smoothPath(ayanami::nav_msgs::Path& path,
                           double max_duration) override;

  bool cancel() override;

 private:
  struct CostmapSavitzkySmootherParam {
    bool do_refinement = true;
    int refinement_num = 2;
  };

  CostmapSavitzkySmootherParam param;

  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
