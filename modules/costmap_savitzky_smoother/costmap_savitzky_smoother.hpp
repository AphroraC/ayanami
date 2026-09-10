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

  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load, and by configure() for callers that build
  /// the object directly.
  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  /// @brief In-place smoothing: modifies path directly; restores the original path on failure/rollback.
  ResultT<void> smoothPath(ayanami::nav_msgs::Path& path,
                           double max_duration) override;

 private:
  /// @brief smoothPath() body with the local map already read-locked.
  ResultT<void> smoothPathWithMap(ayanami::nav_msgs::Path& path,
                                  double max_duration,
                                  const ayanami::costmap::CostmapReader& map);

 public:

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
