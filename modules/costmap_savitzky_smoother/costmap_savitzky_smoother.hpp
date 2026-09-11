#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/messages/nav_msgs.hpp>
#include <ayanami/utility/callback_slot.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 7-point Savitzky-Golay path smoother module (CostmapSmoother family).
 *
 * Coefficients [-2, 3, 6, 7, 6, 3, -2] / 21. The end points are fixed; interior points are convolved over the window
 * (boundaries use replicated clamp padding). Optional refinement rounds perform a second smoothing pass (bounded by the
 * max_duration time budget).
 *
 * Map access discipline (family read contract: smoother = global+local
 * collision checking, WITHOUT touching the live maps): configure()
 * subscribes to NavigationCallbacks::on_costmap_raw and keeps its own
 * lossless full-grid copies (the host emits them under the map's shared
 * lock, only when the content changed). smoothPath() runs entirely on
 * those immutable copies -- no map lock is ever held by the smoother, so a
 * long smoothing pass cannot delay the map writer.
 *
 * Re-work gating: identical input path + unchanged map revisions return
 * the cached output (memoization); a changed path or any map revision
 * advance re-smooths.
 *
 * Collision check: after smoothing, the path is queried point by point on
 * both copies; any inscribed-inflated hit (>=253) rolls back to the
 * original path and still reports SUCCESS -- the rollback is a graceful
 * degradation, not a failure. A missing snapshot degrades the same way
 * (the BT-level IsPathValid still covers the path).
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

 public:

  bool cancel() override;

 private:
  struct CostmapSavitzkySmootherParam {
    bool do_refinement = true;
    int refinement_num = 2;
  };

  /// @brief One raw-slot snapshot (the smoother's private map copy).
  struct RawSnapshot {
    ayanami::nav_msgs::RawCostmap data;
    bool valid{false};
  };

  /// @brief Slot callback: route by map name and store a copy (fast; the
  /// publisher already holds the map's shared lock while emitting).
  void onRawCostmap(const ayanami::nav_msgs::RawCostmap& raw);

  /// @brief Mutex-scoped copy of one snapshot for the smoothing call.
  RawSnapshot snapshot(const std::string& name) const;

  /// @brief Collision check of `path` against one snapshot copy. The path
  /// frame and the copy's frame may differ: the grid<-path transform is
  /// resolved ONCE per call. Returns true on the first inscribed-inflated
  /// hit.
  bool collidesWithSnapshot(const ayanami::nav_msgs::Path& path,
                            const RawSnapshot& snapshot) const;

  /// @brief One SG pass sequence (core smoothing + refinement) under the
  /// time budget / cancel constraint. No map access.
  void smoothInPlace(ayanami::nav_msgs::Path& path, double max_duration);

  CostmapSavitzkySmootherParam param;

  /// @brief Kept for frame metadata only (global/local frame names); map
  /// DATA arrives through the raw slot.
  ayanami::costmap::CostmapMapContext::ConstPtr map_context;

  mutable std::mutex snapshots_mutex;
  RawSnapshot global_snapshot;
  RawSnapshot local_snapshot;
  CallbackSubscription global_subscription;
  CallbackSubscription local_subscription;
  bool subscriptions_active{false};

  // Memoization: last smoothed input / output + the map revisions it was
  // computed against.
  bool has_cache{false};
  ayanami::nav_msgs::Path cached_input;
  ayanami::nav_msgs::Path cached_output;
  static constexpr uint64_t k_no_map = UINT64_MAX;
  uint64_t cached_global_revision{k_no_map};
  uint64_t cached_local_revision{k_no_map};

  std::atomic<bool> canceled{false};
};

}  // namespace modules
}  // namespace ayanami
