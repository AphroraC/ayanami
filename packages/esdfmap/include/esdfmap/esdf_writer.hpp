#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "esdfmap/esdf_map.hpp"

namespace esdfmap {

/// @brief One obstacle-mask read result handed to the writer by its
/// (injected) source reader. `cells` points at storage owned by the source
/// for the duration of the read call only (the writer copies what it needs
/// into its fused raster before returning).
struct EsdfMaskView {
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  std::string frame;
  /// @brief Monotonic source generation (equal generation = byte-identical
  /// content; the writer gates rebuilds on it).
  std::uint64_t generation{0};
  /// @brief 1 = obstacle, 0 = free (row-major, width * height entries).
  const std::vector<unsigned char>* cells{nullptr};
};

/// @brief Reads the BASE mask: it defines the field's geometry AND frame.
/// Rose-style map-frame deployment: the global static map (map frame,
/// fixed window). Fallback deployments without a static map: the rolling
/// local obstacle mask (odom frame). Returns false when unavailable.
using MaskReader = std::function<bool(EsdfMaskView&)>;
/// @brief Reads the OVERLAY mask (optional; e.g. the rolling local live
/// marks). Projected into the base raster through the injected transform
/// (overlay.frame <- base.frame), navi fused-mask semantics: base geometry
/// OR live marks. Skipped when it shares the base frame.
using OverlayMaskReader = std::function<bool(EsdfMaskView&)>;
/// @brief 2D rigid transform lookup, target <- source (translation + yaw;
/// false when unavailable), i.e. it answers p_target = (t, R(yaw)) * p_source.
/// The writer samples base-frame cell centers in the overlay grid, so it
/// calls this with (target = overlay frame, source = base frame). The
/// package is Eigen-free by design.
using TransformLookup =
    std::function<bool(const std::string& target, const std::string& source,
                       double& tx, double& ty, double& yaw)>;
/// @brief Published-snapshot observer: called synchronously on the writer
/// thread right after every atomic snapshot swap. Must be cheap and must not
/// throw. The package stays ayanami-free by design -- hosts bridge this into
/// their own callback-slot emission (e.g. NavigationCallbacks::on_esdfmap_field).
using SnapshotListener =
    std::function<void(const std::shared_ptr<const EsdfSnapshot>&)>;

struct EsdfWriterConfig {
  /// @brief Positive clamp of the signed field (meters).
  double max_distance{3.0};
  /// @brief Rebuild rate ceiling in Hz (0 = no periodic thread; use
  /// rebuildOnce). The generation gate skips unchanged sources entirely.
  double update_rate{20.0};
};

/**
 * @brief The ESDF's sole writer: derives the field from an obstacle mask
 * (navi-style fused semantics) and publishes immutable snapshots.
 *
 * Pipeline per cycle (single writer thread; one map one writer):
 * 1. read the local obstacle mask (injected MaskReader);
 * 2. skip the rebuild when the source generation AND the raster geometry
 *    are unchanged (generation-gated: an unchanged map stays silent);
 * 3. fuse: the local mask OR the global static mask sampled through the
 *    injected transform (field-frame cell centers -> global frame -> grid
 *    lookup; a static global is re-projected per rebuild because the
 *    rolling window origin moves);
 * 4. exact signed EDT (Felzenszwalb, two fields: distance-to-obstacle in
 *    free space, distance-to-free inside obstacles), clamped to
 *    +-max_distance;
 * 5. publish an immutable EsdfSnapshot (atomic swap; sequence increments).
 *
 * The thread model mirrors the ayanami costmap writer: start() spawns the
 * resident thread, stop() joins it; rebuildOnce() is the synchronous
 * entry (inline/test use, same code path).
 */
class EsdfWriter {
 public:
  using Ptr = std::shared_ptr<EsdfWriter>;

  EsdfWriter(EsdfMap::Ptr map, MaskReader base_reader,
             EsdfWriterConfig config = {});
  ~EsdfWriter();

  EsdfWriter(const EsdfWriter&) = delete;
  EsdfWriter& operator=(const EsdfWriter&) = delete;

  /// @brief Attach the optional overlay mask + the transform lookup used
  /// to project it into the base frame (call before start).
  void setOverlaySource(OverlayMaskReader reader, TransformLookup lookup);

  /// @brief Attach the published-snapshot observer (call before start).
  void setSnapshotListener(SnapshotListener listener);

  /// @brief Spawn the resident rebuild thread (idempotent).
  void start();

  /// @brief Stop and join the resident thread (idempotent).
  void stop();

  /// @brief One synchronous rebuild (the resident thread's body, one
  /// cycle). Returns true when a new snapshot was published.
  bool rebuildOnce();

  /// @brief Number of rebuilds skipped by the generation gate so far
  /// (observability: a healthy deployment skips most cycles).
  std::uint64_t skippedRebuilds() const {
    return skipped_rebuilds_.load(std::memory_order_relaxed);
  }

 private:
  void cycleLoop();

  EsdfMap::Ptr map_;
  MaskReader base_reader_;
  OverlayMaskReader overlay_reader_;
  TransformLookup transform_lookup_;
  SnapshotListener snapshot_listener_;
  EsdfWriterConfig config_;

  // Last-applied source state (generation gate: BOTH sources count -- the
  // base rarely changes while the overlay tracks the live marks).
  std::uint64_t last_base_generation_{0};
  std::uint64_t last_overlay_generation_{0};
  double last_origin_x_{0.0};
  double last_origin_y_{0.0};
  bool has_last_{false};
  std::uint64_t sequence_{0};
  // Scratch buffers reused across rebuilds (assign, not reallocate).
  std::vector<unsigned char> fused_;
  std::vector<float> distance_;

  std::atomic<std::uint64_t> skipped_rebuilds_{0};
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::mutex cycle_mutex_;
  std::condition_variable cycle_cv_;
};

}  // namespace esdfmap
