#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "esdfmap/esdf_query.hpp"
#include "esdfmap/esdf_snapshot.hpp"

namespace esdfmap {

/**
 * @brief The ESDF read surface: immutable-snapshot holder + query contract.
 *
 * Concurrency model (one map, one writer, unlimited snapshot readers): the
 * writer publishes whole EsdfSnapshot instances by an atomic shared_ptr
 * swap (publishSnapshot); readers take the current snapshot lock-free
 * (currentSnapshot) and hold it for as long as they need -- a held snapshot
 * stays byte-stable because a published snapshot is never mutated. There is
 * deliberately NO reader lock and no callback view: the snapshot swap is
 * the consistency guarantee.
 *
 * Freshness: current() reports snapshot existence + the optional age gate
 * (setStaleTimeout; 0 disables age gating). Field semantics: points
 * outside the rolling window are UNOBSERVED, not blocked (isPathValid
 * passes them; queries return kOutOfField with +max_distance).
 */
class EsdfMap {
 public:
  using Ptr = std::shared_ptr<EsdfMap>;
  using SnapshotPtr = std::shared_ptr<const EsdfSnapshot>;

  /// @brief Publish a freshly built snapshot (atomic swap).
  void publishSnapshot(SnapshotPtr snapshot) {
    std::atomic_store_explicit(&current_snapshot_, std::move(snapshot),
                               std::memory_order_release);
  }

  /// @brief The immutable current snapshot (lock-free). Hold it for the
  /// duration of a search/optimization run.
  SnapshotPtr currentSnapshot() const {
    return std::atomic_load_explicit(&current_snapshot_,
                                     std::memory_order_acquire);
  }

  /// @brief Age gate for execution-layer safety checks (0 = no age gate:
  /// current once any snapshot exists).
  void setStaleTimeout(double seconds) { stale_timeout_ = seconds; }

  /// @brief True when a snapshot exists and (with a positive stale
  /// timeout) it is fresher than the timeout.
  bool current() const {
    const auto snapshot = currentSnapshot();
    if (!snapshot) {
      return false;
    }
    if (stale_timeout_ <= 0.0) {
      return true;
    }
    const double age = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() -
                           snapshot->received)
                           .count();
    return age <= stale_timeout_;
  }

  /// @brief Age of the current snapshot in seconds (+inf when absent).
  double fieldAgeSeconds() const {
    const auto snapshot = currentSnapshot();
    if (!snapshot) {
      return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         snapshot->received)
        .count();
  }

  /// @brief One-shot interpolated query at a world point (field frame).
  EsdfQueryResult query(double wx, double wy) const {
    const auto snapshot = currentSnapshot();
    if (!snapshot || !snapshot->valid()) {
      EsdfQueryResult unavailable;
      unavailable.status = EsdfQueryStatus::kOutOfField;
      return unavailable;
    }
    return sampleDistance(*snapshot, wx, wy);
  }

  /// @brief Sparse path-validity probe over points already expressed in
  /// the FIELD frame (the caller resolves the path frame; the package is
  /// transform-free). A point fails only when it lies INSIDE an obstacle
  /// (distance <= margin); points outside the rolling window pass --
  /// unobserved, not blocked. Traversed-prefix skipping (start from the
  /// point nearest the robot) is the caller's choice: this probe is
  /// uniform, mirroring the rosemap family's local-window contract.
  bool isPathValid(const std::vector<double>& xs,
                   const std::vector<double>& ys, double margin) const {
    if (xs.empty() || xs.size() != ys.size()) {
      return false;
    }
    const auto snapshot = currentSnapshot();
    if (!snapshot || !snapshot->valid()) {
      return true;  // no field yet: nothing observed, nothing blocked
    }
    for (std::size_t i = 0; i < xs.size(); ++i) {
      const EsdfQueryResult q = sampleDistance(*snapshot, xs[i], ys[i]);
      if (q.status != EsdfQueryStatus::kOk) {
        continue;  // outside the window: unobserved, passes
      }
      if (q.distance <= margin) {
        return false;
      }
    }
    return true;
  }

 private:
  SnapshotPtr current_snapshot_;
  double stale_timeout_{0.0};
};

}  // namespace esdfmap
