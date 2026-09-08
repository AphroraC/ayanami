#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fast_tf.hpp>

#include "ayanami/messages/timestamp.hpp"

namespace ayanami {
namespace interfaces {

/**
 * @brief Thread-safe singleton wrapping fast_tf::detail::transform_buffer.
 *
 * Holds the transform tree (static + timed dynamic links) in one global
 * buffer. Written by the ROS sync layer (periodic + sensor callbacks), read by
 * the pure C++ core (frame transforms, costmap frame, BT robot pose).
 *
 * Cross-DSO singleton discipline: instance() and all member definitions
 * live in src/interfaces/transform_store.cpp (not inline), guaranteeing
 * that multiple .so files resolve to the same instance.
 *
 * Time semantics (Header timestamp / TF common-time semantics):
 * - Externally, the middleware-agnostic ayanami::Timestamp (nanoseconds) is
 *   used uniformly and converted internally to fast_tf::time_t; fast_tf
 *   types do not leak into messages / callers.
 * - "Latest" uses the explicit ayanami::Timestamp::latest() sentinel and no
 *   longer relies on the implicit fromSeconds(0.0) latest (fromSeconds is
 *   kept only as a compatibility helper).
 */
class TransformStore {
public:
  /// @brief Middleware-agnostic timestamp (converted to/from fast_tf::detail::time_t internally).
  using Timestamp = ayanami::Timestamp;

  /// @brief Global singleton instance.
  static TransformStore& instance();

  /// @brief Insert a transform (parent -> child).
  void setTransform(const std::string& parent, const std::string& child,
                    Timestamp stamp, const Eigen::Isometry3d& T,
                    bool is_static = false);

  /// @brief Look up the transform from @p source frame to @p target frame.
  ///
  /// Time semantics: the latest sentinel ({0,0}) resolves on multi-hop
  /// chains to the latest common time (tf2 semantics: the minimum of the
  /// latest sample stamps of the dynamic edges on the common path, sampled
  /// at one instant); a single-edge query is that edge's latest sample. A
  /// concrete stamp is interpolated within the dynamic edge's cache window;
  /// outside it (bidirectional extrapolation) the query fails.
  ///
  /// @return true on success (fills @p out), false if no transform available.
  bool lookupTransform(const std::string& target, const std::string& source,
                       Timestamp stamp, Eigen::Isometry3d& out,
                       double timeout = 0.0) const;

  /// @brief Configure the dynamic edges' per-stamp lookback window
  /// (seconds; fast_tf pruning keeps [latest sample - window, latest
  /// sample]). Keeps the fast_tf default of 500ms when not configured.
  ///
  /// Startup-phase only (AyanamiROS construction / test injection):
  /// implemented as a whole-buffer replacement, so stored TF data is
  /// dropped; concurrent set/lookup is not allowed. The production path
  /// calls it once in the AyanamiROS constructor before any TF writes.
  void configureCacheDuration(double seconds);

  /// @brief Query the latest common time of the target/source chain
  /// (diagnostics/testing surface).
  /// @param out The chain's common latest time; the epoch zero value ({0,0},
  ///        the latest sentinel) means no dynamic edge on the chain holds
  ///        data.
  /// @return false means a frame is unknown or the two frames are not in the
  ///         same tree.
  bool latestCommonTime(const std::string& target, const std::string& source,
                        Timestamp& out) const;

  /// @brief Compatibility helper: seconds since epoch -> Timestamp.
  /// Production code querying "latest" should use Timestamp::latest
  /// explicitly, not fromSeconds(0).
  static Timestamp fromSeconds(double s);

private:
  TransformStore() = default;
  ~TransformStore() = default;
  TransformStore(const TransformStore&) = delete;
  TransformStore& operator=(const TransformStore&) = delete;

  /// @brief The underlying buffer. Held indirectly via unique_ptr:
  /// transform_buffer contains a mutex and is not assignable;
  /// configureCacheDuration is implemented as whole-object replacement
  /// (startup-phase only).
  std::unique_ptr<fast_tf::detail::transform_buffer> buffer{
      std::make_unique<fast_tf::detail::transform_buffer>()};
};

}  // namespace interfaces
}  // namespace ayanami
