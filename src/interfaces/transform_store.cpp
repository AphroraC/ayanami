// TransformStore implementation (cross-DSO singleton discipline: instance defined in the .cpp, not inline).
#include "ayanami/interfaces/transform_store.hpp"

#include <chrono>
#include <exception>

#include <spdlog/spdlog.h>

namespace ayanami {
namespace interfaces {

namespace {
inline fast_tf::detail::time_t toFastTfTime(const ayanami::Timestamp& stamp) {
  return fast_tf::detail::time_t{
      std::chrono::duration_cast<fast_tf::detail::duration_t>(
          std::chrono::nanoseconds(stamp.count()))};
}

inline ayanami::Timestamp fromFastTfTime(fast_tf::detail::time_t time) {
  return ayanami::Timestamp(std::chrono::nanoseconds(
      time.time_since_epoch().count()));
}
}  // namespace

TransformStore&
TransformStore::instance() {
  static TransformStore store;
  return store;
}

void
TransformStore::setTransform(const std::string& parent,
                             const std::string& child, Timestamp stamp,
                             const Eigen::Isometry3d& T, bool is_static) {
  buffer->set(parent, child, toFastTfTime(stamp), T, is_static);
}

bool
TransformStore::lookupTransform(const std::string& target,
                                const std::string& source, Timestamp stamp,
                                Eigen::Isometry3d& out, double timeout) const {
  const auto tolerance = std::chrono::duration_cast<fast_tf::detail::duration_t>(
      std::chrono::duration<double>(timeout));
  try {
    out = buffer->get(target, source, toFastTfTime(stamp), tolerance);
    return true;
  } catch (const std::exception& e) {
 // Diagnostics (debug level): the real reason a map-stamp/pose query failed (no sample / outside pruning window / tree not connected).
    spdlog::debug("TransformStore: lookup {}<-{} @ {:.3f}s failed: {}", target,
                  source, stamp.toSeconds(), e.what());
    return false;
  }
}

TransformStore::Timestamp
TransformStore::fromSeconds(double s) {
  return timestampFromSeconds(s);
}

void
TransformStore::configureCacheDuration(double seconds) {
  const auto dur = std::chrono::duration_cast<fast_tf::detail::duration_t>(
      std::chrono::duration<double>(seconds));
 // Whole-buffer replacement (startup-phase only, see the hpp discipline): the old buffer is released together with its stored TF data.
  buffer = std::make_unique<fast_tf::detail::transform_buffer>(dur);
  spdlog::info("TransformStore: dynamic-link cache duration set to {:.3f}s",
               seconds);
}

bool
TransformStore::latestCommonTime(const std::string& target,
                                 const std::string& source,
                                 Timestamp& out) const {
  try {
    out = fromFastTfTime(buffer->latest_common_time(target, source));
    return true;
  } catch (const std::exception& e) {
    spdlog::debug("TransformStore: latest common time {}<-{} failed: {}",
                  target, source, e.what());
    return false;
  }
}

}  // namespace interfaces
}  // namespace ayanami
