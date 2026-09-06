// TransformStore 实现 (跨 DSO 单例纪律: instance 定义在 .cpp, 不 inline)。
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
 // 诊断 (debug 级): 落图/位姿查询失败的真实原因 (无样本/剪枝窗外/树未连通)。
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
 // 整 buffer 替换 (启动期专用, 见 hpp 纪律): 旧 buffer 连同已存 TF 一起释放。
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
