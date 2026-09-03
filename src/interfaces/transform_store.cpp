// TransformStore 实现 (跨 DSO 单例纪律: instance() 定义在 .cpp, 不 inline)。
#include "ayanami/interfaces/transform_store.hpp"

#include <chrono>
#include <exception>

namespace ayanami {
namespace interfaces {

namespace {
inline fast_tf::detail::time_t toFastTfTime(const ayanami::Timestamp& stamp) {
  return fast_tf::detail::time_t{
      std::chrono::duration_cast<fast_tf::detail::duration_t>(
          std::chrono::nanoseconds(stamp.count()))};
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
  buffer.set(parent, child, toFastTfTime(stamp), T, is_static);
}

bool
TransformStore::lookupTransform(const std::string& target,
                                const std::string& source, Timestamp stamp,
                                Eigen::Isometry3d& out, double timeout) const {
  const auto tolerance = std::chrono::duration_cast<fast_tf::detail::duration_t>(
      std::chrono::duration<double>(timeout));
  try {
    out = buffer.get(target, source, toFastTfTime(stamp), tolerance);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

TransformStore::Timestamp
TransformStore::fromSeconds(double s) {
  return timestampFromSeconds(s);
}

}  // namespace interfaces
}  // namespace ayanami
