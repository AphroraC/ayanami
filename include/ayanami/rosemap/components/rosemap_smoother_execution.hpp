#pragma once

#include <memory>
#include <string>
#include <utility>

#include "ayanami/abstract/components/smoother_execution.hpp"
#include "ayanami/rosemap/core/rosemap_map_context.hpp"
#include "ayanami/rosemap/core/rosemap_smoother.hpp"

namespace ayanami {
namespace rosemap {

/// @brief The rosemap variant of the path smoothing execution.
///
/// Binds a rosemap field context on top of SmootherExecution:
/// - safetyCheck: field freshness (snapshot exists + optional stale gate);
/// - callSmoother: typed call into RosemapSmoother; the module reads its
///   held snapshot internally for the whole optimization run.
class RosemapSmootherExecution : public ayanami::components::SmootherExecution {
 public:
  using Ptr = std::shared_ptr<RosemapSmootherExecution>;

  RosemapSmootherExecution(const std::string& name,
                           ayanami::abstract::AbstractSmoother::Ptr smoother,
                           RosemapMapContext::Ptr ctx)
      : SmootherExecution(std::move(smoother)),
        name(name),
        ctx(std::move(ctx)) {}

  ~RosemapSmootherExecution() override = default;

  bool safetyCheck() override { return !ctx || ctx->current(); }

 protected:
  ResultT<void> callSmoother(ayanami::nav_msgs::Path& path,
                             double max_duration) override {
    auto* typed = dynamic_cast<RosemapSmoother*>(smoother.get());
    if (!typed) {
      return ResultT<void>::failure(
          ErrorCode::INTERNAL_ERROR,
          "RosemapSmootherExecution: module is not a RosemapSmoother");
    }
    if (!ctx) {
      return ResultT<void>::failure(
          ErrorCode::MAP_UNAVAILABLE,
          "RosemapSmootherExecution: no rosemap field");
    }
    return typed->smoothPath(path, max_duration);
  }

 private:
  std::string name;
  RosemapMapContext::Ptr ctx;
};

}  // namespace rosemap
}  // namespace ayanami
