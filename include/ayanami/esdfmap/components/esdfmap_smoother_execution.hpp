#pragma once

#include <memory>
#include <string>
#include <utility>

#include <esdfmap/esdf_map.hpp>

#include "ayanami/abstract/components/smoother_execution.hpp"
#include "ayanami/esdfmap/core/esdfmap_smoother.hpp"

namespace ayanami {
namespace esdfmap {

/// @brief The esdfmap variant of the path smoothing execution.
///
/// Binds an ESDF map on top of SmootherExecution:
/// - safetyCheck: field freshness (snapshot exists + optional stale gate);
/// - callSmoother: typed call into EsdfmapSmoother; the module reads its
///   held snapshot internally for the whole optimization run.
class EsdfmapSmootherExecution : public ayanami::components::SmootherExecution {
 public:
  using Ptr = std::shared_ptr<EsdfmapSmootherExecution>;

  EsdfmapSmootherExecution(const std::string& name,
                           ayanami::abstract::AbstractSmoother::Ptr smoother,
                           ::esdfmap::EsdfMap::Ptr map)
      : SmootherExecution(std::move(smoother)),
        name(name),
        map(std::move(map)) {}

  ~EsdfmapSmootherExecution() override = default;

  bool safetyCheck() override { return !map || map->current(); }

 protected:
  ResultT<void> callSmoother(ayanami::nav_msgs::Path& path,
                             double max_duration) override {
    auto* typed =
        dynamic_cast<EsdfmapSmoother*>(smoother.get());
    if (!typed) {
      return ResultT<void>::failure(
          ErrorCode::INTERNAL_ERROR,
          "EsdfmapSmootherExecution: module is not an EsdfmapSmoother");
    }
    if (!map) {
      return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                    "EsdfmapSmootherExecution: no esdf map");
    }
    return typed->smoothPath(path, max_duration);
  }

 private:
  std::string name;
  ::esdfmap::EsdfMap::Ptr map;
};

}  // namespace esdfmap
}  // namespace ayanami
