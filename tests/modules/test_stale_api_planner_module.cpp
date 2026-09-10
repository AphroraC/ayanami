// Test module: a planner that claims a stale interface version.
//
// Stands in for a .so built against an older abstract_planner.hpp: it is a
// genuine AbstractPlanner (the RTTI cast succeeds) but reports a different
// interface version, so the loader must refuse it. Without the version check
// this is the silent-UB case: matching class names over disagreeing virtual
// layouts.

#include <ayanami/abstract/core/abstract_planner.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::geometry_msgs::PoseStamped;
using ayanami::ResultT;

class StaleApiPlanner : public ayanami::abstract::AbstractPlanner {
 public:
  /// @brief Deliberately not AbstractPlanner::kApiVersion.
  std::uint32_t apiVersion() const override { return 999; }

  ResultT<void> makePlan(const PoseStamped& /*start*/,
                          const PoseStamped& /*goal*/, double /*tolerance*/,
                          ayanami::nav_msgs::Path& plan_out) override {
    (void)plan_out;
    return ResultT<void>::failure(ayanami::ErrorCode::PATH_NOT_FOUND,
                                   "stale-version test module");
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new StaleApiPlanner;
}
