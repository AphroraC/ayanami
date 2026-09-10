// Test module: non-costmap-family smoother (PlainSmoother), verifies the costmap host reject path.

#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::ResultT;
class PlainSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  ResultT<void> smoothPath(ayanami::nav_msgs::Path& /*path*/,
                           double /*max_duration*/) override {
    return ResultT<void>::failure(ayanami::ErrorCode::INVALID_INPUT,
                                    "plain smoother always fails");
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new PlainSmoother;
}
