// 测试插件: 非 costmap 族 smoother (PlainSmoother), 验证 costmap 宿主拒绝路径。

#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::SmootherResultT;

class PlainSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  SmootherResultT smoothPath(ayanami::nav_msgs::Path& /*path*/,
                             double /*max_duration*/,
                             bool /*check_for_collision*/) override {
    return SmootherResultT::failure(ayanami::ErrorCode::INVALID_INPUT,
                                    "plain smoother always fails");
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new PlainSmoother;
}
