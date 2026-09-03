// 测试插件: 非 costmap 族 smoother (PlainSmoother), 验证 costmap 宿主拒绝路径。

#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::FailureCode;
using ayanami::nav_msgs::Path;

class PlainSmoother : public ayanami::abstract::AbstractSmoother {
 public:
  FailureCode smoothPath(const Path& /*in*/, Path& /*out*/, double /*max_duration*/,
                         bool /*check_for_collision*/,
                         std::string& /*message*/) override {
    return FailureCode::InvalidInput;
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new PlainSmoother;
}