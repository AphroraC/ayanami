// 测试插件: 成功 smoother (SuccessSmoother), 经公共工厂 create_extension_module 导出。

#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::FailureCode;
using ayanami::nav_msgs::Path;

class SuccessSmoother : public ayanami::abstract::AbstractSmoother {
public:
  FailureCode smoothPath(const Path& in, Path& out, double /*max_duration*/,
                         bool /*check_for_collision*/,
                         std::string& /*message*/) override {
    out.header.frame_id = "map";
    out.poses = in.poses;
    return FailureCode::None;
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SuccessSmoother;
}