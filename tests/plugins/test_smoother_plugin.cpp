// 测试插件: 成功 smoother (SuccessSmoother), 经公共工厂 create_extension_module 导出。

#include <ayanami/abstract/core/abstract_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::nav_msgs::Path;
using ayanami::SmootherResultT;

class SuccessSmoother : public ayanami::abstract::AbstractSmoother {
public:
  SmootherResultT smoothPath(Path& path, double /*max_duration*/,
                             bool /*check_for_collision*/) override {
    path.header.frame_id = "map";
    return SmootherResultT::success(std::move(path));
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SuccessSmoother;
}
