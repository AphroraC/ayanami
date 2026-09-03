// 测试插件: costmap 族 smoother (SoCostmapSmoother), 公共工厂导出。

#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::FailureCode;
using ayanami::nav_msgs::Path;

class SoCostmapSmoother : public ayanami::costmap::CostmapSmoother {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::ConstPtr /*map*/) override {}
  FailureCode smoothPath(const Path& in, Path& out, double /*max_duration*/,
      bool /*check_for_collision*/, std::string& /*message*/) override {
    out.header.frame_id = "map";
    out.poses = in.poses;
    return FailureCode::None;
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SoCostmapSmoother;
}