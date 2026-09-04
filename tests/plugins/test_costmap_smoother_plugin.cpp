// 测试插件: costmap 族 smoother (SoCostmapSmoother), 公共工厂导出。

#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::nav_msgs::Path;
using ayanami::SmootherResultT;

class SoCostmapSmoother : public ayanami::costmap::CostmapSmoother {
 public:
  void configure(const ayanami::PluginContext& /*context*/,
                 ayanami::costmap::CostmapMapContext::ConstPtr /*map*/) override {}
  SmootherResultT smoothPath(Path& path, double /*max_duration*/,
                             bool /*check_for_collision*/) override {
    path.header.frame_id = "map";
    return SmootherResultT::success(std::move(path));
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SoCostmapSmoother;
}
