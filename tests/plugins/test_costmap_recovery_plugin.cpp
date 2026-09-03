// 测试插件: costmap 族 recovery (SoCostmapRecovery), 公共工厂导出。

#include <ayanami/costmap/core/costmap_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/utility/extension_module.hpp>

namespace {

using ayanami::FailureCode;

class SoCostmapRecovery : public ayanami::costmap::CostmapRecovery {
 public:
  void initialize(const std::string& /*name*/,
                  ayanami::costmap::CostmapContext::ConstPtr /*map*/) override {}
  FailureCode runRecovery(std::string& /*message*/) override {
    return FailureCode::None;
  }
  bool cancel() override { return true; }
};

}  // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new SoCostmapRecovery;
}