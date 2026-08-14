#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <atomic>

namespace {

// 跨 .so 验证计数: 本 .so 内的静态变量, 由 echo_extension_hits() 读取。
std::atomic<int> g_hits{0};

class EchoExtension : public ayanami::ExtensionModule {
public:
  EchoExtension() {
    // 构造期 add 到 NavigationCallbacks 槽 (spec §4.6)。
    // 因本 .so 链接 libayanami, 该引用解析到 libayanami 内定义的同一槽实例。
    ayanami::interfaces::NavigationCallbacks::on_get_path_result.add(
        [](const auto&){ g_hits.fetch_add(1, std::memory_order_relaxed); });
  }
};

} // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new EchoExtension;
}

extern "C" int echo_extension_hits() {
  return g_hits.load(std::memory_order_relaxed);
}
