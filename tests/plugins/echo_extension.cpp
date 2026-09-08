#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/extension_module.hpp>

#include <atomic>

namespace {

// Cross-.so verification counter: a static variable inside this .so, read by echoExtensionHits.
std::atomic<int> g_hits{0};

class EchoExtension : public ayanami::ExtensionModule {
public:
  EchoExtension() {
    // Adds itself to the NavigationCallbacks slot at construction time.
    // Because this .so links libayanami, the reference resolves to the same slot instance defined inside libayanami.
    ayanami::interfaces::NavigationCallbacks::on_get_path_result.add(
        [](const auto&){ g_hits.fetch_add(1, std::memory_order_relaxed); });
  }
};

} // namespace

extern "C" ayanami::ExtensionModule* create_extension_module() {
  return new EchoExtension;
}

extern "C" int echoExtensionHits() {
  return g_hits.load(std::memory_order_relaxed);
}
