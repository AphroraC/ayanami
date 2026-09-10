#include <ayanami/utility/bootstrap.hpp>

#include <mutex>

#include <ayanami/utility/config.hpp>
#include <ayanami/utility/logging.hpp>

namespace ayanami {

void bootstrap(const std::string& config_path) {
  static std::mutex mutex;
  static std::string logging_configured_for;
  static bool logging_configured = false;

  std::lock_guard<std::mutex> lock(mutex);
  // Always rebuild the singleton. The idiom this replaces
  // (GlobalConfig::instance(path, /*override_path=*/true)) was unconditional,
  // and that matters: a caller may rewrite the config at the same path -- tests
  // do exactly that -- so "same path" does not mean "same content". Skipping
  // here silently served a stale document.
  GlobalConfig::instance(config_path, /*override_path=*/true);
  // Logging is derived only from the `logging` section of that same document
  // and does not change between the startup call sites, so configure it once
  // per path: four startup sites all bootstrapping would otherwise reopen the
  // log file four times.
  if (!logging_configured || logging_configured_for != config_path) {
    configureLogging();
    logging_configured = true;
    logging_configured_for = config_path;
  }
}

} // namespace ayanami
