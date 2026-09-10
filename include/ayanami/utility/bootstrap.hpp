#pragma once

#include <string>

namespace ayanami {

/**
 * @brief One-shot process bootstrap: point GlobalConfig at config_path and
 * configure logging from it.
 *
 * This replaces the `GlobalConfig::instance(path, /*override_path=*​/true)` call
 * that used to appear in four places on the startup path (host initialize, core
 * configureCoreMaps, core loadModules, BT pilot configure), each of which
 * deleted and re-read config.json. It also removes the ordering trap the old
 * arrangement had: configureLogging() reads GlobalConfig through the *lazy*
 * accessor, so whichever component ran first silently decided which directory
 * the entire process logged from.
 *
 * Safe to call from every layer that needs the config root (the four startup
 * sites all do, with the same path). The config singleton is always rebuilt --
 * a caller may have rewritten the file at that path, so "same path" must not
 * mean "same content" -- while logging is configured once per path to avoid
 * reopening the log file at every call site.
 *
 * Not for per-navigation use: the configuration root is a process-wide
 * lifecycle property, not per-goal state.
 */
void bootstrap(const std::string& config_path);

} // namespace ayanami
