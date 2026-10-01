#pragma once

#include <spdlog/spdlog.h>

#include <string>

namespace ayanami {

/**
 * @brief Well-known logger names, one per framework layer.
 *
 * Each layer logs through its own named logger so output lines carry a
 * [name] tag and each layer's verbosity can be tuned independently (see
 * configureLogging's logger_levels parameter).
 */
namespace logging {
inline constexpr const char* kRosLogger = "ros";
inline constexpr const char* kPlannerLogger = "planner";
inline constexpr const char* kSmootherLogger = "smoother";
inline constexpr const char* kControllerLogger = "controller";
inline constexpr const char* kRecoveryLogger = "recovery";
inline constexpr const char* kBtLogger = "BT";
inline constexpr const char* kCostmapLogger = "costmap";
}  // namespace logging

/**
 * @brief Parse a log level string (case-insensitive).
 *
 * Supports: trace / debug / info / warn|warning / error / critical / off.
 * Unknown strings fall back to info.
 */
spdlog::level::level_enum logLevelFromString(const std::string& level);

/**
 * @brief Create-or-get the named layer logger from spdlog's registry.
 *
 * spdlog is linked as a shared library, so the registry (and therefore the
 * logger object) is process-wide: the same instance is returned to every
 * caller, across .so boundaries. A newly created logger clones the current
 * default logger's sinks and level, so messages logged before
 * configureLogging still land somewhere sensible; configureLogging (re)assigns
 * sinks and levels for the known layer names.
 */
std::shared_ptr<spdlog::logger> layerLogger(const std::string& name);

/// Layer logger accessors (cached create-or-get; see layerLogger()).
std::shared_ptr<spdlog::logger> rosLogger();
std::shared_ptr<spdlog::logger> plannerLogger();
std::shared_ptr<spdlog::logger> smootherLogger();
std::shared_ptr<spdlog::logger> controllerLogger();
std::shared_ptr<spdlog::logger> recoveryLogger();
std::shared_ptr<spdlog::logger> btLogger();
std::shared_ptr<spdlog::logger> costmapLogger();

/**
 * @brief Configure the spdlog loggers from the root config.json's logging
 * section.
 *
 * Parameters (modeled after asuka):
 *   - logging_level    (string, default "info"): global level, i.e. the
 *     logger master switch applied to every registered logger
 *   - console_level    (string, default "info"): independent console level
 *     (the console sink's own gate)
 *   - console_output   (bool,   default true)  : whether to log to console
 *   - file_output      (bool,   default true)  : whether to log to file
 *   - logging_dir      (string, default "logs"): log directory (absolute, or
 *     relative to cwd; the costmap PGM dumps land here too)
 *   - rotate_logs      (bool,   default true)  : file rotation
 *   - max_file_size_kb (int,    default 8192)  : per-file rotation limit (KB)
 *   - max_files        (int,    default 10)    : rotated files retained
 *   - logger_levels    (object, optional)      : per-layer level overrides,
 *     e.g. {"costmap": "debug", "controller": "warn"}. Each known layer
 *     logger ("ros", "planner", "smoother", "controller", "recovery", "BT",
 *     "costmap") replaces its master-switch gate with its override; unknown
 *     names are ignored. A more verbose override than logging_level surfaces
 *     in the file (the file sink itself is ungated; gating happens at logger
 *     level) and on the console when console_level also allows it.
 *
 * The file name is <logging_dir>/ayanami.log. Log lines are tagged with the
 * logger name ([ayanami] for the unnamed-framework default, plus one tag per
 * layer logger). Idempotent and reconfigurable: each call rebuilds the
 * console/file sinks (other sinks are kept) and re-syncs levels to all
 * registered loggers.
 */
void configureLogging();

}  // namespace ayanami
