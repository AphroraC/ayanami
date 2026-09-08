#pragma once

#include <spdlog/spdlog.h>

#include <string>

namespace ayanami {

/**
 * @brief Parse a log level string (case-insensitive).
 *
 * Supports: trace / debug / info / warn|warning / error / critical / off.
 * Unknown strings fall back to info.
 */
spdlog::level::level_enum logLevelFromString(const std::string& level);

/**
 * @brief Configure the spdlog default logger from the root config.json's
 * logging section.
 *
 * Parameters (modeled after asuka):
 *   - logging_level    (string, default "info"): file/global level (the
 *     logger master switch)
 *   - console_level    (string, default "info"): independent console level
 *   - console_output   (bool,   default true)  : whether to log to console
 *   - file_output      (bool,   default true)  : whether to log to file
 *   - logging_dir      (string, default "logs"): log directory (relative to
 *     cwd)
 *   - rotate_logs      (bool,   default true)  : file rotation
 *   - max_file_size_kb (int,    default 8192)  : per-file rotation limit (KB)
 *   - max_files        (int,    default 10)    : rotated files retained
 *
 * The file name is <logging_dir>/ayanami.log. Idempotent and reconfigurable:
 * each call rebuilds the console/file sinks (other sinks are kept) and syncs
 * the global level to all registered loggers.
 */
void configureLogging();

}  // namespace ayanami
