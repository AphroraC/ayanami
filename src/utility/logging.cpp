#include <ayanami/utility/logging.hpp>

#include <ayanami/utility/config.hpp>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <memory>
#include <utility>
#include <vector>

namespace ayanami {

namespace {

bool is_console_sink(const spdlog::sink_ptr& sink) {
  return dynamic_cast<spdlog::sinks::stdout_sink_mt*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::stdout_sink_st*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::stdout_color_sink_mt*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::stdout_color_sink_st*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::stderr_sink_mt*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::stderr_sink_st*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::stderr_color_sink_mt*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::stderr_color_sink_st*>(sink.get()) != nullptr;
}

bool is_file_sink(const spdlog::sink_ptr& sink) {
  return dynamic_cast<spdlog::sinks::rotating_file_sink_mt*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::rotating_file_sink_st*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::basic_file_sink_mt*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::basic_file_sink_st*>(sink.get()) != nullptr;
}

/// All layer loggers, in the order configureLogging assigns them.
constexpr const char* kLayerLoggers[] = {
  logging::kRosLogger,
  logging::kPlannerLogger,
  logging::kSmootherLogger,
  logging::kControllerLogger,
  logging::kRecoveryLogger,
  logging::kFsmLogger,
  logging::kCostmapLogger,
};

}  // namespace

spdlog::level::level_enum logLevelFromString(const std::string& level) {
  std::string normalized = level;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (normalized == "trace") return spdlog::level::trace;
  if (normalized == "debug") return spdlog::level::debug;
  if (normalized == "info") return spdlog::level::info;
  if (normalized == "warn" || normalized == "warning") {
    return spdlog::level::warn;
  }
  if (normalized == "error") return spdlog::level::err;
  if (normalized == "critical") return spdlog::level::critical;
  if (normalized == "off") return spdlog::level::off;
  return spdlog::level::info;
}

std::shared_ptr<spdlog::logger> layerLogger(const std::string& name) {
  if (auto existing = spdlog::get(name)) {
    return existing;
  }
  // Clone the current default logger's output so messages logged before
  // configureLogging still land somewhere sensible.
  const auto fallback = spdlog::default_logger();
  std::vector<spdlog::sink_ptr> sinks = fallback ? fallback->sinks() : std::vector<spdlog::sink_ptr>{};
  if (sinks.empty()) {
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  }
  auto logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
  logger->set_level(fallback ? fallback->level() : spdlog::level::info);
  try {
    spdlog::register_logger(logger);
  } catch (const spdlog::spdlog_ex&) {
    // Lost a creation race: the registered instance wins.
    return spdlog::get(name);
  }
  return logger;
}

std::shared_ptr<spdlog::logger> rosLogger() {
  static const auto logger = layerLogger(logging::kRosLogger);
  return logger;
}

std::shared_ptr<spdlog::logger> plannerLogger() {
  static const auto logger = layerLogger(logging::kPlannerLogger);
  return logger;
}

std::shared_ptr<spdlog::logger> smootherLogger() {
  static const auto logger = layerLogger(logging::kSmootherLogger);
  return logger;
}

std::shared_ptr<spdlog::logger> controllerLogger() {
  static const auto logger = layerLogger(logging::kControllerLogger);
  return logger;
}

std::shared_ptr<spdlog::logger> recoveryLogger() {
  static const auto logger = layerLogger(logging::kRecoveryLogger);
  return logger;
}

std::shared_ptr<spdlog::logger> fsmLogger() {
  static const auto logger = layerLogger(logging::kFsmLogger);
  return logger;
}

std::shared_ptr<spdlog::logger> costmapLogger() {
  static const auto logger = layerLogger(logging::kCostmapLogger);
  return logger;
}

void configureLogging() {
  // The logging section's single source of truth: the root config.json (no separate config_logging file).
  const Config* config = GlobalConfig::instance();

  const auto logging_level =
    logLevelFromString(config->param<std::string>("logging", "logging_level", std::string("info")));
  const auto console_level =
    logLevelFromString(config->param<std::string>("logging", "console_level", std::string("info")));
  const bool console_output = config->param<bool>("logging", "console_output", true);
  const bool file_output = config->param<bool>("logging", "file_output", true);
  const std::string log_dir =
    config->param<std::string>("logging", "logging_dir", std::string("logs"));
  if (file_output) {
    std::error_code ec;
    std::filesystem::create_directories(log_dir, ec);
  }

  // File-sink factory (asuka-style): one file per logger, honoring the
  // shared rotation settings. Ungated (trace): verbosity is decided per
  // logger, so a raised layer actually reaches its file.
  const auto make_file_sink = [&](const std::string& filename)
      -> std::shared_ptr<spdlog::sinks::sink> {
    const bool rotate_logs = config->param<bool>("logging", "rotate_logs", true);
    std::shared_ptr<spdlog::sinks::sink> sink;
    const std::string log_path = log_dir + "/" + filename;
    if (rotate_logs) {
      const size_t max_file_size_bytes =
        static_cast<size_t>(config->param<int>("logging", "max_file_size_kb", 8192)) * 1024;
      const size_t max_files = static_cast<size_t>(config->param<int>("logging", "max_files", 10));
      sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_path, max_file_size_bytes, max_files);
    } else {
      sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path, /*truncate=*/true);
    }
    sink->set_level(spdlog::level::trace);
    return sink;
  };

  // Console sink: ONE instance shared by the default logger and every layer
  // logger (the console stays a single interleaved stream; the files split).
  std::shared_ptr<spdlog::sinks::sink> console_sink;
  if (console_output) {
    auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    sink->set_level(console_level);
    console_sink = std::move(sink);
  }

  auto current_default = spdlog::default_logger();

  // Default logger: console + its own file (ayanami.log) + any non-console
  // sinks the default already carried (ringbuffer etc.).
  std::vector<spdlog::sink_ptr> default_sinks;
  if (console_sink) {
    default_sinks.push_back(console_sink);
  }
  if (file_output) {
    default_sinks.push_back(make_file_sink("ayanami.log"));
  }
  for (const auto& sink : current_default->sinks()) {
    if (!is_console_sink(sink) && !is_file_sink(sink)) {
      default_sinks.push_back(sink);
    }
  }

  // Tag every line with the logger name; the framework's unnamed default
  // logs as [ayanami].
  for (const auto& sink : default_sinks) {
    sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
  }

  // Layer loggers: the shared console + their OWN file
  // (ayanami_<layer>.log). Per-layer overrides are read with the quiet
  // optional param (missing keys inherit the master switch).
  const Config logger_levels = config->child("logging", "logger_levels");
  std::vector<std::pair<std::shared_ptr<spdlog::logger>, spdlog::level::level_enum>> overrides;
  for (const char* name : kLayerLoggers) {
    auto logger = layerLogger(name);
    std::vector<spdlog::sink_ptr> sinks;
    if (console_sink) {
      sinks.push_back(console_sink);
    }
    if (file_output) {
      sinks.push_back(make_file_sink(std::string("ayanami_") + name + ".log"));
    }
    for (const auto& sink : sinks) {
      sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
    }
    logger->sinks() = sinks;
    spdlog::level::level_enum level = logging_level;
    if (const auto level_str = logger_levels.param<std::string>("logger_levels", name)) {
      level = logLevelFromString(*level_str);
    }
    overrides.emplace_back(std::move(logger), level);
  }

  // Master switch first (spdlog::set_level covers every registered logger),
  // then the per-layer overrides -- otherwise it would stomp them.
  spdlog::set_level(logging_level);
  for (const auto& override_entry : overrides) {
    override_entry.first->set_level(override_entry.second);
  }

  // spdlog's built-in default logger is unnamed (an empty [] tag in the
  // pattern); replace it with a named one sharing the default sink set.
  // set_default_logger does not register, so spdlog::set_level/apply_all skip
  // it and its level is managed here: min(logging_level, console_level), so a
  // more verbose console is not truncated by the master switch.
  auto default_logger = std::make_shared<spdlog::logger>("ayanami", default_sinks.begin(), default_sinks.end());
  const auto min_level = (console_output && console_level < logging_level) ? console_level : logging_level;
  default_logger->set_level(min_level);
  spdlog::set_default_logger(default_logger);
}

}  // namespace ayanami