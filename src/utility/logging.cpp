#include <ayanami/utility/logging.hpp>

#include <ayanami/utility/config.hpp>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <vector>
#include <memory>

namespace ayanami {

namespace {

bool is_console_sink(const spdlog::sink_ptr& sink) {
  return dynamic_cast<spdlog::sinks::stdout_sink_mt*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::stdout_sink_st*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::stdout_color_sink_mt*>(sink.get()) !=
             nullptr ||
         dynamic_cast<spdlog::sinks::stdout_color_sink_st*>(sink.get()) !=
             nullptr ||
         dynamic_cast<spdlog::sinks::stderr_sink_mt*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::stderr_sink_st*>(sink.get()) != nullptr ||
         dynamic_cast<spdlog::sinks::stderr_color_sink_mt*>(sink.get()) !=
             nullptr ||
         dynamic_cast<spdlog::sinks::stderr_color_sink_st*>(sink.get()) !=
             nullptr;
}

bool is_file_sink(const spdlog::sink_ptr& sink) {
  return dynamic_cast<spdlog::sinks::rotating_file_sink_mt*>(sink.get()) !=
             nullptr ||
         dynamic_cast<spdlog::sinks::rotating_file_sink_st*>(sink.get()) !=
             nullptr ||
         dynamic_cast<spdlog::sinks::basic_file_sink_mt*>(sink.get()) !=
             nullptr ||
         dynamic_cast<spdlog::sinks::basic_file_sink_st*>(sink.get()) !=
             nullptr;
}

}  // namespace

spdlog::level::level_enum logLevelFromString(const std::string& level) {
  std::string normalized = level;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
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

void configureLogging() {
  // logging 段单一来源: 根 config.json (无独立 config_logging 文件)。
  const Config* config = GlobalConfig::instance();

  const auto logging_level = logLevelFromString(
      config->param<std::string>("logging", "logging_level", std::string("info")));
  const auto console_level = logLevelFromString(
      config->param<std::string>("logging", "console_level", std::string("info")));
  const bool console_output =
      config->param<bool>("logging", "console_output", true);
  const bool file_output = config->param<bool>("logging", "file_output", true);

  auto logger = spdlog::default_logger();
  if (logger) {
    // 重建 sink 集: console (按需) + file (按需) + 保留其他 sink (ringbuffer 等)。
    std::vector<spdlog::sink_ptr> new_sinks;

    if (console_output) {
      auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_level(console_level);
      new_sinks.push_back(std::move(console_sink));
    }

    if (file_output) {
      const std::string log_dir = config->param<std::string>(
          "logging", "logging_dir", std::string("logs"));
      std::error_code ec;
      std::filesystem::create_directories(log_dir, ec);
      if (!ec) {
        const bool rotate_logs =
            config->param<bool>("logging", "rotate_logs", true);
        spdlog::sink_ptr file_sink;
        const std::string log_path = log_dir + "/ayanami.log";
        if (rotate_logs) {
          const size_t max_file_size_bytes = static_cast<size_t>(
              config->param<int>("logging", "max_file_size_kb", 8192)) *
              1024;
          const size_t max_files =
              static_cast<size_t>(config->param<int>("logging", "max_files", 10));
          file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
              log_path, max_file_size_bytes, max_files);
        } else {
          file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
              log_path, /*truncate=*/true);
        }
        file_sink->set_level(logging_level);
        new_sinks.push_back(std::move(file_sink));
      }
    }

    for (const auto& sink : logger->sinks()) {
      if (!is_console_sink(sink) && !is_file_sink(sink)) {
        new_sinks.push_back(sink);
      }
    }
    logger->sinks() = std::move(new_sinks);
  }

  // 全局级别: 所有已注册 logger 取 logging_level; default_logger 最后提升到
  // min(logging_level, console_level) (console 更详细时不被总闸截断)。
  spdlog::set_level(logging_level);
  spdlog::apply_all([logging_level](const std::shared_ptr<spdlog::logger>& l) {
    l->set_level(logging_level);
  });
  if (logger) {
    const auto min_level =
        (console_output && console_level < logging_level) ? console_level
                                                          : logging_level;
    logger->set_level(min_level);
  }
}

}  // namespace ayanami
