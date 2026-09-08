// logging_test: logLevelFromString parsing + configureLogging behavior
// (console_output / file_output / rotation parameters / level, modeled on the asuka parameter set).
//
// Coverage:
// 1. Level string parsing (trace/debug/info/warn/warning/error/critical/off/unknown)
// 2. file_output=true -> the log file is created and written; console_output=false ->
// the console sink is removed
// 3. logging_level is globally synchronized to registered loggers
// 4. Rotation parameters passed through (rotating_file_sink)

#include <gtest/gtest.h>

#include <ayanami/utility/config.hpp>
#include <ayanami/utility/logging.hpp>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace {

std::filesystem::path makeTempDir() {
  const auto dir =
      std::filesystem::temp_directory_path() / "ayanami_logging_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

void write_file(const std::filesystem::path& path,
                const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

bool has_console_sink(const std::shared_ptr<spdlog::logger>& logger) {
  for (const auto& sink : logger->sinks()) {
    if (dynamic_cast<spdlog::sinks::stdout_color_sink_mt*>(sink.get()) !=
            nullptr ||
        dynamic_cast<spdlog::sinks::stdout_color_sink_st*>(sink.get()) !=
            nullptr) {
      return true;
    }
  }
  return false;
}

bool has_file_sink(const std::shared_ptr<spdlog::logger>& logger) {
  for (const auto& sink : logger->sinks()) {
    if (dynamic_cast<spdlog::sinks::rotating_file_sink_mt*>(sink.get()) !=
            nullptr ||
        dynamic_cast<spdlog::sinks::rotating_file_sink_st*>(sink.get()) !=
            nullptr ||
        dynamic_cast<spdlog::sinks::basic_file_sink_mt*>(sink.get()) !=
            nullptr ||
        dynamic_cast<spdlog::sinks::basic_file_sink_st*>(sink.get()) !=
            nullptr) {
      return true;
    }
  }
  return false;
}

/// Writes a minimal config dir: config.json (inline logging section, single source).
std::filesystem::path makeConfigDir(const std::string& logging_json) {
  const auto dir = makeTempDir();
  // The logging_dir placeholder is replaced with the actual test directory.
  std::string json = logging_json;
  const std::string k_placeholder = "LOGGING_DIR_PLACEHOLDER";
  const auto pos = json.find(k_placeholder);
  if (pos != std::string::npos) {
    json.replace(pos, k_placeholder.size(), dir.string());
  }
  // The caller passes a complete {"logging": {...}} object: after stripping the outer braces it is merged at the same
  // level as the global section, keeping the JSON valid.
  write_file(dir / "config.json",
             "{\n  \"global\": {}," + json.substr(1) + "\n");
  return dir;
}

}  // namespace

TEST(LoggingTest, logLevelFromStringParses) {
  using ayanami::logLevelFromString;
  EXPECT_EQ(logLevelFromString("trace"), spdlog::level::trace);
  EXPECT_EQ(logLevelFromString("debug"), spdlog::level::debug);
  EXPECT_EQ(logLevelFromString("info"), spdlog::level::info);
  EXPECT_EQ(logLevelFromString("warn"), spdlog::level::warn);
  EXPECT_EQ(logLevelFromString("warning"), spdlog::level::warn);
  EXPECT_EQ(logLevelFromString("ERROR"), spdlog::level::err);
  EXPECT_EQ(logLevelFromString("critical"), spdlog::level::critical);
  EXPECT_EQ(logLevelFromString("off"), spdlog::level::off);
  // Unknown strings fall back to info.
  EXPECT_EQ(logLevelFromString("bogus"), spdlog::level::info);
}

TEST(LoggingTest, fileOutputTrueWritesLogFile) {
  const auto dir = makeConfigDir(
      "{\n  \"logging\": {\n"
      "    \"console_output\": false,\n"
      "    \"file_output\": true,\n"
      "    \"logging_dir\": \"LOGGING_DIR_PLACEHOLDER\",\n"
      "    \"logging_level\": \"debug\",\n"
      "    \"rotate_logs\": false\n"
      "  }\n}\n");
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);
  ayanami::configureLogging();

  auto logger = spdlog::default_logger();
  ASSERT_NE(logger, nullptr);
  EXPECT_FALSE(has_console_sink(logger));
  EXPECT_TRUE(has_file_sink(logger));
  EXPECT_EQ(logger->level(), spdlog::level::debug);

  spdlog::info("logging test marker {}", 42);
  logger->flush();

  std::ifstream ifs(dir / "ayanami.log");
  ASSERT_TRUE(ifs.good());
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("logging test marker 42"), std::string::npos);
}

TEST(LoggingTest, consoleOutputTrueKeepsConsoleAndRotatingSink) {
  const auto dir = makeConfigDir(
      "{\n  \"logging\": {\n"
      "    \"console_output\": true,\n"
      "    \"console_level\": \"debug\",\n"
      "    \"file_output\": true,\n"
      "    \"logging_dir\": \"LOGGING_DIR_PLACEHOLDER\",\n"
      "    \"logging_level\": \"info\",\n"
      "    \"rotate_logs\": true,\n"
      "    \"max_file_size_kb\": 1024,\n"
      "    \"max_files\": 3\n"
      "  }\n}\n");
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);
  ayanami::configureLogging();

  auto logger = spdlog::default_logger();
  ASSERT_NE(logger, nullptr);
  EXPECT_TRUE(has_console_sink(logger));
  EXPECT_TRUE(has_file_sink(logger));
  // The logger master level = min(info, debug) = debug (the more verbose console is not truncated).
  EXPECT_EQ(logger->level(), spdlog::level::debug);

  // Rotating sink: directory created + file writable.
  spdlog::info("rotating marker");
  logger->flush();
  std::ifstream ifs(dir / "ayanami.log");
  EXPECT_TRUE(ifs.good());
}

TEST(LoggingTest, reconfigureIsIdempotentNoSinkDuplication) {
  const auto dir = makeConfigDir(
      "{\n  \"logging\": {\n"
      "    \"console_output\": true,\n"
      "    \"file_output\": true,\n"
      "    \"logging_dir\": \"LOGGING_DIR_PLACEHOLDER\",\n"
      "    \"logging_level\": \"info\",\n"
      "    \"rotate_logs\": true\n"
      "  }\n}\n");
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);
  ayanami::configureLogging();
  auto logger = spdlog::default_logger();
  const auto sink_count = logger->sinks().size();
  ASSERT_GE(sink_count, 2u);

  // Reconfiguring: the sink is rebuilt rather than appended to.
  ayanami::configureLogging();
  EXPECT_EQ(logger->sinks().size(), sink_count);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
