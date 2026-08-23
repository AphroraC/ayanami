// logging_test: log_level_from_string 解析 + configure_logging 行为
// (console_output / file_output / 轮转参数 / 级别, 学习 asuka 参数集)。
//
// 覆盖:
//   1. 级别字符串解析 (trace/debug/info/warn/warning/error/critical/off/未知)
//   2. file_output=true → 日志文件创建且内容写入; console_output=false →
//      console sink 移除
//   3. logging_level 全局同步到已注册 logger
//   4. 轮转参数传递 (rotating_file_sink)

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

std::filesystem::path make_temp_dir() {
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

// 写最小 config 目录: config.json + config_logging.json。
std::filesystem::path make_config_dir(const std::string& logging_json) {
  const auto dir = make_temp_dir();
  write_file(dir / "config.json",
             "{\n  \"global\": {\n"
             "    \"config_logging\": \"config_logging.json\"\n"
             "  }\n}\n");
  // logging_dir 占位符替换为实际测试目录。
  std::string json = logging_json;
  const std::string kPlaceholder = "LOGGING_DIR_PLACEHOLDER";
  const auto pos = json.find(kPlaceholder);
  if (pos != std::string::npos) {
    json.replace(pos, kPlaceholder.size(), dir.string());
  }
  write_file(dir / "config_logging.json", json);
  return dir;
}

}  // namespace

TEST(LoggingTest, LogLevelFromStringParses) {
  using ayanami::log_level_from_string;
  EXPECT_EQ(log_level_from_string("trace"), spdlog::level::trace);
  EXPECT_EQ(log_level_from_string("debug"), spdlog::level::debug);
  EXPECT_EQ(log_level_from_string("info"), spdlog::level::info);
  EXPECT_EQ(log_level_from_string("warn"), spdlog::level::warn);
  EXPECT_EQ(log_level_from_string("warning"), spdlog::level::warn);
  EXPECT_EQ(log_level_from_string("ERROR"), spdlog::level::err);
  EXPECT_EQ(log_level_from_string("critical"), spdlog::level::critical);
  EXPECT_EQ(log_level_from_string("off"), spdlog::level::off);
  // 未知字符串回退 info。
  EXPECT_EQ(log_level_from_string("bogus"), spdlog::level::info);
}

TEST(LoggingTest, FileOutputTrueWritesLogFile) {
  const auto dir = make_config_dir(
      "{\n  \"logging\": {\n"
      "    \"console_output\": false,\n"
      "    \"file_output\": true,\n"
      "    \"logging_dir\": \"LOGGING_DIR_PLACEHOLDER\",\n"
      "    \"logging_level\": \"debug\",\n"
      "    \"rotate_logs\": false\n"
      "  }\n}\n");
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);
  ayanami::configure_logging();

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

TEST(LoggingTest, ConsoleOutputTrueKeepsConsoleAndRotatingSink) {
  const auto dir = make_config_dir(
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
  ayanami::configure_logging();

  auto logger = spdlog::default_logger();
  ASSERT_NE(logger, nullptr);
  EXPECT_TRUE(has_console_sink(logger));
  EXPECT_TRUE(has_file_sink(logger));
  // logger 总闸 = min(info, debug) = debug (console 更详细不被截断)。
  EXPECT_EQ(logger->level(), spdlog::level::debug);

  // rotating sink: 创建目录 + 文件可写。
  spdlog::info("rotating marker");
  logger->flush();
  std::ifstream ifs(dir / "ayanami.log");
  EXPECT_TRUE(ifs.good());
}

TEST(LoggingTest, ReconfigureIsIdempotentNoSinkDuplication) {
  const auto dir = make_config_dir(
      "{\n  \"logging\": {\n"
      "    \"console_output\": true,\n"
      "    \"file_output\": true,\n"
      "    \"logging_dir\": \"LOGGING_DIR_PLACEHOLDER\",\n"
      "    \"logging_level\": \"info\",\n"
      "    \"rotate_logs\": true\n"
      "  }\n}\n");
  ayanami::GlobalConfig::instance(dir.string(), /*override_path=*/true);
  ayanami::configure_logging();
  auto logger = spdlog::default_logger();
  const auto sink_count = logger->sinks().size();
  ASSERT_GE(sink_count, 2u);

  // 再次配置: sink 重建而非追加。
  ayanami::configure_logging();
  EXPECT_EQ(logger->sinks().size(), sink_count);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
