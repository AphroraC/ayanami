// logging_test: logLevelFromString 解析 + configureLogging 行为
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

// 写最小 config 目录: config.json (logging 段内联, 单一来源)。
std::filesystem::path makeConfigDir(const std::string& logging_json) {
  const auto dir = makeTempDir();
  // logging_dir 占位符替换为实际测试目录。
  std::string json = logging_json;
  const std::string k_placeholder = "LOGGING_DIR_PLACEHOLDER";
  const auto pos = json.find(k_placeholder);
  if (pos != std::string::npos) {
    json.replace(pos, k_placeholder.size(), dir.string());
  }
  // 调用方传入完整 {"logging": {...}} 对象: 剥掉外层花括号后与 global 段
  // 同级合并, 保证合法 JSON。
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
  // 未知字符串回退 info。
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
  // logger 总闸 = min(info, debug) = debug (console 更详细不被截断)。
  EXPECT_EQ(logger->level(), spdlog::level::debug);

  // rotating sink: 创建目录 + 文件可写。
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

  // 再次配置: sink 重建而非追加。
  ayanami::configureLogging();
  EXPECT_EQ(logger->sinks().size(), sink_count);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
