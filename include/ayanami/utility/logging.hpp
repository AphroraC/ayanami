#pragma once

#include <spdlog/spdlog.h>

#include <string>

namespace ayanami {

/**
 * @brief 解析日志级别字符串 (大小写不敏感)。
 *
 * 支持: trace / debug / info / warn|warning / error / critical / off。
 * 未知字符串回退 info。
 */
spdlog::level::level_enum log_level_from_string(const std::string& level);

/**
 * @brief 从 config_logging.json 的 logging 段配置 spdlog 默认 logger。
 *
 * 参数 (学习 asuka):
 *   - logging_level    (string, 默认 "info"): 文件/全局级别 (logger 总闸)
 *   - console_level    (string, 默认 "info"): 控制台独立级别
 *   - console_output   (bool,   默认 true)  : 是否输出控制台
 *   - file_output      (bool,   默认 true)  : 是否输出文件
 *   - logging_dir      (string, 默认 "logs"): 日志目录 (相对 cwd)
 *   - rotate_logs      (bool,   默认 true)  : 文件轮转
 *   - max_file_size_kb (int,    默认 8192)  : 轮转单文件上限 (KB)
 *   - max_files        (int,    默认 10)    : 轮转保留文件数
 *
 * 文件名为 <logging_dir>/ayanami.log。幂等且可重配: 每次调用重建
 * console/file sink (保留其他 sink), 全局级别同步到所有已注册 logger。
 */
void configure_logging();

}  // namespace ayanami
