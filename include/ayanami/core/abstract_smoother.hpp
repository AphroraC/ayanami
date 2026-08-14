#pragma once

#include "ayanami/messages/nav_msgs.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 路径平滑插件抽象 (spec §5.1)。
 */
class AbstractSmoother {
public:
  using Ptr = std::shared_ptr<AbstractSmoother>;

  /**
   * @brief 平滑给定路径。
   * @param[out] smoothed_path 平滑后的路径。
   * @param[out] message 失败时的人类可读错误。
   * @return 错误码: 0=SUCCESS, 200+ 为错误段。
   */
  virtual uint32_t smoothPath(const ayanami::nav_msgs::Path& path,
                              ayanami::nav_msgs::Path& smoothed_path,
                              double max_duration, bool check_for_collision,
                              std::string& message) = 0;

  /// @brief 取消进行中的平滑。默认不支持, 返回 false。
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
