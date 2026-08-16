#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 恢复行为插件抽象 (spec §5.1)。
 *
 * 注意: 恢复类命名一律 recovery。MBF 语义: 恢复是插件化、可取消的动作。
 */
class AbstractRecovery {
public:
  using Ptr = std::shared_ptr<AbstractRecovery>;

  virtual ~AbstractRecovery() = default;

  /**
   * @brief 执行一次恢复动作。
   * @param[out] message 失败时的人类可读错误。
   * @return 错误码: 0=SUCCESS, 300+ 为错误段。
   */
  virtual uint32_t runRecovery(std::string& message) = 0;

  /// @brief 取消进行中的恢复。默认不支持, 返回 false。
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
