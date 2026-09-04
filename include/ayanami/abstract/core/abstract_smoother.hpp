#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 路径平滑插件抽象 (MBF 式分层)。
 *
 * 零地图类型: 接口不包含任何地图参数。in-place 语义: 直接平滑传入路径,
 * 成功时路径已被修改, 结果只携带状态与诊断。
 */
class AbstractSmoother : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractSmoother>;

  virtual ~AbstractSmoother() = default;

  /// @brief 就地平滑给定路径 (纯虚)。
  /// @param[in,out] path 待平滑路径; 成功时已被就地更新。
  /// @return SmootherResultT: ok=true 时 path 有效; 失败携带 ErrorCode。
  virtual SmootherResultT smoothPath(ayanami::nav_msgs::Path& path,
                                     double max_duration,
                                     bool check_for_collision) = 0;

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief 取消进行中的平滑 (协作式中断; 默认不支持)。
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
