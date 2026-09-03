#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 路径平滑插件抽象 (spec §5.1, MBF 式分层)。
 *
 * 零地图类型: 接口不包含任何地图参数; 地图依赖只出现在地图族层
 * (如 CostmapSmoother) 的 initialize(地图模块) 注入点。
 */
class AbstractSmoother : public virtual ExtensionModule {
public:
  using Ptr = std::shared_ptr<AbstractSmoother>;

  virtual ~AbstractSmoother() = default;

  /**
   * @brief 平滑给定路径 (纯虚, 插件必须实现)。
   * @param[out] smoothed_path 平滑后的路径。
   * @param[out] message 失败时的人类可读诊断 (仅诊断, 决策只看 failure)。
   * @return FailureCode: None=成功, 其余为统一失败码。
   */
  virtual FailureCode smoothPath(const ayanami::nav_msgs::Path& path,
                                 ayanami::nav_msgs::Path& smoothed_path,
                                 double max_duration, bool check_for_collision,
                                  std::string& message) = 0;

  virtual void configure() {}
  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief 取消进行中的平滑。默认不支持, 返回 false。
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
