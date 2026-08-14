#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 全局路径规划插件抽象 (spec §5.1)。
 *
 * 零地图类型: 只消费自研 messages。派生 (如 CostmapPlanner, spec §5.5)
 * 通过 initialize*Plugin 注入 AbstractMapContext, 本接口不知道地图形式。
 */
class AbstractPlanner {
public:
  using Ptr = std::shared_ptr<AbstractPlanner>;

  virtual ~AbstractPlanner() = default;

  /**
   * @brief 计算 start→goal 的全局路径。
   * @param[out] plan  生成的路径。
   * @param[out] cost  路径代价。
   * @param[out] message 失败时的人类可读错误。
   * @return 错误码: 0=SUCCESS, 50+ 为错误段 (NO_PATH_FOUND=58 等)。
   */
  virtual uint32_t makePlan(const ayanami::geometry_msgs::PoseStamped& start,
                            const ayanami::geometry_msgs::PoseStamped& goal,
                            double tolerance, ayanami::nav_msgs::Path& plan,
                            double& cost, std::string& message) = 0;

  /// @brief 取消进行中的规划。默认不支持, 返回 false。
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
