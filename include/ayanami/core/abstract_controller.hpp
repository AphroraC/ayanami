#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 局部控制器插件抽象 (spec §5.1)。
 *
 * 输入当前位姿/速度, 输出 cmd_vel; 判断是否到达目标; 接收待执行路径。
 * 零地图类型 (地图经 AbstractMapContext 注入)。
 */
class AbstractController {
public:
  using Ptr = std::shared_ptr<AbstractController>;

  virtual ~AbstractController() = default;

  /**
   * @brief 计算速度命令。
   * @param[out] cmd_vel  体坐标系速度命令。
   * @param[out] message  失败时的人类可读错误。
   * @return 错误码: 0=SUCCESS, 100+ 为错误段。
   */
  virtual uint32_t computeVelocityCommands(
      const ayanami::geometry_msgs::PoseStamped& pose,
      const ayanami::geometry_msgs::TwistStamped& velocity,
      ayanami::geometry_msgs::TwistStamped& cmd_vel,
      std::string& message) = 0;

  /// @brief 距离/角度容差内是否已到达目标。
  virtual bool isGoalReached(double dist_tol, double angle_tol) = 0;

  /// @brief 设置要跟踪的路径。
  virtual bool setPlan(const ayanami::nav_msgs::Path& plan) = 0;

  /// @brief 取消进行中的控制。默认不支持, 返回 false。
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
