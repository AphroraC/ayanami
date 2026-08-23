#pragma once

#include "ayanami/abstract/core/abstract_goal_checker.hpp"
#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 局部控制器插件抽象 (spec §5.1)。
 *
 * 输入当前位姿/速度, 输出 cmd_vel; 接收待执行路径。到达判定由宿主经
 * goal_checker 完成 (控制器自身不再判断是否到达目标)。
 * 零地图类型 (地图经 AbstractMapContext 注入)。
 */
class AbstractController {
public:
  using Ptr = std::shared_ptr<AbstractController>;

  virtual ~AbstractController() = default;

  /**
   * @brief 计算速度命令。
   * @param goal_checker  只读目标检查插件 (只读: 控制器可用 getTolerances
   *                      取容差, 不调用其状态化判定); 空指针需防御。
   * @param[out] cmd_vel  体坐标系速度命令。
   * @param[out] message  失败时的人类可读诊断 (仅诊断, 决策只看 failure)。
   * @return FailureCode: None=成功, 其余为统一失败码。
   */
  virtual FailureCode computeVelocityCommands(
      const ayanami::geometry_msgs::PoseStamped& pose,
      const ayanami::geometry_msgs::TwistStamped& velocity,
      ayanami::geometry_msgs::TwistStamped& cmd_vel,
      std::string& message,
      const ayanami::abstract::AbstractGoalChecker::Ptr& goal_checker) = 0;

  /// @brief 设置要跟踪的路径。
  virtual bool setPlan(const ayanami::nav_msgs::Path& plan) = 0;

  /// @brief 取消进行中的控制。默认不支持, 返回 false。
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
