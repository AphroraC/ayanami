#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief 局部控制器插件抽象 (MBF 式分层)。
 *
 * 输入当前位姿/速度, 输出单周期速度命令; 待执行路径经 setPlan 接收。
 * 到达判定由 execution 层经 goal_checker 完成 (控制器自身不判断是否到达)。
 * 零地图类型: 接口不包含任何地图参数。
 */
class AbstractController : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractController>;

  virtual ~AbstractController() = default;

  /// @brief 计算单周期速度命令 (纯虚)。
  /// @return ControllerResultT: ok=true 携带 ControllerCommand; 失败携带
  ///         ErrorCode + 诊断 (仅诊断, 决策只看 ErrorCode)。
  virtual ControllerResultT computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped& pose,
      const ayanami::geometry_msgs::TwistStamped& velocity) = 0;

  /// @brief 设置要跟踪的路径。
  virtual bool setPlan(const ayanami::nav_msgs::Path& plan) = 0;

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief 取消进行中的控制 (协作式中断; 默认不支持)。
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
