#pragma once

#include "ayanami/utility/extension_module.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <string>
#include <vector>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief 行为树扩展接口 (spec 要求: 动态装载对象直接/间接派生 ExtensionModule)。
 *
 * 导航相关的 BT 节点注册与默认树 XML 全部下沉到具体扩展 .so (如
 * navigation_behavior_tree), 使 BehaviorTreePilot 核心不含任何默认导航 XML 或
 * 策略: pilot 只装载并持有 BehaviorTreeExtension(s), 在 buildTree 时依次
 * registerNodes + registerTrees, 最后按配置的 tree_id createTree。
 *
 * 实现要点:
 * - registerNodes: 把自定义 BT 节点类型注册进 factory (pilot 树执行所需)。
 * - registerTrees: 把具名树 (XML) 注册进 factory, 返回已注册的树 id 列表。
 * - defaultTreeId: 配置未显式给 tree_id 时的回退树名。
 */
class BehaviorTreeExtension : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<BehaviorTreeExtension>;

  ~BehaviorTreeExtension() override = default;

  /// @brief 扩展名 (日志/调试)。
  virtual std::string name() const = 0;

  /// @brief 把本扩展实现的自定义节点注册进 factory。
  virtual void registerNodes(BT::BehaviorTreeFactory& factory) const = 0;

  /// @brief 把本扩展实现的树 (XML) 注册进 factory, 返回注册的树 id。
  virtual std::vector<std::string> registerTrees(
      BT::BehaviorTreeFactory& factory) const = 0;

  /// @brief 配置未显式给 tree_id 时的回退树名 (默认空 = 无回退)。
  virtual std::string defaultTreeId() const { return ""; }
};

}  // namespace behavior_tree
}  // namespace ayanami