// 模板 dlopen 加载路径的显式实例化 。
//
// PluginManager<T> 方法在 plugin_manager.hpp 内联定义; 此处为 的
// 4 个导航插件抽象做显式实例化, 把 dlopen/dlsym 加载路径预编译进 libayanami,
// 避免每次编译整块模板展开。新增 PluginType 使用点需在此补实例化。

#include "ayanami/abstract/components/plugin_manager.hpp"

#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/abstract/core/abstract_goal_checker.hpp"
#include "ayanami/abstract/core/abstract_planner.hpp"
#include "ayanami/abstract/core/abstract_progress_checker.hpp"
#include "ayanami/abstract/core/abstract_recovery.hpp"
#include "ayanami/abstract/core/abstract_smoother.hpp"

namespace ayanami {
namespace components {

template class PluginManager<ayanami::abstract::AbstractPlanner>;
template class PluginManager<ayanami::abstract::AbstractSmoother>;
template class PluginManager<ayanami::abstract::AbstractController>;
template class PluginManager<ayanami::abstract::AbstractRecovery>;
template class PluginManager<ayanami::abstract::AbstractGoalChecker>;
template class PluginManager<ayanami::abstract::AbstractProgressChecker>;

} // namespace components
} // namespace ayanami
