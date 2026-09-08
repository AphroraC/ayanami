// Explicit instantiations for the template dlopen loading path.
//
// PluginManager<T> methods are defined inline in plugin_manager.hpp; this
// file provides explicit instantiations for the
// 4 navigation plugin abstractions, precompiling the dlopen/dlsym loading
// path into libayanami to avoid re-expanding the whole template on every
// build. New PluginType use sites must add their instantiation here.

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
