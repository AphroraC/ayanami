// Explicit instantiations for the template dlopen loading path.
//
// ModuleManager<T> methods are defined inline in module_manager.hpp; this
// file provides explicit instantiations for the
// 4 navigation module abstractions, precompiling the dlopen/dlsym loading
// path into libayanami to avoid re-expanding the whole template on every
// build. New ModuleType use sites must add their instantiation here.

#include "ayanami/abstract/components/module_manager.hpp"

#include "ayanami/abstract/core/abstract_controller.hpp"
#include "ayanami/abstract/core/abstract_goal_checker.hpp"
#include "ayanami/abstract/core/abstract_planner.hpp"
#include "ayanami/abstract/core/abstract_progress_checker.hpp"
#include "ayanami/abstract/core/abstract_recovery.hpp"
#include "ayanami/abstract/core/abstract_smoother.hpp"

namespace ayanami {
namespace components {

template class ModuleManager<ayanami::abstract::AbstractPlanner>;
template class ModuleManager<ayanami::abstract::AbstractSmoother>;
template class ModuleManager<ayanami::abstract::AbstractController>;
template class ModuleManager<ayanami::abstract::AbstractRecovery>;
template class ModuleManager<ayanami::abstract::AbstractGoalChecker>;
template class ModuleManager<ayanami::abstract::AbstractProgressChecker>;

} // namespace components
} // namespace ayanami
