#pragma once

// costmap family Action aggregate header (production path):
// CostmapNavigationServer's createActions constructs these map-aware
// Actions directly, replacing the base class's new*Execution virtual
// factory indirection layer.
#include "ayanami/costmap/components/costmap_controller_action.hpp"
#include "ayanami/costmap/components/costmap_planner_action.hpp"
#include "ayanami/costmap/components/costmap_recovery_action.hpp"
#include "ayanami/costmap/components/costmap_smoother_action.hpp"