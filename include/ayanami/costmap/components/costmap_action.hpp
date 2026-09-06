#pragma once

// costmap 族 Action 聚合头 (生产路径): CostmapNavigationServer
// 的 createActions 直接构造这些带地图的 Action, 取代基类 new*Execution
// 虚工厂间接层。
#include "ayanami/costmap/components/costmap_controller_action.hpp"
#include "ayanami/costmap/components/costmap_planner_action.hpp"
#include "ayanami/costmap/components/costmap_recovery_action.hpp"
#include "ayanami/costmap/components/costmap_smoother_action.hpp"