// path_ops.hpp — 迁移自 nav_2d_utils/path_ops.hpp (纯算法)。
#pragma once

#include "dwb_types.hpp"

namespace dwb_core {

/**
 * @brief Insert extra points along the path to meet the given resolution
 * @param global_plan_in Input path
 * @param resolution The minimum spacing between two consecutive points
 * @return New path with possibly extra points inserted
 */
nav_2d_msgs::Path2D adjustPlanResolution(
    const nav_2d_msgs::Path2D& global_plan_in, double resolution);

}  // namespace dwb_core
