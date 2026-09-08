// geometry_utils.hpp -- pure geometry functions used by DWB from nav2_util/geometry_utils.hpp
// (euclideanDistance / first_after_integrated_distance, ported from nav2).
#pragma once

#include <cmath>

#include "dwb_types.hpp"

namespace dwb_core {
namespace geometry_utils {

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Pose2D
 */
inline double euclideanDistance(const geometry_msgs::Pose2D& pos1,
                                 const geometry_msgs::Pose2D& pos2) {
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;
  return std::hypot(dx, dy);
}

/**
 * @brief Find first element in iterator that is greater integrated distance
 *        than comparevalue
 */
template <typename Iter, typename Getter>
inline Iter first_after_integrated_distance(Iter begin, Iter end,
                                            Getter getCompareVal) {
  if (begin == end) {
    return end;
  }
  Getter dist = 0.0;
  for (Iter it = begin; it != end - 1; it++) {
    dist += euclideanDistance(*it, *(it + 1));
    if (dist > getCompareVal) {
      return it + 1;
    }
  }
  return end;
}

}  // namespace geometry_utils
}  // namespace dwb_core
