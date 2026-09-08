// alignment_util.hpp -- ported from dwb_critics/alignment_util.hpp.
#pragma once

#include "../dwb_types.hpp"

namespace dwb_critics {
/**
 * @brief Projects the given pose forward the specified distance in the x
 * direction.
 */
geometry_msgs::Pose2D getForwardPose(const geometry_msgs::Pose2D& pose,
                                     double distance);

}  // namespace dwb_critics
