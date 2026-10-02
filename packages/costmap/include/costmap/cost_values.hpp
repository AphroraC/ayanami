#pragma once

#include <cstdint>

namespace costmap_2d {

using CostValue = std::uint8_t;

constexpr CostValue FREE_SPACE = 0;
constexpr CostValue INSCRIBED_INFLATED_OBSTACLE = 253;
constexpr CostValue LETHAL_OBSTACLE = 254;
constexpr CostValue NO_INFORMATION = 255;

// 253 (inscribed inflated) already counts as lethal, so the inflation band is not mistaken for non-lethal.
inline bool isLethal(CostValue cost) { return cost >= INSCRIBED_INFLATED_OBSTACLE; }

}  // namespace costmap_2d
