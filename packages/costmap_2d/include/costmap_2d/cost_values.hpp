#pragma once

#include <cstdint>

namespace costmap_2d {

using CostValue = std::uint8_t;

constexpr CostValue FREE_SPACE = 0;
constexpr CostValue INSCRIBED_INFLATED_OBSTACLE = 253;
constexpr CostValue LETHAL_OBSTACLE = 254;
constexpr CostValue NO_INFORMATION = 255;

inline bool isLethal(CostValue cost) { return cost == LETHAL_OBSTACLE; }

}  // namespace costmap_2d
