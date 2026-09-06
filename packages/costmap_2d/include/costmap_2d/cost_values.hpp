#pragma once

#include <cstdint>

namespace costmap_2d {

using CostValue = std::uint8_t;

constexpr CostValue FREE_SPACE = 0;
constexpr CostValue INSCRIBED_INFLATED_OBSTACLE = 253;
constexpr CostValue LETHAL_OBSTACLE = 254;
constexpr CostValue NO_INFORMATION = 255;

// 253 (inscribed inflated) 起判为 lethal, 避免膨胀带被误判为非致命。
inline bool isLethal(CostValue cost) { return cost >= INSCRIBED_INFLATED_OBSTACLE; }

}  // namespace costmap_2d
