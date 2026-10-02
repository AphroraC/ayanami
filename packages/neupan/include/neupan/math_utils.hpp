/*
 * neupan_cpp: C++ port of the NeuPAN planner.
 *
 * Ported from NeuPAN (https://github.com/hanruihua/NeuPAN),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <cmath>

namespace neupan {

inline constexpr double kPi = 3.14159265358979323846;

// Wraps an angle to [-pi, pi] (neupan.util.WrapToPi). std::remainder is exact
// and constant time for any input, where a subtract loop crawls on large
// angles and never terminates on inf; inf and NaN come back as NaN.
inline double wrapToPi(double rad) { return std::remainder(rad, 2.0 * kPi); }

}  // namespace neupan
