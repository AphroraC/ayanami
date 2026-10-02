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

#include <functional>
#include <string>

namespace neupan {

// Receives the library's warnings. The default prints to stderr; a host such
// as the ROS node routes them into its own logger. An empty handler restores
// the default.
using WarningHandler = std::function<void(const std::string&)>;
void setWarningHandler(WarningHandler handler);

void logWarning(const std::string& message);

}  // namespace neupan
