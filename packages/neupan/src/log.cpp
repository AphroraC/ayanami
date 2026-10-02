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

#include "neupan/log.hpp"

#include <cstdio>
#include <mutex>
#include <utility>

namespace neupan {

namespace {

std::mutex& handlerMutex() {
  static std::mutex m;
  return m;
}

WarningHandler& handler() {
  static WarningHandler h;
  return h;
}

}  // namespace

void setWarningHandler(WarningHandler h) {
  const std::lock_guard<std::mutex> lock(handlerMutex());
  handler() = std::move(h);
}

void logWarning(const std::string& message) {
  const std::lock_guard<std::mutex> lock(handlerMutex());
  if (handler())
    handler()(message);
  else
    std::fprintf(stderr, "[neupan] WARNING: %s\n", message.c_str());
}

}  // namespace neupan
