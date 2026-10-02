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

#include "neupan/safety_supervisor.hpp"

#include <stdexcept>

namespace neupan {

SafetySupervisor::SafetySupervisor(const Options& opts) : opts_(opts) {
  if (!(opts.scan_timeout > 0.0) || !(opts.stall_timeout > 0.0) ||
      !(opts.stall_speed >= 0.0) || opts.solver_fail_grace < 0)
    throw std::invalid_argument(
        "safety supervisor: timeouts must be > 0, stall_speed and "
        "solver_fail_grace >= 0");
}

Vec2 SafetySupervisor::filter(const NeuPANPlanner::Info& info,
                              const Vec2& action, double now) {
  // Arrival skips the solve, so it says nothing about the solver. A stop
  // cycle did solve and counts like any other.
  if (!info.arrive)
    consecutive_unsolved_ = info.solved ? 0 : consecutive_unsolved_ + 1;

  const bool hold = info.stop || info.arrive || solverGaveUp();
  const Vec2 cmd = hold ? Vec2::Zero() : action;
  updateStall(cmd, info.arrive, now);
  return cmd;
}

void SafetySupervisor::updateStall(const Vec2& cmd, bool arrived, double now) {
  const bool moving = (cmd.array().abs() > opts_.stall_speed).any();
  // The first planned cycle starts the clock.
  if (moving || arrived || !last_progress_) {
    last_progress_ = now;
    stalled_ = false;
  } else if (now - *last_progress_ > opts_.stall_timeout) {
    stalled_ = true;
  }
}

SafetySupervisor::Status SafetySupervisor::status(
    const NeuPANPlanner::Info& info) const {
  if (info.arrive) return {Level::Ok, "arrived"};  // arrival skips the solve
  if (stalled_) return {Level::Error, "stalled"};
  if (solverGaveUp()) return {Level::Error, "solver failing"};
  if (info.stop) return {Level::Warn, "stopped at collision threshold"};
  if (!info.solved) return {Level::Warn, "unsolved"};
  return {Level::Ok, "tracking"};
}

}  // namespace neupan
