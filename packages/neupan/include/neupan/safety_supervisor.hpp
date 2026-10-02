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

#include <optional>

#include "neupan/neupan_planner.hpp"
#include "neupan/types.hpp"

namespace neupan {

// Safety policy between NeuPANPlanner and the chassis, kept free of any
// middleware so it can be unit tested. It covers what the planner cannot
// see from one frame: obstacle data that stopped arriving, a solver that
// keeps failing (driving blind on a nominal with no avoidance), and a stall
// (idling short of the goal with neither stop nor arrive set). Times are
// seconds on any monotonic clock.
class SafetySupervisor {
 public:
  struct Options {
    double scan_timeout = 0.5;   // obstacle data older than this is unknown
    int solver_fail_grace = 5;   // unsolved cycles tolerated before stopping
    double stall_speed = 0.02;   // |v| and |w| at or below this count as idle
    double stall_timeout = 3.0;  // idle this long without arriving = stalled
  };

  enum class Level { Ok, Warn, Error };
  struct Status {
    Level level;
    const char* message;
  };

  explicit SafetySupervisor(const Options& opts);

  void recordScan(double now) { last_scan_ = now; }
  // False until the first scan: with no scan at all the planner tracks the
  // path alone, which the host reports separately.
  bool scanStale(double now) const {
    return last_scan_ && now - *last_scan_ > opts_.scan_timeout;
  }

  // Books one planning cycle and returns the command that may be sent: the
  // planner's action, or zero when it stopped, arrived, or the solver has
  // failed for more than solver_fail_grace cycles in a row.
  Vec2 filter(const NeuPANPlanner::Info& info, const Vec2& action, double now);

  bool solverGaveUp() const {
    return consecutive_unsolved_ > opts_.solver_fail_grace;
  }
  int consecutiveUnsolved() const { return consecutive_unsolved_; }
  bool stalled() const { return stalled_; }

  // Summary of the last cycle for diagnostics.
  Status status(const NeuPANPlanner::Info& info) const;

 private:
  void updateStall(const Vec2& cmd, bool arrived, double now);

  Options opts_;
  std::optional<double> last_scan_;
  std::optional<double> last_progress_;
  int consecutive_unsolved_ = 0;
  bool stalled_ = false;
};

}  // namespace neupan
