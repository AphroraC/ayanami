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

#include <memory>
#include <vector>

#include "neupan/math_utils.hpp"
#include "neupan/robot.hpp"
#include "neupan/types.hpp"

namespace neupan {

// Port of neupan/blocks/initial_path.py restricted to curve_style == "line"
// (each path point is [x, y, theta, gear], gear always 1 for line paths).
// Method names follow upstream so the two can be read side by side.
class InitialPath {
 public:
  struct Options {
    std::vector<Vec3> waypoints;
    bool loop = false;
    double interval = -1.0;  // <= 0: default dt * ref_speed
    double arrive_threshold = 0.1;
    double close_threshold = 0.1;
    int ind_range = 10;
    int arrive_index_threshold = 1;
  };

  using PathPoint = Eigen::Vector4d;  // x, y, theta, gear

  // ref_speed only sets the default point spacing; the speed actually
  // tracked is passed to generateNomRefState each cycle.
  InitialPath(std::shared_ptr<const Robot> robot, double ref_speed,
              const Options& opts);

  // neupan.forward step 1: advances the progress index to the point closest
  // to `state` and returns true once the path end is reached. Arrival latches
  // until a new path is installed or reset() is called. Builds the path from
  // `state` and the configured waypoints first if none is set.
  bool checkArrive(const Vec3& state);

  NomRef generateNomRefState(const Vec3& state, const Mat2X& cur_vel_array,
                             double ref_speed) const;

  // Path setters. All of them restart progress at the path start, except
  // replaceInitialPath, which re-projects `state` to keep it.
  void setIpathWithState(const Vec3& state);  // state + configured waypoints
  void setIpathWithWaypoints(const std::vector<Vec3>& waypoints);
  void updateInitialPathFromGoal(const Vec3& start, const Vec3& goal);
  // Port of set_initial_path: installs an external path, re-densified to the
  // point spacing.
  void setInitialPath(std::vector<PathPoint> path);
  void replaceInitialPath(std::vector<PathPoint> path, const Vec3& state);
  void reset();

  bool hasPath() const { return !curve_list_.empty(); }
  bool hasConfiguredWaypoints() const { return !opts_.waypoints.empty(); }
  const std::vector<PathPoint>& initialPath() const { return initial_path_; }
  // Incremented whenever a new path is installed, so observers can tell when
  // to republish it.
  int revision() const { return revision_; }
  bool arrived() const { return arrive_flag_; }

 private:
  std::vector<PathPoint> generateLineCurve(
      const std::vector<Vec3>& waypoints) const;
  void setPath(std::vector<PathPoint> path);
  void splitPathWithGear();
  // Moves point_index_ to the closest point within ind_range ahead.
  void advanceToClosestPoint(const Vec3& state);
  bool checkCurveArrive(const Vec3& state) const;
  // Returns intersection of circle (ref xy, radius=length) with the curve,
  // advancing ref_index; mirrors find_interaction_point.
  Vec3 findInteractionPoint(const Vec3& ref_state, int& ref_index,
                            double length) const;

  const std::vector<PathPoint>& curCurve() const {
    return curve_list_[curve_index_];
  }

  std::shared_ptr<const Robot> robot_;
  Options opts_;
  double interval_;  // point spacing of generated / re-densified paths

  std::vector<PathPoint> initial_path_;
  std::vector<std::vector<PathPoint>> curve_list_;
  int revision_ = 0;

  // progress along the path, updated by checkArrive
  int curve_index_ = 0;
  int point_index_ = 0;
  bool arrive_flag_ = false;
};

}  // namespace neupan
