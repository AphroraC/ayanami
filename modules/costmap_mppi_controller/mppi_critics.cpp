#include "mppi_critics.hpp"

#include <algorithm>
#include <cmath>

namespace ayanami {
namespace modules {
namespace mppi {
namespace critics {

namespace {
constexpr double k_inscribed = 253.0;
constexpr double k_lethal_rejection = 1e6;
constexpr int k_cursor_window = 6;

double pointDistance(double ax, double ay, double bx, double by) {
  return std::hypot(bx - ax, by - ay);
}
}  // namespace

double pathFollow(const Trajectory& trajectory, const CriticData& data) {
  if (data.path == nullptr || data.path->size() < 2) {
    return 0.0;
  }
  const auto& path = *data.path;
  std::size_t cursor = std::min(data.path_cursor, path.size() - 1);
  double cost = 0.0;
  // Reorientation grace: with steer-lag kinematics, changing direction
  // transiently LEAVES the path before rejoining it; a flat linear
  // distance penalty then prefers standing still forever (nav2's omni
  // model has no lag and never meets this local minimum). The per-state
  // weight ramps in over the first k_reorient_steps so reorientation is
  // affordable while sustained deviation is not.
  constexpr int k_reorient_steps = 8;
  int step_index = -1;
  for (const State& state : trajectory.states) {
    ++step_index;
    // Monotone nearest-point within a bounded forward window: the cursor
    // follows the trajectory down the path and never rewinds, which is
    // what makes path progress profitable.
    const std::size_t window_end =
        std::min(path.size() - 1, cursor + k_cursor_window);
    double best =
        pointDistance(state.x, state.y, path[cursor].x(), path[cursor].y());
    std::size_t best_index = cursor;
    for (std::size_t probe = cursor + 1; probe <= window_end; ++probe) {
      const double distance =
          pointDistance(state.x, state.y, path[probe].x(), path[probe].y());
      if (distance < best) {
        best = distance;
        best_index = probe;
      }
    }
    cursor = best_index;
    const double weight =
        std::min(1.0, static_cast<double>(step_index) / k_reorient_steps);
    cost += weight * best;
  }
  return cost * data.w_path_follow;
}

double goal(const Trajectory& trajectory, const CriticData& data) {
  if (trajectory.states.empty()) {
    return 0.0;
  }
  const State& final_state = trajectory.states.back();
  return pointDistance(final_state.x, final_state.y, data.goal_xy.x(),
                       data.goal_xy.y()) *
         data.w_goal;
}

double goalAngle(const Trajectory& trajectory, const CriticData& data) {
  if (!data.angular_free || trajectory.states.empty()) {
    return 0.0;
  }
  const State& final_state = trajectory.states.back();
  if (pointDistance(final_state.x, final_state.y, data.goal_xy.x(),
                    data.goal_xy.y()) > data.goal_angle_radius) {
    return 0.0;
  }
  const double yaw_error = std::abs(std::remainder(
      final_state.yaw - data.goal_yaw, 6.283185307179586476925286766559));
  return yaw_error * data.w_goal_angle;
}

double twirling(const Trajectory& trajectory, const CriticData& data) {
  if (!data.angular_free) {
    return 0.0;
  }
  double cost = 0.0;
  for (const BodyTwist& twist : trajectory.twists) {
    cost += twist.omega * twist.omega;
  }
  return cost * data.w_twirling;
}

double constraint(const Trajectory& trajectory, const CriticData& data) {
  auto penalty = [](double value, double limit) {
    const double excess = std::abs(value) - limit;
    return excess > 0.0 ? excess * excess : 0.0;
  };
  double cost = 0.0;
  for (const BodyTwist& twist : trajectory.twists) {
    cost += penalty(twist.vx, data.max_vel_x);
    cost += penalty(twist.vy, data.max_vel_y);
    cost += penalty(twist.omega, data.max_vel_theta);
  }
  return cost * data.w_constraint;
}

double velocityDeadband(const Trajectory& trajectory, const CriticData& data) {
  auto penalty = [&](double value, double deadband) {
    const double magnitude = std::abs(value);
    if (magnitude <= 0.0 || magnitude >= deadband) {
      return 0.0;
    }
    // Zero at both ends of (0, deadband): the optimum sits either fully
    // stopped or clearly outside the dead zone.
    const double normalized = magnitude / deadband;
    return 4.0 * normalized * (1.0 - normalized);
  };
  double cost = 0.0;
  for (const BodyTwist& twist : trajectory.twists) {
    cost += penalty(twist.vx, data.deadband_xy);
    cost += penalty(twist.vy, data.deadband_xy);
    cost += penalty(twist.omega, data.deadband_theta);
  }
  return cost * data.w_velocity_deadband;
}

double obstacles(const Trajectory& trajectory, const CriticData& data) {
  if (!data.cost_at) {
    return 0.0;
  }
  const bool has_footprint = !data.footprint_offsets.empty();
  double cost = 0.0;
  // Segment CENTER midpoints close the sub-step gap: rollout states are
  // ~0.1 m apart at speed, and a single-cell obstacle slips between two
  // discrete state queries otherwise. With a footprint the consecutive
  // footprint sweeps already overlap along the segment, so midpoints stay
  // center-only.
  auto lethalAt = [&](double wx, double wy) {
    return data.cost_at(wx, wy) >= k_inscribed;
  };
  auto gradedAt = [&](double wx, double wy) {
    return static_cast<double>(data.cost_at(wx, wy));
  };
  // Worst graded cost over the footprint (or the bare center when no
  // footprint): MAX, not sum -- the corners double-counting one inflated
  // cell would skew the field.
  auto stateCost = [&](const State& state) -> bool {
    if (!has_footprint) {
      if (lethalAt(state.x, state.y)) {
        return true;
      }
      cost += gradedAt(state.x, state.y);
      return false;
    }
    const double cos_yaw = std::cos(state.yaw);
    const double sin_yaw = std::sin(state.yaw);
    double worst = 0.0;
    for (const auto& offset : data.footprint_offsets) {
      const double wx = state.x + cos_yaw * offset.x() - sin_yaw * offset.y();
      const double wy = state.y + sin_yaw * offset.x() + cos_yaw * offset.y();
      if (lethalAt(wx, wy)) {
        return true;
      }
      worst = std::max(worst, gradedAt(wx, wy));
    }
    cost += worst;
    return false;
  };
  for (std::size_t t = 0; t < trajectory.states.size(); ++t) {
    if (stateCost(trajectory.states[t])) {
      // Rejection-scale: the softmax then gives this sample zero weight.
      return k_lethal_rejection;
    }
    if (t + 1 < trajectory.states.size()) {
      const State mid{
          0.5 * (trajectory.states[t].x + trajectory.states[t + 1].x),
          0.5 * (trajectory.states[t].y + trajectory.states[t + 1].y), 0.0};
      if (lethalAt(mid.x, mid.y)) {
        return k_lethal_rejection;
      }
      if (!has_footprint) {
        cost += gradedAt(mid.x, mid.y);
      }
    }
  }
  return cost * data.w_obstacles;
}

double totalCost(const Trajectory& trajectory, const CriticData& data) {
  double cost = pathFollow(trajectory, data);
  cost += goal(trajectory, data);
  cost += goalAngle(trajectory, data);
  cost += twirling(trajectory, data);
  cost += constraint(trajectory, data);
  cost += velocityDeadband(trajectory, data);
  const double obstacle_cost = obstacles(trajectory, data);
  if (obstacle_cost >= k_lethal_rejection) {
    return k_lethal_rejection;
  }
  cost += obstacle_cost;
  return cost;
}

}  // namespace critics
}  // namespace mppi
}  // namespace modules
}  // namespace ayanami
