#include "mppi_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ayanami {
namespace modules {
namespace mppi {

namespace {
constexpr int k_dims = 4;
}

Sampler::Sampler(unsigned int seed) : engine_(seed) {}

void Sampler::fillNoise(std::vector<double>& noise, int batch,
                        int time_steps, const NoiseSigma& sigma) {
  const double sigma_table[k_dims] = {sigma.steer, sigma.steer, sigma.wheel,
                                      sigma.wheel};
  noise.resize(static_cast<std::size_t>(batch) * time_steps * k_dims);
  // One distribution per dimension keeps the per-dim sigma semantics
  // explicit; the engine is shared so the stream stays deterministic under
  // a fixed seed regardless of the sampling order.
  std::normal_distribution<double> distributions[k_dims] = {
      std::normal_distribution<double>(0.0, sigma_table[0]),
      std::normal_distribution<double>(0.0, sigma_table[1]),
      std::normal_distribution<double>(0.0, sigma_table[2]),
      std::normal_distribution<double>(0.0, sigma_table[3])};
  for (int k = 0; k < batch; ++k) {
    for (int t = 0; t < time_steps; ++t) {
      for (int dim = 0; dim < k_dims; ++dim) {
        noise[(static_cast<std::size_t>(k) * time_steps + t) * k_dims + dim] =
            distributions[dim](engine_);
      }
    }
  }
}

void Sampler::fillNoise3D(std::vector<double>& noise, int batch,
                          int time_steps, const NoiseSigma3D& sigma) {
  const double sigma_table[3] = {sigma.vx, sigma.vy, sigma.omega};
  noise.resize(static_cast<std::size_t>(batch) * time_steps * 3);
  std::normal_distribution<double> distributions[3] = {
      std::normal_distribution<double>(0.0, sigma_table[0]),
      std::normal_distribution<double>(0.0, sigma_table[1]),
      std::normal_distribution<double>(0.0, sigma_table[2])};
  for (int k = 0; k < batch; ++k) {
    for (int t = 0; t < time_steps; ++t) {
      for (int dim = 0; dim < 3; ++dim) {
        noise[(static_cast<std::size_t>(k) * time_steps + t) * 3 + dim] =
            distributions[dim](engine_);
      }
    }
  }
}

void rolloutTrajectory(const State& start, double fl_steer, double rr_steer,
                       const ControlSequence& sequence,
                       const SwerveGeometry& geometry,
                       const ControlLimits& limits, double dt,
                       double omega_override, Trajectory& trajectory) {
  trajectory.states.clear();
  trajectory.twists.clear();
  RolloutState current;
  current.x = start.x;
  current.y = start.y;
  current.yaw = start.yaw;
  current.fl_steer = fl_steer;
  current.rr_steer = rr_steer;
  trajectory.states.push_back(State{current.x, current.y, current.yaw});
  for (const Control4D& target : sequence) {
    BodyTwist executed;
    current = rolloutStep(current, target, geometry, limits, dt,
                          omega_override, executed);
    trajectory.states.push_back(State{current.x, current.y, current.yaw});
    trajectory.twists.push_back(executed);
  }
}

void shiftSequence(ControlSequence& sequence) {
  if (sequence.empty()) {
    return;
  }
  for (std::size_t t = 1; t < sequence.size(); ++t) {
    sequence[t - 1] = sequence[t];
  }
  sequence.back() = sequence[sequence.size() >= 2 ? sequence.size() - 2 : 0];
}

SolveMode selectSolveMode(double tracking_error, double yaw_error,
                          bool obstacle_nearby, double dist_threshold,
                          double yaw_threshold) {
  if (obstacle_nearby) {
    return SolveMode::Swerve4D;
  }
  if (tracking_error < dist_threshold && yaw_error < yaw_threshold) {
    return SolveMode::Twist3D;
  }
  return SolveMode::Swerve4D;
}

Control3D clampControl3D(const Control3D& control, double max_vel_x,
                         double max_vel_y, double max_vel_theta) {
  Control3D clamped;
  clamped.vx = std::clamp(control.vx, -max_vel_x, max_vel_x);
  clamped.vy = std::clamp(control.vy, -max_vel_y, max_vel_y);
  clamped.omega = std::clamp(control.omega, -max_vel_theta, max_vel_theta);
  return clamped;
}

void rolloutTrajectory3D(const State& start,
                         const std::vector<Control3D>& sequence, double dt,
                         double omega_override, Trajectory& trajectory) {
  constexpr double k_two_pi = 6.283185307179586476925286766559;
  trajectory.states.clear();
  trajectory.twists.clear();
  trajectory.states.reserve(sequence.size() + 1);
  trajectory.twists.reserve(sequence.size());
  State current = start;
  trajectory.states.push_back(current);
  for (const Control3D& target : sequence) {
    BodyTwist executed{target.vx, target.vy, target.omega};
    if (!std::isnan(omega_override)) {
      executed.omega = omega_override;
    }
    const double cos_yaw = std::cos(current.yaw);
    const double sin_yaw = std::sin(current.yaw);
    current.x += (cos_yaw * executed.vx - sin_yaw * executed.vy) * dt;
    current.y += (sin_yaw * executed.vx + cos_yaw * executed.vy) * dt;
    current.yaw =
        std::remainder(current.yaw + executed.omega * dt, k_two_pi);
    trajectory.states.push_back(current);
    trajectory.twists.push_back(executed);
  }
}

void shiftSequence3D(std::vector<Control3D>& sequence) {
  if (sequence.empty()) {
    return;
  }
  for (std::size_t t = 1; t < sequence.size(); ++t) {
    sequence[t - 1] = sequence[t];
  }
  sequence.back() = sequence[sequence.size() >= 2 ? sequence.size() - 2 : 0];
}

// Shared softmax core over any control element type: the weighted average
// is element-wise, so the sequence types only differ in their dimension
// count and field layout.
template <typename ControlT>
bool weightedUpdateImpl(const std::vector<ControlT>& base,
                        const std::vector<std::vector<ControlT>>& samples,
                        const std::vector<double>& costs, double temperature,
                        std::vector<ControlT>& out) {
  if (samples.empty() || costs.size() != samples.size() ||
      base.empty() || base.size() != samples.front().size() ||
      !(temperature > 0.0)) {
    return false;
  }
  // SCALE-FREE SOFTMAX: lambda = temperature x (90th-percentile spread of
  // the finite costs above the min). A fixed absolute lambda collapses
  // into a hard argmin when the critic scale outgrows it -- the weighted
  // average degenerates to the single best sample (or the anchor) and the
  // solve freezes at a fixed point.
  constexpr double k_rejection_scale = 1e5;
  std::vector<double> finite;
  finite.reserve(costs.size());
  for (const double cost : costs) {
    if (std::isfinite(cost) && cost < k_rejection_scale) {
      finite.push_back(cost);
    }
  }
  if (finite.empty()) {
    return false;
  }
  std::sort(finite.begin(), finite.end());
  const double min_cost = finite.front();
  const double percentile = finite[static_cast<std::size_t>(
      0.9 * static_cast<double>(finite.size() - 1))];
  const double spread = std::max(percentile - min_cost, 1e-6);
  const double lambda = temperature * spread;

  std::vector<double> weights(costs.size());
  double weight_sum = 0.0;
  for (std::size_t k = 0; k < costs.size(); ++k) {
    const bool usable =
        std::isfinite(costs[k]) && costs[k] < k_rejection_scale;
    const double shifted = usable ? (costs[k] - min_cost) / lambda
                                  : std::numeric_limits<double>::infinity();
    const double weight = std::isfinite(shifted) ? std::exp(-shifted) : 0.0;
    weights[k] = weight;
    weight_sum += weight;
  }
  if (!(weight_sum > 0.0)) {
    return false;
  }
  const int dims = static_cast<int>(sizeof(ControlT) / sizeof(double));
  out.assign(base.size(), ControlT{});
  std::vector<double> acc(base.size() * dims, 0.0);
  for (std::size_t k = 0; k < samples.size(); ++k) {
    const double weight = weights[k] / weight_sum;
    if (weight == 0.0) {
      continue;
    }
    for (std::size_t t = 0; t < base.size(); ++t) {
      const double* fields = reinterpret_cast<const double*>(&samples[k][t]);
      for (int d = 0; d < dims; ++d) {
        acc[t * dims + d] += weight * fields[d];
      }
    }
  }
  for (std::size_t t = 0; t < base.size(); ++t) {
    double* fields = reinterpret_cast<double*>(&out[t]);
    for (int d = 0; d < dims; ++d) {
      fields[d] = acc[t * dims + d];
    }
  }
  return true;
}

bool weightedUpdate(const ControlSequence& base,
                    const std::vector<ControlSequence>& samples,
                    const std::vector<double>& costs, double temperature,
                    ControlSequence& out) {
  return weightedUpdateImpl(base, samples, costs, temperature, out);
}

bool weightedUpdate3D(const std::vector<Control3D>& base,
                      const std::vector<std::vector<Control3D>>& samples,
                      const std::vector<double>& costs, double temperature,
                      std::vector<Control3D>& out) {
  return weightedUpdateImpl(base, samples, costs, temperature, out);
}

}  // namespace mppi
}  // namespace modules
}  // namespace ayanami
