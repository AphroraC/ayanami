#pragma once

#include <random>
#include <vector>

#include "swerve_kinematics.hpp"

namespace ayanami {
namespace modules {
namespace mppi {

using swerve::BodyTwist;
using swerve::RolloutState;
using swerve::Control4D;
using swerve::ControlLimits;
using swerve::SwerveGeometry;

/// @brief Planar pose used by rollouts and critics.
struct State {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// @brief Sampling-noise sigmas per control dimension (fl_steer, rr_steer,
/// fl_vel, rr_vel).
struct NoiseSigma {
  double steer{0.3};
  double wheel{2.0};
};

/// @brief MPPI solve settings.
struct Settings {
  int batch{600};            ///< sampled trajectories per solve
  int time_steps{20};        ///< rollout horizon length
  double dt{0.05};           ///< rollout step (seconds)
  double temperature{0.3};   ///< softmax temperature (smaller = greedier)
  int iterations{1};         ///< weighted-update iterations per solve
};

/// @brief A control sequence over the horizon (time_steps entries).
using ControlSequence = std::vector<Control4D>;

/// @brief Twist-space control (MPPI-H 3D mode): the chassis twist is
/// sampled directly -- valid when tracking error is small (steers roughly
/// aligned, steer lag negligible; the IROS2024 mode-switch premise).
struct Control3D {
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

/// @brief Sampling-noise sigmas for the 3D control space.
struct NoiseSigma3D {
  double vx{0.3};
  double vy{0.3};
  double omega{0.6};
};

/// @brief Which control space the current solve samples: Swerve4D (steer
/// rate-limited, feasible near obstacles / large errors) or Twist3D
/// (efficient precise tracking in free space).
enum class SolveMode { Swerve4D, Twist3D };

/// @brief MPPI-H mode gate (IROS2024 switching rule + obstacle veto):
/// Twist3D only when BOTH errors are inside their thresholds AND no
/// obstacle is near; every violation falls back to Swerve4D.
SolveMode selectSolveMode(double tracking_error, double yaw_error,
                          bool obstacle_nearby, double dist_threshold,
                          double yaw_threshold);

/// @brief One sampled rollout: states[0] is the start, size = time_steps+1.
struct Trajectory {
  std::vector<State> states;
  std::vector<BodyTwist> twists;  ///< twist executed at each step (states[t] -> states[t+1])
};

/// @brief Deterministic RNG holder: tests seed it, production uses the
/// default entropy seed. Not thread safe -- one controller worker thread
/// owns the solve.
class Sampler {
 public:
  explicit Sampler(unsigned int seed = std::random_device{}());
  /// @brief Fills `noise` with per-dimension gaussian noise (dims: steer,
  /// steer, wheel, wheel), sized batch * time_steps * 4.
  void fillNoise(std::vector<double>& noise, int batch, int time_steps,
                 const NoiseSigma& sigma);
  /// @brief Same for the 3D control space (dims: vx, vy, omega).
  void fillNoise3D(std::vector<double>& noise, int batch, int time_steps,
                   const NoiseSigma3D& sigma);
  std::mt19937& engine() { return engine_; }

 private:
  std::mt19937 engine_;
};

/// @brief Rollout of one control sequence from (start pose + current steer
/// state). omega_override forwards to rolloutStep (NaN = sampled omega).
/// Fills `out` in place (capacity reused across calls -- the batch loop
/// rolls hundreds of samples per solve).
void rolloutTrajectory(const State& start, double fl_steer, double rr_steer,
                       const ControlSequence& sequence,
                       const SwerveGeometry& geometry,
                       const ControlLimits& limits, double dt,
                       double omega_override, Trajectory& out);

/// @brief MPPI weighted update: weights = exp(-(cost - min_cost)/lambda),
/// normalized; out = sum_k weight_k * sample_k per (timestep, dim).
/// Sample sequences must already be clamped. Returns false only on a
/// degenerate temperature (<= 0) or empty inputs.
bool weightedUpdate(const ControlSequence& base,
                    const std::vector<ControlSequence>& samples,
                    const std::vector<double>& costs, double temperature,
                    ControlSequence& out);

/// @brief Same weighted update over the 3D control space.
bool weightedUpdate3D(const std::vector<Control3D>& base,
                      const std::vector<std::vector<Control3D>>& samples,
                      const std::vector<double>& costs, double temperature,
                      std::vector<Control3D>& out);

/// @brief Shifts `sequence` left by one step (warm start for the next
/// cycle): element 0 drops, the last element replicates.
void shiftSequence(ControlSequence& sequence);
void shiftSequence3D(std::vector<Control3D>& sequence);

/// @brief Clamps a 3D control into the twist-space limits.
Control3D clampControl3D(const Control3D& control, double max_vel_x,
                         double max_vel_y, double max_vel_theta);

/// @brief 3D rollout: the sampled twist integrates the pose directly (no
/// steer-lag modeling -- the mode's accepted simplification).
/// omega_override (NaN = sampled omega) keeps fixed/zero spin modes
/// consistent.
void rolloutTrajectory3D(const State& start,
                         const std::vector<Control3D>& sequence, double dt,
                         double omega_override, Trajectory& out);

}  // namespace mppi
}  // namespace modules
}  // namespace ayanami
