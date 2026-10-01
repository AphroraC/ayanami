#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <ayanami/messages/geometry_msgs.hpp>
#include <ayanami/messages/recovery_params.hpp>
#include <ayanami/rosemap/core/rosemap_recovery.hpp>
#include <ayanami/types.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief ESDF-gradient escape recovery (rosemap family; the
 * navi_minco_bit RecoverServer escape semantics rebuilt on the
 * start/tick/finish protocol).
 *
 * Pure C++ (zero ROS): on start() the robot pose is resolved through the
 * field module and the escape direction is picked as the local distance-
 * field uphill (the family's interpolated gradient). On a plateau
 * (gradient norm below min_gradient) the direction falls back to a
 * direction_count-way sweep keeping the farthest sample. Every tick
 * returns RUNNING carrying the suggested escape velocity (world-frame
 * direction rotated into the body frame; RecoveryExecution is the sole
 * publisher), re-sampling the direction every resample_interval ticks to
 * track field evolution. SUCCESS when the robot's clearance exceeds
 * exit_distance; FAILED (RecoveryFailed) after the tick budget -- the BT
 * recovery ladder moves on to the next recovery.
 */
class RosemapEscapeRecovery final : public ayanami::rosemap::RosemapRecovery {
 public:
  RosemapEscapeRecovery() = default;
  ~RosemapEscapeRecovery() override = default;

  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load, and by configure() for callers that
  /// build the object directly.
  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ModuleContext& context,
                 ayanami::rosemap::RosemapMapContext::ConstPtr map) override;

  void setCyclePeriod(double seconds) override;

  void start() override;
  ResultT<void> tick(RecoveryCycleData& cycle) override;
  void finish() override;

  bool cancel() override;

 private:
  struct RosemapEscapeRecoveryParam {
    /// @brief Escape speed fed to the execution layer (m/s).
    double escape_speed = 0.4;
    /// @brief SUCCESS gate: clearance above this counts as escaped (m).
    double exit_distance = 0.4;
    /// @brief Gradient norm below this reads as a plateau -> sweep.
    double min_gradient = 1e-3;
    /// @brief Sweep resolution on a plateau (directions).
    int direction_count = 36;
    /// @brief Sweep sample radius (m).
    double sweep_radius = 0.4;
    /// @brief Re-pick the direction every N ticks (field evolution).
    int resample_interval = 10;
    /// @brief Tick budget (ticks; exceeded -> FAILED).
    int max_ticks = 600;
  };

  RosemapEscapeRecoveryParam param;

  /// @brief Pick the escape direction at the current pose (uphill gradient
  /// with the plateau sweep fallback).
  bool pickDirection(const Eigen::Vector2d& position,
                     Eigen::Vector2d& direction) const;

  ayanami::rosemap::RosemapMapContext::ConstPtr map_context;

  /// @brief Tick cadence pushed by the execution before each run (s).
  double cycle_dt{0.01};
  std::atomic<bool> canceled{false};
  Eigen::Vector2d escape_direction{1.0, 0.0};
  int ticks_remaining{0};
  int ticks_to_resample{0};
};

}  // namespace modules
}  // namespace ayanami
