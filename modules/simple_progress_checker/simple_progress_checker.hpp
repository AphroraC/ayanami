#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <ayanami/abstract/core/abstract_progress_checker.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief Simple progress checker module (modeled on MBF SimpleProgressChecker).
 *
 * Pure C++ (zero ROS): tracks a baseline pose and timestamp. Within time_period, if the moved distance >=
 * required_movement or the rotated angle >= movement_angle_required, the baseline is updated and progress is declared;
 * no movement within the time window is judged as no progress (stuck).
 */
class SimpleProgressChecker final : public ayanami::abstract::AbstractProgressChecker {
 public:
  SimpleProgressChecker() = default;

  void configure(const ayanami::PluginContext& context) override;

  void reset() override;

  bool check(const ayanami::geometry_msgs::PoseStamped& current_pose) override;

 private:
  struct SimpleProgressCheckerParam {
    double required_movement = 0.5;         // m
    double movement_angle_required = 0.2;   // rad
    double time_period = 5.0;               // s
  };

  SimpleProgressCheckerParam param;

  ayanami::geometry_msgs::PoseStamped base_pose;
  std::chrono::steady_clock::time_point base_time;
  bool initialized{false};
  std::atomic<bool> reset_pending{false};
};

}  // namespace modules
}  // namespace ayanami
