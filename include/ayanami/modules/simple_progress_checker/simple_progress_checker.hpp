#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <ayanami/abstract/core/abstract_progress_checker.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief 简单进展检查模块 (对标 MBF SimpleProgressChecker)。
 *
 * 纯 C++ (零 ROS): 跟踪基线位姿与时戳。time_period 内若移动距离 >=
 * required_movement 或转角 >= movement_angle_required 则更新基线并判有进展;
 * 时间窗内未移动则判无进展 (卡住)。
 */
class SimpleProgressChecker : public ayanami::abstract::AbstractProgressChecker {
 public:
  SimpleProgressChecker();

  void reset() override;

  bool check(const ayanami::geometry_msgs::PoseStamped& current_pose) override;

 private:
  void loadConfig();

  struct Config {
    double required_movement = 0.5;         // m
    double movement_angle_required = 0.2;   // rad
    double time_period = 5.0;               // s
  };

  Config config_;

  ayanami::geometry_msgs::PoseStamped base_pose_;
  std::chrono::steady_clock::time_point base_time_;
  bool initialized_{false};
  std::atomic<bool> reset_pending_{false};
};

}  // namespace modules
}  // namespace ayanami