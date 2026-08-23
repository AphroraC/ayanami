// kinematic_parameters.hpp — 迁移自 dwb_plugins/kinematic_parameters.hpp
// (去 ROS 动态参数回调; Config 一次性读取 + setSpeedLimit 运行时覆盖)。
#pragma once

#include <memory>
#include <string>

#include "ayanami/utility/config.hpp"

namespace dwb_plugins {

/**
 * @struct KinematicParameters
 * @brief A struct containing one representation of the robot's kinematics
 */
struct KinematicParameters {
  friend class KinematicsHandler;

  inline double getMinX() { return min_vel_x_; }
  inline double getMaxX() { return max_vel_x_; }
  inline double getAccX() { return acc_lim_x_; }
  inline double getDecelX() { return decel_lim_x_; }

  inline double getMinY() { return min_vel_y_; }
  inline double getMaxY() { return max_vel_y_; }
  inline double getAccY() { return acc_lim_y_; }
  inline double getDecelY() { return decel_lim_y_; }

  inline double getMinSpeedXY() { return min_speed_xy_; }
  inline double getMaxSpeedXY() { return max_speed_xy_; }

  inline double getMinTheta() { return -max_vel_theta_; }
  inline double getMaxTheta() { return max_vel_theta_; }
  inline double getAccTheta() { return acc_lim_theta_; }
  inline double getDecelTheta() { return decel_lim_theta_; }
  inline double getMinSpeedTheta() { return min_speed_theta_; }

  inline double getMinSpeedXY_SQ() { return min_speed_xy_sq_; }
  inline double getMaxSpeedXY_SQ() { return max_speed_xy_sq_; }

protected:
  // For parameter descriptions, see nav2 dwb_plugins KinematicParams
  double min_vel_x_{0};
  double min_vel_y_{0};
  double max_vel_x_{0};
  double max_vel_y_{0};
  double base_max_vel_x_{0};
  double base_max_vel_y_{0};
  double max_vel_theta_{0};
  double base_max_vel_theta_{0};
  double min_speed_xy_{0};
  double max_speed_xy_{0};
  double base_max_speed_xy_{0};
  double min_speed_theta_{0};
  double acc_lim_x_{0};
  double acc_lim_y_{0};
  double acc_lim_theta_{0};
  double decel_lim_x_{0};
  double decel_lim_y_{0};
  double decel_lim_theta_{0};

  // Cached square values of min_speed_xy and max_speed_xy
  double min_speed_xy_sq_{0};
  double max_speed_xy_sq_{0};
};

/**
 * @brief 等价 nav2_costmap_2d 的 NO_SPEED_LIMIT 哨兵 (costmap_filters/
 * filter_values.hpp)。
 */
constexpr double NO_SPEED_LIMIT = 0.0;

/**
 * @class KinematicsHandler
 * @brief A class managing the representation of the robot's kinematics
 */
class KinematicsHandler {
public:
  KinematicsHandler();
  ~KinematicsHandler();
  void initialize(const ayanami::Config& cfg, const std::string& plugin_name);

  inline KinematicParameters getKinematics() { return kinematics_; }

  void setSpeedLimit(const double& speed_limit, const bool& percentage);

  using Ptr = std::shared_ptr<KinematicsHandler>;

protected:
  KinematicParameters kinematics_;
  std::string plugin_name_;
};

}  // namespace dwb_plugins
