// kinematic_parameters.hpp -- ported from dwb_plugins/kinematic_parameters.hpp
// (ROS dynamic parameter callbacks removed; one-shot Config reads + setSpeedLimit runtime override).
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

  inline double getMinX() { return min_vel_x; }
  inline double getMaxX() { return max_vel_x; }
  inline double getAccX() { return acc_lim_x; }
  inline double getDecelX() { return decel_lim_x; }

  inline double getMinY() { return min_vel_y; }
  inline double getMaxY() { return max_vel_y; }
  inline double getAccY() { return acc_lim_y; }
  inline double getDecelY() { return decel_lim_y; }

  inline double getMinSpeedXY() { return min_speed_xy; }
  inline double getMaxSpeedXY() { return max_speed_xy; }

  inline double getMinTheta() { return -max_vel_theta; }
  inline double getMaxTheta() { return max_vel_theta; }
  inline double getAccTheta() { return acc_lim_theta; }
  inline double getDecelTheta() { return decel_lim_theta; }
  inline double getMinSpeedTheta() { return min_speed_theta; }

  inline double getMinSpeedXY_SQ() { return min_speed_xy_sq; }
  inline double getMaxSpeedXY_SQ() { return max_speed_xy_sq; }

protected:
  // For parameter descriptions, see nav2 dwb_plugins KinematicParams
  double min_vel_x{0};
  double min_vel_y{0};
  double max_vel_x{0};
  double max_vel_y{0};
  double base_max_vel_x{0};
  double base_max_vel_y{0};
  double max_vel_theta{0};
  double base_max_vel_theta{0};
  double min_speed_xy{0};
  double max_speed_xy{0};
  double base_max_speed_xy{0};
  double min_speed_theta{0};
  double acc_lim_x{0};
  double acc_lim_y{0};
  double acc_lim_theta{0};
  double decel_lim_x{0};
  double decel_lim_y{0};
  double decel_lim_theta{0};

  // Cached square values of min_speed_xy and max_speed_xy
  double min_speed_xy_sq{0};
  double max_speed_xy_sq{0};
};

/**
 * @brief Equivalent of the nav2_costmap_2d NO_SPEED_LIMIT sentinel (costmap_filters/
 * filter_values.hpp).
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

  inline KinematicParameters getKinematics() { return kinematics; }

  void setSpeedLimit(const double& speed_limit, const bool& percentage);

  using Ptr = std::shared_ptr<KinematicsHandler>;

protected:
  KinematicParameters kinematics;
  std::string plugin_name;
};

}  // namespace dwb_plugins
