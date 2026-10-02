#pragma once

#include "ayanami/messages/geometry_msgs.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/messages/sensor_msgs.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <functional>
#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief Host navigation module entry point.
 *
 * The ROS-side dlopen host module (e.g. CostmapNavigation) forwards inputs
 * only through this interface: initialize/start/stop/spinOnce/setGoal +
 * setPointcloud/setLaserScan; internally it holds the full
 * NavigationServerBase + FsmDriver stack.
 */
class AbstractNavigation : public virtual ExtensionModule {
public:
  using Ptr = std::shared_ptr<AbstractNavigation>;

  virtual ~AbstractNavigation() = default;

  /// @brief Load configuration from config_path and initialize all components/modules.
  virtual void initialize(const std::string& config_path) = 0;

  /// @brief Start the internal loops (resident BT tree / subscriptions).
  virtual void start() = 0;

  /// @brief Stop the internal loops.
  virtual void stop() = 0;

  /// @brief Poll once without blocking (for integration into external threads/timers).
  virtual void spinOnce() = 0;

  /// @brief Set the navigation goal.
  virtual void setGoal(const ayanami::geometry_msgs::PoseStamped& goal) = 0;

  /// @brief Sensor observation input (point cloud). Semantics are defined
  /// by the map family's host (costmap: obstacle-to-map pipeline; mesh:
  /// reconstruction input; ...). The ROS adapter syncs TF and hands the
  /// converted result straight here. Default no-op (hosts without a use for
  /// clouds may ignore).
  virtual void setPointcloud(const ayanami::sensor_msgs::PointCloud2& cloud) {
    (void)cloud;
  }
  /// @brief Sensor observation input (laser scan), same contract as above.
  virtual void setLaserScan(const ayanami::sensor_msgs::LaserScan& scan) {
    (void)scan;
  }
};

} // namespace abstract
} // namespace ayanami
