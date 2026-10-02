#pragma once

#include <memory>
#include <string>

#include <ayanami/abstract/components/navigation_fsm.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/costmap/components/costmap_navigation_server.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief The FSM-era costmap navigation host: the full costmap stack +
 * FsmDriver orchestration over costmap-family modules.
 *
 * Composition: CostmapNavigationServer member (sole writer thread,
 * observation pipeline, sensor liveness, global static + inflation /
 * local obstacle + inflation), modules loaded through the base family
 * hooks (planner/smoother/controller are all costmap family), executions
 * assembled directly (the Action layer is bypassed) and driven by the
 * FsmDriver.
 */
class CostmapNavigationFsm : public components::NavigationServerBase,
                             public abstract::AbstractNavigation {
 public:
  CostmapNavigationFsm();
  ~CostmapNavigationFsm() override;

  // ---- AbstractNavigation ----
  void initialize(const std::string& config_path) override;
  void start() override;
  void stop() override;
  void spinOnce() override {}
  void setGoal(const ayanami::geometry_msgs::PoseStamped& goal) override;
  void setPointcloud(
      const ayanami::sensor_msgs::PointCloud2& cloud) override;
  void setLaserScan(const ayanami::sensor_msgs::LaserScan& scan) override;

  // ---- NavigationServerBase family hooks ----
  bool initializePlannerModule(const std::string& name,
                               const ayanami::Config& module_config,
                               abstract::AbstractPlanner::Ptr) override;
  bool initializeSmootherModule(const std::string& name,
                                const ayanami::Config& module_config,
                                abstract::AbstractSmoother::Ptr) override;
  bool initializeControllerModule(const std::string& name,
                                  const ayanami::Config& module_config,
                                  abstract::AbstractController::Ptr) override;
  bool initializeRecoveryModule(const std::string& name,
                                const ayanami::Config& module_config,
                                abstract::AbstractRecovery::Ptr) override;

  /// @brief Repurposed assembly step (loadModules' tail call): build the
  /// family executions + the FsmDriver instead of Actions.
  void createActions() override;

  // ---- accessors (tests) ----
  components::FsmDriver* fsm() const { return fsm_.get(); }
  costmap::CostmapNavigationServer* costmapServer() const {
    return costmap_server_.get();
  }

 private:
  std::shared_ptr<costmap::CostmapNavigationServer> costmap_server_;
  std::unique_ptr<components::FsmDriver> fsm_;
};

}  // namespace modules
}  // namespace ayanami
