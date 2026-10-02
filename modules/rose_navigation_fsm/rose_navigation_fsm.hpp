#pragma once

#include <memory>
#include <string>

#include <esdfmap/esdf_map.hpp>
#include <esdfmap/esdf_writer.hpp>

#include <ayanami/abstract/components/navigation_fsm.hpp>
#include <ayanami/abstract/components/navigation_server_base.hpp>
#include <ayanami/abstract/core/abstract_navigation.hpp>
#include <ayanami/costmap/components/costmap_navigation_server.hpp>

namespace ayanami {
namespace modules {

/**
 * @brief The FSM-era navigation host: costmap perception + derived ESDF
 * field + FsmDriver orchestration.
 *
 * Composition (the map-family servers' successor):
 * - CostmapNavigationServer member: the full costmap machinery (sole writer
 *   thread, observation pipeline, sensor liveness, global static +
 *   inflation / local obstacle + inflation). Its module loading is NOT
 *   used; the host loads its own mixed-family module set.
 * - esdfmap::EsdfWriter: derives the signed distance field in the MAP
 *   frame (rose static-map mode): the BASE mask is the global static map
 *   (fixed window, defines the field geometry/frame) and the rolling LOCAL
 *   live marks are OR-projected into it through TF (read under the map
 *   shared lock), publishing immutable snapshots. Without a global costmap
 *   the local mask stands in as the base (odom frame).
 * - Modules: costmap-family planner (the costmap's reader), esdfmap-family
 *   smoother (the field's only reader), map-free abstract controller
 *   (pure path tracking), placeholder recovery.
 * - FsmDriver: the BT's semantic successor, driven by injected executions.
 *
 * Module loading reuses NavigationServerBase::loadModules with overridden
 * family hooks; createActions is repurposed as the execution + FSM assembly
 * step (the Action layer is bypassed entirely).
 */
class RoseNavigationFsm : public components::NavigationServerBase,
                          public abstract::AbstractNavigation {
 public:
  RoseNavigationFsm();
  ~RoseNavigationFsm() override;

  // ---- AbstractNavigation ----
  void initialize(const std::string& config_path) override;
  void start() override;
  void stop() override;
  void spinOnce() override {}
  void setGoal(const ayanami::geometry_msgs::PoseStamped& goal) override;
  void setPointcloud(
      const ayanami::sensor_msgs::PointCloud2& cloud) override;
  void setLaserScan(const ayanami::sensor_msgs::LaserScan& scan) override;

  // ---- NavigationServerBase family hooks (mixed-family module set) ----
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
  ::esdfmap::EsdfMap* esdfMap() const { return esdf_map_.get(); }

 private:
  /// @brief Configure the ESDF writer: mask readers over the costmap
  /// layers + the transform lookup (all injected; the package itself is
  /// libayanami-free).
  void configureEsdfWriter(const std::string& config_path);

  /// @brief Locate (and cache) the local costmap's obstacle layer.
  void cacheLocalObstacleLayer();

  /// @brief Read one mask from the local obstacle presence grid (under the
  /// map shared lock) into `cells`; returns false when unavailable.
  bool readLocalMask(std::vector<unsigned char>& cells,
                     ::esdfmap::EsdfMaskView& view);

  /// @brief Read the global static mask (lethal cells of the global master
  /// grid, short-lock snapshot) into `cells`.
  bool readGlobalMask(std::vector<unsigned char>& cells,
                      ::esdfmap::EsdfMaskView& view);

  std::shared_ptr<costmap::CostmapNavigationServer> costmap_server_;
  ::esdfmap::EsdfMap::Ptr esdf_map_;
  ::esdfmap::EsdfWriter::Ptr esdf_writer_;
  costmap_2d::SimpleObstacleLayer* local_obstacle_layer_{nullptr};
  /// @brief Mask read buffers (the reader copies the grid out under the
  /// map lock; the view points at these for the duration of the call).
  std::vector<unsigned char> local_mask_cells_;
  std::vector<unsigned char> global_mask_cells_;

  std::unique_ptr<components::FsmDriver> fsm_;
};

}  // namespace modules
}  // namespace ayanami
