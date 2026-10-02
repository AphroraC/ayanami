// RoseNavigationFsm host module implementation.
//
// The FSM-era successor of rosemap_navigation / costmap_navigation: same
// AbstractNavigation boundary (initialize/start/stop/setGoal/observations),
// the BT pilot replaced by the FsmDriver, and the map stack composed as
// plain members (costmap machinery + derived ESDF writer).

#include "rose_navigation_fsm.hpp"

#include <ayanami/costmap/components/costmap_planner_execution.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/esdfmap/components/esdfmap_smoother_execution.hpp>
#include <ayanami/esdfmap/core/esdfmap_smoother.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/bootstrap.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/logging.hpp>

#include <costmap/cost_values.hpp>

#include <spdlog/spdlog.h>

#include <cmath>
#include <mutex>
#include <utility>

namespace ayanami {
namespace modules {

RoseNavigationFsm::RoseNavigationFsm()
    : costmap_server_(std::make_shared<costmap::CostmapNavigationServer>()) {}

RoseNavigationFsm::~RoseNavigationFsm() { stop(); }

void RoseNavigationFsm::initialize(const std::string& config_path) {
  // bootstrap() owns both the config root and logging for the process.
  ayanami::bootstrap(config_path);

  // Costmap machinery (writer thread, observation pipeline, layered maps).
  // The costmap server's own module loading is never used -- this host
  // loads its own mixed-family module set through the base hooks.
  costmap_server_->configureCoreMaps(config_path);
  cacheLocalObstacleLayer();

  // ESDF field + writer (mask readers + transform lookup injected below;
  // the package is libayanami-free by design).
  esdf_map_ = std::make_shared<::esdfmap::EsdfMap>();
  configureEsdfWriter(config_path);

  // Modules (hooks above) -> createActions() assembles executions + FSM.
  loadModules(config_path);
}

void RoseNavigationFsm::configureEsdfWriter(const std::string& config_path) {
  ayanami::Config nav_cfg(
      ayanami::GlobalConfig::getConfigPath("config_navigation"));
  ::esdfmap::EsdfWriterConfig writer_config;
  writer_config.max_distance =
      nav_cfg.param<double>("esdfmap", "max_distance", 3.0);
  writer_config.update_rate =
      nav_cfg.param<double>("esdfmap", "update_rate", 20.0);
  esdf_map_->setStaleTimeout(
      nav_cfg.param<double>("esdfmap", "stale_timeout", 0.0));
  (void)config_path;

  // Rose-style MAP-FRAME field: the BASE mask is the global static map
  // (map frame, fixed window -- it defines the field geometry and frame);
  // the rolling LOCAL live marks are the overlay, projected through TF.
  // Without a global costmap the local mask stands in as the base (odom
  // frame; the fusion then skips on equal frames and the field keeps
  // working). Reads happen under the map shared lock; the view points at
  // reader-owned storage for the duration of the call only.
  esdf_writer_ = std::make_shared<::esdfmap::EsdfWriter>(
      esdf_map_,
      [this](::esdfmap::EsdfMaskView& view) {
        return readGlobalMask(global_mask_cells_, view) ||
               readLocalMask(global_mask_cells_, view);
      },
      writer_config);
  esdf_writer_->setOverlaySource(
      [this](::esdfmap::EsdfMaskView& view) {
        local_mask_cells_.clear();
        return readLocalMask(local_mask_cells_, view);
      },
      [](const std::string& target, const std::string& source, double& tx,
         double& ty, double& yaw) {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        if (!components::RobotStateController::instance().lookupTransform(
                target, source, ayanami::Timestamp::latest(), transform,
                0.0)) {
          return false;
        }
        tx = transform(0, 3);
        ty = transform(1, 3);
        yaw = std::atan2(transform(1, 0), transform(0, 0));
        return true;
      });
  // Visualization bridge: every published snapshot (writer thread, sequence-
  // gated) is converted to the core EsdfGrid message and emitted on the
  // global slot -- ONLY while a subscriber exists (the slot's bool conversion
  // reads its subscriber count), so an unobserved deployment pays zero copy.
  esdf_writer_->setSnapshotListener(
      [](const std::shared_ptr<const ::esdfmap::EsdfSnapshot>& snapshot) {
        auto& slot =
            ayanami::interfaces::NavigationCallbacks::on_esdfmap_field;
        if (!snapshot || !snapshot->valid() || !slot) {
          return;
        }
        ayanami::nav_msgs::EsdfGrid grid;
        grid.header.frame_id = snapshot->frame;
        grid.resolution = snapshot->resolution;
        grid.size_x = snapshot->size_x;
        grid.size_y = snapshot->size_y;
        grid.origin_x = snapshot->origin_x;
        grid.origin_y = snapshot->origin_y;
        grid.max_distance = snapshot->max_distance;
        grid.sequence = snapshot->sequence;
        grid.distance = snapshot->distance;
        grid.occupied = snapshot->occupied;
        slot(grid);
      });
}

void RoseNavigationFsm::cacheLocalObstacleLayer() {
  local_obstacle_layer_ = nullptr;
  const auto wrapper = costmap_server_->localCostmap();
  if (!wrapper) {
    return;
  }
  for (const auto& layer : wrapper->get().getModules()) {
    if (auto* obstacle =
            dynamic_cast<costmap_2d::SimpleObstacleLayer*>(layer.get())) {
      local_obstacle_layer_ = obstacle;
      return;
    }
  }
}

bool RoseNavigationFsm::readLocalMask(std::vector<unsigned char>& cells,
                                      ::esdfmap::EsdfMaskView& view) {
  const auto wrapper = costmap_server_->localCostmap();
  if (!wrapper || !local_obstacle_layer_) {
    return false;
  }
  // Pose resolution before the lock (the costmap read discipline); the
  // presence grid is read under the shared lock and copied out.
  std::shared_lock<std::shared_mutex> lock(wrapper->mutex());
  const auto& presence = local_obstacle_layer_->presenceGrid();
  if (presence.empty() || local_obstacle_layer_->geomSizeX() == 0) {
    return false;
  }
  cells = presence;  // copy: the writer only consumes it inside the call
  view.width = static_cast<int>(local_obstacle_layer_->geomSizeX());
  view.height = static_cast<int>(local_obstacle_layer_->geomSizeY());
  view.resolution = local_obstacle_layer_->geomResolution();
  view.origin_x = local_obstacle_layer_->geomOriginX();
  view.origin_y = local_obstacle_layer_->geomOriginY();
  view.frame = wrapper->globalFrame();
  view.generation = wrapper->generation();
  view.cells = &cells;
  return true;
}

bool RoseNavigationFsm::readGlobalMask(std::vector<unsigned char>& cells,
                                       ::esdfmap::EsdfMaskView& view) {
  const auto context = costmap_server_->costmapContext();
  if (!context) {
    return false;
  }
  costmap::CostmapSnapshot snapshot;
  if (!context->snapshotGlobal(snapshot) || !snapshot.valid()) {
    return false;
  }
  cells.clear();
  cells.reserve(snapshot.costs.size());
  for (const unsigned char cost : snapshot.costs) {
    // Lethal-only: inflation gradient is the costmap's safety expression,
    // not obstacle geometry -- the ESDF computes its own distances.
    cells.push_back(cost >= costmap_2d::LETHAL_OBSTACLE ? 1 : 0);
  }
  view.width = static_cast<int>(snapshot.size_x);
  view.height = static_cast<int>(snapshot.size_y);
  view.resolution = snapshot.resolution;
  view.origin_x = snapshot.origin_x;
  view.origin_y = snapshot.origin_y;
  view.frame = snapshot.frame;
  view.generation = snapshot.generation;
  view.cells = &cells;
  return true;
}

// ---- family hooks: mixed-family module set ----

bool RoseNavigationFsm::initializePlannerModule(
    const std::string& name, const ayanami::Config& module_config,
    abstract::AbstractPlanner::Ptr module) {
  auto* costmap_planner =
      dynamic_cast<costmap::CostmapPlanner*>(module.get());
  if (!costmap_planner) {
    SPDLOG_LOGGER_ERROR(
        ayanami::plannerLogger(),
        "RoseNavigationFsm: planner '{}' is not a CostmapPlanner "
        "(costmap family required: the planner is the costmap's reader)",
        name);
    return false;
  }
  costmap_planner->configure(
      ModuleContext{name, module_config},
      costmap_server_->costmapContextConst());
  return true;
}

bool RoseNavigationFsm::initializeSmootherModule(
    const std::string& name, const ayanami::Config& module_config,
    abstract::AbstractSmoother::Ptr module) {
  auto* esdf_smoother =
      dynamic_cast<esdfmap::EsdfmapSmoother*>(module.get());
  if (!esdf_smoother) {
    SPDLOG_LOGGER_ERROR(
        ayanami::smootherLogger(),
        "RoseNavigationFsm: smoother '{}' is not an EsdfmapSmoother "
        "(esdfmap family required: the smoother is the field's only reader)",
        name);
    return false;
  }
  esdf_smoother->configure(ModuleContext{name, module_config}, esdf_map_);
  return true;
}

bool RoseNavigationFsm::initializeControllerModule(
    const std::string&, const ayanami::Config&,
    abstract::AbstractController::Ptr) {
  // Map-free by contract: a pure path-tracking controller takes no map.
  return true;
}

bool RoseNavigationFsm::initializeRecoveryModule(
    const std::string&, const ayanami::Config&,
    abstract::AbstractRecovery::Ptr) {
  return true;
}

// ---- assembly (loadModules tail call) ----

void RoseNavigationFsm::createActions() {
  // One execution per family (FsmDriver reuse model), assembled directly --
  // the Action layer is bypassed.
  components::PlannerExecution::Ptr planner_execution;
  if (auto planner = planner_modules.getModule(default_planner)) {
    planner_execution = std::make_shared<costmap::CostmapPlannerExecution>(
        default_planner, planner, costmap_server_->costmapContext());
  } else {
    SPDLOG_LOGGER_ERROR(ayanami::plannerLogger(),
                        "RoseNavigationFsm: no planner module loaded");
    return;
  }

  components::SmootherExecution::Ptr smoother_execution;
  if (auto smoother = smoother_modules.getModule(default_smoother)) {
    smoother_execution =
        std::make_shared<esdfmap::EsdfmapSmootherExecution>(
            default_smoother, smoother, esdf_map_);
  } else {
    SPDLOG_LOGGER_ERROR(ayanami::smootherLogger(),
                        "RoseNavigationFsm: no smoother module loaded");
    return;
  }

  components::ControllerExecution::Ptr controller_execution;
  if (auto controller = controller_modules.getModule(default_controller)) {
    controller_execution =
        std::make_shared<components::ControllerExecution>(controller);
    configureControllerExecution(controller_execution);
    // FollowPath feedback semantics: the execution's per-cycle metrics go
    // out through the on_exe_path_feedback slot.
    controller_execution->setFeedbackCallback(
        [](const ayanami::actions::ExePathFeedback& feedback) {
          ayanami::interfaces::NavigationCallbacks::on_exe_path_feedback(
              feedback);
        });
  } else {
    SPDLOG_LOGGER_ERROR(ayanami::controllerLogger(),
                        "RoseNavigationFsm: no controller module loaded");
    return;
  }

  std::vector<components::RecoveryExecution::Ptr> recoveries;
  for (const auto& name : recovery_sequence) {
    if (auto recovery = recovery_modules.getModule(name)) {
      auto execution = std::make_shared<components::RecoveryExecution>(
          recovery);
      configureRecoveryExecution(execution);
      execution->setCyclePeriod(std::chrono::milliseconds(10));
      recoveries.push_back(std::move(execution));
    }
  }

  // FSM tunables (the former blackboard-injected policy keys).
  ayanami::Config nav_cfg(
      ayanami::GlobalConfig::getConfigPath("config_navigation"));
  components::FsmDriverConfig driver_config;
  driver_config.fsm_frequency =
      nav_cfg.param<double>("navigation", "fsm_frequency", 20.0);
  driver_config.planner_frequency =
      nav_cfg.param<double>("navigation", "planner_frequency", 5.0);
  driver_config.smoother_frequency =
      nav_cfg.param<double>("navigation", "smoother_frequency", 10.0);
  driver_config.max_smooth_duration =
      nav_cfg.param<double>("navigation", "smoother_timeout", 2.0);
  driver_config.max_nav_retries =
      static_cast<std::uint32_t>(nav_cfg.param<int>(
          "navigation", "max_nav_retries", 6));
  driver_config.oscillation_distance =
      nav_cfg.param<double>("navigation", "oscillation_distance", 0.1);
  driver_config.oscillation_timeout =
      nav_cfg.param<double>("navigation", "oscillation_timeout", 5.0);

  fsm_ = std::make_unique<components::FsmDriver>(
      planner_execution, smoother_execution, controller_execution,
      std::move(recoveries), driver_config);
  // Path validity: the planner module's own probe (it is the costmap
  // reader; default falls back to the interface's non-empty check).
  if (auto planner = planner_modules.getModule(default_planner)) {
    fsm_->setPathValidator(
        [planner](const ayanami::nav_msgs::Path& path) {
          return planner->isPathValid(path);
        });
  }
}

// ---- lifecycle ----

void RoseNavigationFsm::start() {
  costmap_server_->start();
  if (esdf_writer_) {
    esdf_writer_->start();
  }
  NavigationServerBase::start();  // module activation
  if (fsm_) {
    fsm_->start();
  }
}

void RoseNavigationFsm::stop() {
  if (fsm_) {
    fsm_->stop();
  }
  if (esdf_writer_) {
    esdf_writer_->stop();
  }
  costmap_server_->stop();
  NavigationServerBase::stop();  // module deactivation
}

void RoseNavigationFsm::setGoal(
    const ayanami::geometry_msgs::PoseStamped& goal) {
  if (!fsm_) {
    SPDLOG_LOGGER_ERROR(ayanami::btLogger(),
                        "RoseNavigationFsm: setGoal before initialize");
    return;
  }
  // Non-blocking latest-wins mailbox delivery (the host setGoal contract):
  // the first goal starts an attempt; a goal during an attempt is an
  // in-place revision.
  fsm_->beginNavigation(goal);
}

void RoseNavigationFsm::setPointcloud(
    const ayanami::sensor_msgs::PointCloud2& cloud) {
  costmap_server_->handleObstaclePointCloud(cloud);
}

void RoseNavigationFsm::setLaserScan(
    const ayanami::sensor_msgs::LaserScan& scan) {
  costmap_server_->handleObstacleLaserScan(scan);
}

}  // namespace modules
}  // namespace ayanami
