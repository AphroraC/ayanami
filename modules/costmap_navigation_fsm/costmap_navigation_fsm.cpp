// CostmapNavigationFsm host module implementation.
//
// The FSM-era successor of costmap_navigation: same AbstractNavigation
// boundary, the BT pilot replaced by the FsmDriver, the costmap stack held
// as a plain member.

#include "costmap_navigation_fsm.hpp"

#include <ayanami/costmap/components/costmap_controller_execution.hpp>
#include <ayanami/costmap/components/costmap_planner_execution.hpp>
#include <ayanami/costmap/components/costmap_smoother_execution.hpp>
#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/core/costmap_planner.hpp>
#include <ayanami/costmap/core/costmap_smoother.hpp>
#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/bootstrap.hpp>
#include <ayanami/utility/config.hpp>
#include <ayanami/utility/logging.hpp>

#include <spdlog/spdlog.h>

#include <utility>

namespace ayanami {
namespace modules {

CostmapNavigationFsm::CostmapNavigationFsm()
    : costmap_server_(std::make_shared<costmap::CostmapNavigationServer>()) {}

CostmapNavigationFsm::~CostmapNavigationFsm() { stop(); }

void CostmapNavigationFsm::initialize(const std::string& config_path) {
  ayanami::bootstrap(config_path);

  // Costmap machinery; the costmap server's own module loading is never
  // used -- this host loads its own module set through the base hooks.
  costmap_server_->configureCoreMaps(config_path);

  loadModules(config_path);
}

// ---- family hooks: costmap-family module set ----

bool CostmapNavigationFsm::initializePlannerModule(
    const std::string& name, const ayanami::Config& module_config,
    abstract::AbstractPlanner::Ptr module) {
  auto* costmap_planner =
      dynamic_cast<costmap::CostmapPlanner*>(module.get());
  if (!costmap_planner) {
    SPDLOG_LOGGER_ERROR(ayanami::plannerLogger(),
                        "CostmapNavigationFsm: planner '{}' is not a "
                        "CostmapPlanner",
                        name);
    return false;
  }
  costmap_planner->configure(
      ModuleContext{name, module_config},
      costmap_server_->costmapContextConst());
  return true;
}

bool CostmapNavigationFsm::initializeSmootherModule(
    const std::string& name, const ayanami::Config& module_config,
    abstract::AbstractSmoother::Ptr module) {
  auto* costmap_smoother =
      dynamic_cast<costmap::CostmapSmoother*>(module.get());
  if (!costmap_smoother) {
    SPDLOG_LOGGER_ERROR(ayanami::smootherLogger(),
                        "CostmapNavigationFsm: smoother '{}' is not a "
                        "CostmapSmoother",
                        name);
    return false;
  }
  costmap_smoother->configure(
      ModuleContext{name, module_config},
      costmap_server_->costmapContextConst());
  return true;
}

bool CostmapNavigationFsm::initializeControllerModule(
    const std::string& name, const ayanami::Config& module_config,
    abstract::AbstractController::Ptr module) {
  auto* costmap_controller =
      dynamic_cast<costmap::CostmapController*>(module.get());
  if (!costmap_controller) {
    SPDLOG_LOGGER_ERROR(ayanami::controllerLogger(),
                        "CostmapNavigationFsm: controller '{}' is not a "
                        "CostmapController",
                        name);
    return false;
  }
  costmap_controller->configure(
      ModuleContext{name, module_config},
      costmap_server_->costmapContextConst());
  return true;
}

bool CostmapNavigationFsm::initializeRecoveryModule(
    const std::string&, const ayanami::Config&,
    abstract::AbstractRecovery::Ptr) {
  return true;
}

// ---- assembly (loadModules tail call) ----

void CostmapNavigationFsm::createActions() {
  components::PlannerExecution::Ptr planner_execution;
  if (auto planner = planner_modules.getModule(default_planner)) {
    planner_execution = std::make_shared<costmap::CostmapPlannerExecution>(
        default_planner, planner, costmap_server_->costmapContext());
  } else {
    SPDLOG_LOGGER_ERROR(ayanami::plannerLogger(),
                        "CostmapNavigationFsm: no planner module loaded");
    return;
  }

  components::SmootherExecution::Ptr smoother_execution;
  if (auto smoother = smoother_modules.getModule(default_smoother)) {
    smoother_execution =
        std::make_shared<costmap::CostmapSmootherExecution>(
            default_smoother, smoother, costmap_server_->costmapContext());
  } else {
    SPDLOG_LOGGER_ERROR(ayanami::smootherLogger(),
                        "CostmapNavigationFsm: no smoother module loaded");
    return;
  }

  components::ControllerExecution::Ptr controller_execution;
  if (auto controller = controller_modules.getModule(default_controller)) {
    controller_execution =
        std::make_shared<costmap::CostmapControllerExecution>(
            default_controller, controller,
            costmap_server_->costmapContext());
    configureControllerExecution(controller_execution);
    controller_execution->setFeedbackCallback(
        [](const ayanami::actions::ExePathFeedback& feedback) {
          ayanami::interfaces::NavigationCallbacks::on_exe_path_feedback(
              feedback);
        });
  } else {
    SPDLOG_LOGGER_ERROR(ayanami::controllerLogger(),
                        "CostmapNavigationFsm: no controller module loaded");
    return;
  }

  std::vector<components::RecoveryExecution::Ptr> recoveries;
  for (const auto& name : recovery_sequence) {
    if (auto recovery = recovery_modules.getModule(name)) {
      auto execution =
          std::make_shared<components::RecoveryExecution>(recovery);
      configureRecoveryExecution(execution);
      execution->setCyclePeriod(std::chrono::milliseconds(10));
      recoveries.push_back(std::move(execution));
    }
  }

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
  if (auto planner = planner_modules.getModule(default_planner)) {
    fsm_->setPathValidator(
        [planner](const ayanami::nav_msgs::Path& path) {
          return planner->isPathValid(path);
        });
  }
}

// ---- lifecycle ----

void CostmapNavigationFsm::start() {
  costmap_server_->start();
  NavigationServerBase::start();  // module activation
  if (fsm_) {
    fsm_->start();
  }
}

void CostmapNavigationFsm::stop() {
  if (fsm_) {
    fsm_->stop();
  }
  costmap_server_->stop();
  NavigationServerBase::stop();  // module deactivation
}

void CostmapNavigationFsm::setGoal(
    const ayanami::geometry_msgs::PoseStamped& goal) {
  if (!fsm_) {
    SPDLOG_LOGGER_ERROR(ayanami::btLogger(),
                        "CostmapNavigationFsm: setGoal before initialize");
    return;
  }
  fsm_->beginNavigation(goal);
}

void CostmapNavigationFsm::setPointcloud(
    const ayanami::sensor_msgs::PointCloud2& cloud) {
  costmap_server_->handleObstaclePointCloud(cloud);
}

void CostmapNavigationFsm::setLaserScan(
    const ayanami::sensor_msgs::LaserScan& scan) {
  costmap_server_->handleObstacleLaserScan(scan);
}

}  // namespace modules
}  // namespace ayanami
