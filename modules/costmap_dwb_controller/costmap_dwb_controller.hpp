// costmap_dwb_controller.hpp -- ayanami port of nav2 dwb_core::DWBLocalPlanner.
//
// Algorithm logic maps function-by-function to nav2_dwb_controller (dwb_core):
// configure -> configure(ModuleContext, map)
// setPlan -> setPlan (Path -> Path2D, resets critics + generator)
// computeVelocityCommands -> computeVelocityCommand (abstract signature;
// the module reads the local grid under lock via its held map module)
// prepareGlobalPlan / coreScoringAlgorithm / scoreTrajectory /
// transformGlobalPlan / loadCritics keep their original names and semantics
//
// Lock discipline: each control cycle reads through the map module's readLocal callback and reads the local grid directly. The read scope is the cycle;
// the snapshot (shared_ptr) keeps the grid alive for the whole scoring cycle, and before prepare the current grid /
// footprint is bound to each critic (critic::setCostmap). It holds no map write lock --
// the old implementation's ctx->local_costmap->mutex internal lock was removed.
//
// Replacement points (zero ROS):
// - pluginlib::ClassLoader -> static registry (registration.hpp)
// - tf2_ros::Buffer + nav_2d_utils::tf_help -> RobotStateController bridge singleton
// - rclcpp parameters -> ayanami::Config (json section dwb_module_name, keys are dotted full names)
// - DWBPublisher (ROS publishing) -> removed (debug visualization via an optional callback slot, omitted for now)
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d.hpp>

#include <ayanami/costmap/core/costmap_controller.hpp>
#include <ayanami/costmap/core/costmap_reader.hpp>
#include <ayanami/utility/config.hpp>

#include "dwb_types.hpp"
#include "trajectory_critic.hpp"
#include "trajectory_generator.hpp"

namespace ayanami {
namespace modules {

using dwb_core::TrajectoryCritic;
using dwb_core::TrajectoryGenerator;

class CostmapDwbController final : public ayanami::costmap::CostmapController {
public:
  using Ptr = std::shared_ptr<CostmapDwbController>;

  /**
   * @brief Constructor. Registers all built-in critics / generators into the registries.
   */
  CostmapDwbController();

  virtual ~CostmapDwbController() = default;

  /// @brief costmap family injection contract (equivalent to nav2 configure): reads its own
  /// configuration via ModuleContext, creates and initializes the trajectory generator, and runs loadCritics.
  /// @brief Parameter-loading seam (ExtensionModule): called by the loader
  /// right after a successful load, and by configure() for callers that build
  /// the object directly.
  void configureParameters(const ayanami::ModuleContext& context) override;

  void configure(const ModuleContext& context,
                 ayanami::costmap::CostmapMapContext::ConstPtr map) override;

  /// @brief Equivalent of nav2 setPlan.
  bool setPlan(const ayanami::nav_msgs::Path& plan) override;

  /// @brief Arrival contract from the goal checker (via the execution layer):
  /// forwarded to the critics so one of them (RotateToGoal) can validate its
  /// own engagement radius against the real acceptance region.
  void setGoalTolerances(const ayanami::PoseTolerance& tolerance) override;

  /// @brief Equivalent of nav2 computeVelocityCommands (abstract signature; locks and reads the map internally).
  ResultT<void> computeVelocityCommand(
      const ayanami::geometry_msgs::PoseStamped& pose,
      const ayanami::geometry_msgs::TwistStamped& velocity,
      ayanami::geometry_msgs::TwistStamped& cmd) override;

  /// @brief computeVelocityCommand() body with the local map already read-locked
  /// (moved out verbatim so the public entry point is only the read scope).
  ResultT<void> computeVelocityCommandWithMap(
      const ayanami::geometry_msgs::PoseStamped& pose,
      const ayanami::geometry_msgs::TwistStamped& velocity,
      ayanami::geometry_msgs::TwistStamped& cmd,
      ayanami::geometry_msgs::TwistStamped& cmd_vel);


  bool cancel() override { return false; }

  /// @brief Score a given command. Can be used for testing.
  virtual dwb_msgs::TrajectoryScore scoreTrajectory(
      const dwb_msgs::Trajectory2D& traj, double best_score = -1);

  /// @brief Limits the maximum linear speed of the robot.
  void setSpeedLimit(const double& speed_limit, const bool& percentage) {
    if (traj_generator) {
      traj_generator->setSpeedLimit(speed_limit, percentage);
    }
  }

protected:
  /// @brief 2D core (the 2D version of nav2 computeVelocityCommands).
  virtual nav_2d_msgs::Twist2DStamped computeVelocityCommands(
      const nav_2d_msgs::Pose2DStamped& pose,
      const nav_2d_msgs::Twist2D& velocity,
      std::shared_ptr<dwb_msgs::LocalPlanEvaluation>& results);

  void prepareGlobalPlan(const nav_2d_msgs::Pose2DStamped& pose,
                         nav_2d_msgs::Path2D& transformed_plan,
                         nav_2d_msgs::Pose2DStamped& goal_pose,
                         bool publish_plan = true);

  virtual dwb_msgs::TrajectoryScore coreScoringAlgorithm(
      const ::geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D velocity,
      std::shared_ptr<dwb_msgs::LocalPlanEvaluation>& results);

  virtual nav_2d_msgs::Path2D transformGlobalPlan(
      const nav_2d_msgs::Pose2DStamped& pose);

  /// @brief Equivalent of nav2 resolveCriticClassName (keeps the suffix/namespace completion logic).
  std::string resolveCriticClassName(std::string base_name);

  virtual void loadCritics();

  /// @brief Module parameters (json section = controller.name; read at configure time via
  /// the ModuleContext-injected own configuration and copied into members).
  struct CostmapDwbControllerParam {
    double transform_tolerance{0.1};
    bool prune_plan{true};
    double prune_distance{2.0};
    double forward_prune_distance{2.0};
    bool debug_trajectory_details{false};
    bool short_circuit_trajectory_evaluation{true};
    bool shorten_transformed_plan{true};
    std::string trajectory_generator_name{
        "dwb_plugins::StandardTrajectoryGenerator"};
    std::vector<std::string> default_critic_namespaces;
    std::vector<std::string> critics;
  };

  nav_2d_msgs::Path2D global_plan;  ///< Saved Global Plan
  CostmapDwbControllerParam param;

  /// @brief This module's configuration (moved in from ModuleContext at configure time; loadCritics
  /// reuses the same instance; no thread_local injection).
  std::unique_ptr<ayanami::Config> module_config;

  std::vector<TrajectoryCritic::Ptr> critics;

  // Module handling (static registry)
  TrajectoryGenerator::Ptr traj_generator;

  std::string dwb_module_name;

  // ayanami injections (const pointer to the map module + lock-held read view for the current cycle)
  ayanami::costmap::CostmapMapContext::ConstPtr map_context;
  const costmap_2d::Costmap2D* local_grid{nullptr};
  std::string local_frame;
  std::vector<::geometry_msgs::Point> footprint_spec;
};

}  // namespace modules
}  // namespace ayanami
