// registration.cpp -- static registry implementation.
#include "registration.hpp"

#include "critics/base_obstacle.hpp"
#include "critics/goal_align.hpp"
#include "critics/goal_dist.hpp"
#include "critics/obstacle_footprint.hpp"
#include "critics/oscillation.hpp"
#include "critics/path_align.hpp"
#include "critics/path_dist.hpp"
#include "critics/prefer_forward.hpp"
#include "critics/rotate_to_goal.hpp"
#include "critics/twirling.hpp"
#include "plugins/limited_accel_generator.hpp"
#include "plugins/standard_traj_generator.hpp"

namespace ayanami {
namespace modules {

std::map<std::string, std::function<dwb_core::TrajectoryCritic::Ptr()>>&
criticRegistry() {
  static std::map<std::string, std::function<dwb_core::TrajectoryCritic::Ptr()>>
      registry;
  return registry;
}

std::map<std::string, std::function<dwb_core::TrajectoryGenerator::Ptr()>>&
trajectoryGeneratorRegistry() {
  static std::map<std::string,
                  std::function<dwb_core::TrajectoryGenerator::Ptr()>>
      registry;
  return registry;
}

void registerAllCritics() {
  auto& registry = criticRegistry();
  registry["BaseObstacleCritic"] =
      []() -> dwb_core::TrajectoryCritic::Ptr {
    return std::make_shared<dwb_critics::BaseObstacleCritic>();
  };
  registry["ObstacleFootprintCritic"] =
      []() -> dwb_core::TrajectoryCritic::Ptr {
    return std::make_shared<dwb_critics::ObstacleFootprintCritic>();
  };
  registry["GoalDistCritic"] = []() -> dwb_core::TrajectoryCritic::Ptr {
    return std::make_shared<dwb_critics::GoalDistCritic>();
  };
  registry["PathDistCritic"] = []() -> dwb_core::TrajectoryCritic::Ptr {
    return std::make_shared<dwb_critics::PathDistCritic>();
  };
  registry["GoalAlignCritic"] = []() -> dwb_core::TrajectoryCritic::Ptr {
    return std::make_shared<dwb_critics::GoalAlignCritic>();
  };
  registry["PathAlignCritic"] = []() -> dwb_core::TrajectoryCritic::Ptr {
    return std::make_shared<dwb_critics::PathAlignCritic>();
  };
  registry["OscillationCritic"] = []() -> dwb_core::TrajectoryCritic::Ptr {
    return std::make_shared<dwb_critics::OscillationCritic>();
  };
  registry["RotateToGoalCritic"] = []() -> dwb_core::TrajectoryCritic::Ptr {
    return std::make_shared<dwb_critics::RotateToGoalCritic>();
  };
  registry["PreferForwardCritic"] = []() -> dwb_core::TrajectoryCritic::Ptr {
    return std::make_shared<dwb_critics::PreferForwardCritic>();
  };
  registry["TwirlingCritic"] = []() -> dwb_core::TrajectoryCritic::Ptr {
    return std::make_shared<dwb_critics::TwirlingCritic>();
  };
  // MapGridCritic is an abstract base class (validCellToQueue could be instantiated but is not exported, matching nav2's
  // default_critics.xml: 10 exported modules).
}

void registerAllTrajectoryGenerators() {
  auto& registry = trajectoryGeneratorRegistry();
  registry["StandardTrajectoryGenerator"] =
      []() -> dwb_core::TrajectoryGenerator::Ptr {
    return std::make_shared<dwb_plugins::StandardTrajectoryGenerator>();
  };
  registry["LimitedAccelGenerator"] =
      []() -> dwb_core::TrajectoryGenerator::Ptr {
    return std::make_shared<dwb_plugins::LimitedAccelGenerator>();
  };
}

}  // namespace modules
}  // namespace ayanami
