// registration.hpp -- static registries for critics / trajectory generators (the ayanami
// replacement for pluginlib: all sub-plugins are compiled into the module .so, looked up by short class name).
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "trajectory_critic.hpp"
#include "trajectory_generator.hpp"

namespace ayanami {
namespace modules {

/// @brief Critic factory registry (key: short class name, e.g. "BaseObstacleCritic").
std::map<std::string, std::function<dwb_core::TrajectoryCritic::Ptr()>>&
criticRegistry();

/// @brief Trajectory generator factory registry (keys: "StandardTrajectoryGenerator"
/// / "LimitedAccelGenerator").
std::map<std::string, std::function<dwb_core::TrajectoryGenerator::Ptr()>>&
trajectoryGeneratorRegistry();

/// @brief Registers all built-in critics (11; the base class is not instantiable).
void registerAllCritics();

/// @brief Registers all built-in trajectory generators (2).
void registerAllTrajectoryGenerators();

}  // namespace modules
}  // namespace ayanami
