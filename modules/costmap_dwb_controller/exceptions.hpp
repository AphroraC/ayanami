// exceptions.hpp -- DWB exception hierarchy (ported from dwb_core/exceptions.hpp +
// illegal_trajectory_tracker.hpp/cpp; nav2_core::PlannerException becomes
// std::runtime_error).
#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace dwb_core {

/**
 * @brief Base planner exception (equivalent of nav2_core::PlannerException).
 */
class PlannerException : public std::runtime_error {
public:
  explicit PlannerException(const std::string& description)
      : std::runtime_error(description) {}
};

/**
 * @brief Thrown when the planner cannot complete its operation due to TF errors
 */
class PlannerTFException : public PlannerException {
public:
  explicit PlannerTFException(const std::string description)
      : PlannerException(description) {}
};

/**
 * @brief Thrown when one of the critics encountered a fatal error
 */
class IllegalTrajectoryException : public PlannerException {
public:
  IllegalTrajectoryException(const std::string critic_name,
                             const std::string description)
      : PlannerException(description), critic_name(critic_name) {}
  std::string getCriticName() const { return critic_name; }

protected:
  std::string critic_name;
};

/**
 * @brief Tracks the failure-reason distribution of illegal trajectories (ported from illegal_trajectory_tracker).
 */
class IllegalTrajectoryTracker {
public:
  IllegalTrajectoryTracker() : legal_count(0), illegal_count(0) {}

  void addIllegalTrajectory(const IllegalTrajectoryException& e);
  void addLegalTrajectory();

  std::map<std::pair<std::string, std::string>, double> getPercentages() const;

  std::string getMessage() const;

protected:
  std::map<std::pair<std::string, std::string>, unsigned int> counts;
  unsigned int legal_count, illegal_count;
};

/**
 * @brief Thrown when all the trajectories explored are illegal
 */
class NoLegalTrajectoriesException : public PlannerException {
public:
  explicit NoLegalTrajectoriesException(const IllegalTrajectoryTracker& tracker)
      : PlannerException(tracker.getMessage()), tracker(tracker) {}
  IllegalTrajectoryTracker tracker;
};

}  // namespace dwb_core
