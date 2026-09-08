// exceptions.cpp -- IllegalTrajectoryTracker implementation (ported from
// dwb_core/src/illegal_trajectory_tracker.cpp).
#include "exceptions.hpp"

#include <map>
#include <sstream>
#include <string>
#include <utility>

namespace dwb_core {

void IllegalTrajectoryTracker::addIllegalTrajectory(
    const dwb_core::IllegalTrajectoryException& e) {
  counts[std::make_pair(e.getCriticName(), e.what())]++;
  illegal_count++;
}

void IllegalTrajectoryTracker::addLegalTrajectory() { legal_count++; }

std::map<std::pair<std::string, std::string>, double>
IllegalTrajectoryTracker::getPercentages() const {
  std::map<std::pair<std::string, std::string>, double> percents;
  double denominator = static_cast<double>(legal_count + illegal_count);
  for (auto const& x : counts) {
    percents[x.first] = static_cast<double>(x.second) / denominator;
  }
  return percents;
}

std::string IllegalTrajectoryTracker::getMessage() const {
  std::ostringstream msg;
  if (legal_count == 0) {
    msg << "No valid trajectories out of " << illegal_count << "! ";
  } else {
    unsigned int total = legal_count + illegal_count;
    msg << legal_count << " valid trajectories found (";
    msg << static_cast<double>(100 * legal_count) /
               static_cast<double>(total);
    msg << "% of " << total << "). ";
  }
  return msg.str();
}

}  // namespace dwb_core
