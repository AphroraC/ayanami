// exceptions.cpp — IllegalTrajectoryTracker 实现 (迁移自
// dwb_core/src/illegal_trajectory_tracker.cpp)。
#include "exceptions.hpp"

#include <map>
#include <sstream>
#include <string>
#include <utility>

namespace dwb_core {

void IllegalTrajectoryTracker::addIllegalTrajectory(
    const dwb_core::IllegalTrajectoryException& e) {
  counts_[std::make_pair(e.getCriticName(), e.what())]++;
  illegal_count_++;
}

void IllegalTrajectoryTracker::addLegalTrajectory() { legal_count_++; }

std::map<std::pair<std::string, std::string>, double>
IllegalTrajectoryTracker::getPercentages() const {
  std::map<std::pair<std::string, std::string>, double> percents;
  double denominator = static_cast<double>(legal_count_ + illegal_count_);
  for (auto const& x : counts_) {
    percents[x.first] = static_cast<double>(x.second) / denominator;
  }
  return percents;
}

std::string IllegalTrajectoryTracker::getMessage() const {
  std::ostringstream msg;
  if (legal_count_ == 0) {
    msg << "No valid trajectories out of " << illegal_count_ << "! ";
  } else {
    unsigned int total = legal_count_ + illegal_count_;
    msg << legal_count_ << " valid trajectories found (";
    msg << static_cast<double>(100 * legal_count_) /
               static_cast<double>(total);
    msg << "% of " << total << "). ";
  }
  return msg.str();
}

}  // namespace dwb_core
