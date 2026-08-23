#include "simple_goal_checker.hpp"

extern "C" ayanami::abstract::AbstractGoalChecker* create_goal_checker_module() {
  return new ayanami::modules::SimpleGoalChecker();
}