#include <ayanami/modules/simple_progress_checker/simple_progress_checker.hpp>

extern "C" ayanami::abstract::AbstractProgressChecker*
create_progress_checker_module() {
  return new ayanami::modules::SimpleProgressChecker();
}