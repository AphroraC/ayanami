#include "abstract_nothing_recovery.hpp"

#include <spdlog/spdlog.h>

#include <ayanami/utility/logging.hpp>

namespace ayanami {
namespace modules {

ResultT<void> NothingRecovery::tick(RecoveryCycleData& cycle) {
  SPDLOG_LOGGER_INFO(
    ayanami::recoveryLogger(),
    "NothingRecovery: invoked (recovery placeholder); no action taken, "
    "reporting SUCCESS");
  cycle.status = RecoveryStatus::SUCCESS;
  return ResultT<void>::success();
}

}  // namespace modules
}  // namespace ayanami
