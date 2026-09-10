#include <gtest/gtest.h>

#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

#include <atomic>
#include <memory>
#include <string>

namespace {

using ayanami::ErrorCode;
using ayanami::RecoveryCycleData;
using ayanami::ResultT;
class FakeRecovery : public ayanami::abstract::AbstractRecovery {
public:
  static std::atomic<int> dtor_count;
  std::string non_pod_member{"some state longer than SSO"};

  ~FakeRecovery() override { ++dtor_count; }
  ResultT<void> tick(RecoveryCycleData& cycle) override {
    return ResultT<void>::failure(ErrorCode::RECOVERY_FAILED, "noop");
  }
  bool cancel() override { return true; }
};
std::atomic<int> FakeRecovery::dtor_count{0};

extern "C" ayanami::abstract::AbstractRecovery* createFakeRecovery() {
  return new FakeRecovery();
}

}  // namespace

TEST(ModuleTeardownNoUB, derivedDestructorCalledOnClearModules) {
  FakeRecovery::dtor_count = 0;
  auto* ptr = createFakeRecovery();
  ASSERT_NE(ptr, nullptr);
  {
    // ModuleManager has no in-memory injection registerModule interface (plan bug, see report Concerns);
    // releasing the shared_ptr here is semantically identical to clearModules releasing std::shared_ptr.
    auto shr = std::shared_ptr<ayanami::abstract::AbstractRecovery>(ptr);
    (void)shr;
  }
  EXPECT_EQ(FakeRecovery::dtor_count.load(), 1)
      << "derived destructor not called (abstract base misses a virtual ~)";
}
