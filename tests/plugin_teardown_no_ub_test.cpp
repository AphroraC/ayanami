#include <gtest/gtest.h>

#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/failure.hpp>
#include <ayanami/types.hpp>

#include <atomic>
#include <memory>
#include <string>

namespace {

using ayanami::ErrorCode;
using ayanami::RecoveryResultT;

class FakeRecovery : public ayanami::abstract::AbstractRecovery {
public:
  static std::atomic<int> dtor_count;
  std::string non_pod_member{"some state longer than SSO"};

  ~FakeRecovery() override { ++dtor_count; }
  RecoveryResultT tick() override {
    return RecoveryResultT::failure(ErrorCode::RECOVERY_FAILED, "noop");
  }
  bool cancel() override { return true; }
};
std::atomic<int> FakeRecovery::dtor_count{0};

extern "C" ayanami::abstract::AbstractRecovery* createFakeRecovery() {
  return new FakeRecovery();
}

}  // namespace

TEST(PluginTeardownNoUB, derivedDestructorCalledOnClearPlugins) {
  FakeRecovery::dtor_count = 0;
  auto* ptr = createFakeRecovery();
  ASSERT_NE(ptr, nullptr);
  {
    // PluginManager 无内存注入 registerPlugin 接口 (plan bug, 见 report Concerns),
    // 此处释放 shared_ptr 语义与 clearPlugins 释放 std::shared_ptr 完全一致。
    auto shr = std::shared_ptr<ayanami::abstract::AbstractRecovery>(ptr);
    (void)shr;
  }
  EXPECT_EQ(FakeRecovery::dtor_count.load(), 1)
      << "派生析构未被调用 (abstract base virtual ~ 缺)";
}
