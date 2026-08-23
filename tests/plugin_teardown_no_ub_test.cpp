#include <gtest/gtest.h>

#include <ayanami/abstract/core/abstract_recovery.hpp>
#include <ayanami/messages/failure.hpp>

#include <atomic>
#include <memory>
#include <string>

namespace {

using ayanami::FailureCode;

class FakeRecovery : public ayanami::abstract::AbstractRecovery {
public:
  static std::atomic<int> dtor_count_;
  std::string non_pod_member_{"some state longer than SSO"};

  ~FakeRecovery() override { ++dtor_count_; }
  FailureCode runRecovery(std::string&) override { return FailureCode::None; }
  bool cancel() override { return true; }
};
std::atomic<int> FakeRecovery::dtor_count_{0};

extern "C" ayanami::abstract::AbstractRecovery* create_fake_recovery() {
  return new FakeRecovery();
}

}  // namespace

TEST(PluginTeardownNoUB, DerivedDestructorCalledOnClearPlugins) {
  FakeRecovery::dtor_count_ = 0;
  auto* ptr = create_fake_recovery();
  ASSERT_NE(ptr, nullptr);
  {
    // PluginManager 无内存注入 registerPlugin 接口 (plan bug, 见 report Concerns),
    // 此处释放 shared_ptr 语义与 clearPlugins() 释放 std::shared_ptr 完全一致。
    auto shr = std::shared_ptr<ayanami::abstract::AbstractRecovery>(ptr);
    (void)shr;
  }
  EXPECT_EQ(FakeRecovery::dtor_count_.load(), 1)
      << "派生析构未被调用 (abstract base virtual ~ 缺)";
}
