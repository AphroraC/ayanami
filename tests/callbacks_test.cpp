#include <gtest/gtest.h>

#include <ayanami/interfaces/callbacks.hpp>
#include <ayanami/utility/extension_module.hpp>
#include <ayanami/utility/load_module.hpp>

#include <atomic>
#include <string>
#include <thread>
#include <vector>

using namespace ayanami::interfaces;
using ayanami::actions::GetPathResult;
using ayanami::actions::GoalResult;

namespace {

// Read by main from argv (ctest passes the $<TARGET_FILE:echo_extension> path)
std::string g_echo_so;

} // namespace

TEST(NavigationCallbacksTest, addAndCallAcrossCompilationUnits) {
  // The slot object is defined out-of-line in libayanami's src/interfaces/callbacks.cpp (not inline),
  // so the test TU (which links libayanami) refers to the same instance -> consistent across TUs.
  int received = 0;
  const int id = NavigationCallbacks::on_get_path_result.add(
      [&](const auto&){ received++; });
  // Rvalue guard: non-trivial types must be passed as lvalues so the first observer cannot move the argument away.
  const GoalResult<GetPathResult> result{};
  NavigationCallbacks::on_get_path_result(result);
  EXPECT_EQ(received, 1);
  NavigationCallbacks::on_get_path_result.remove(id);
}

TEST(NavigationCallbacksTest, threadSafeAcrossAddRemoveCall) {
  // Same concurrency test, but attached to the NavigationCallbacks slot.
  std::atomic<int> hits{0};
  const int id = NavigationCallbacks::on_get_path_result.add([&](const auto&){ hits++; });
  std::vector<std::thread> ts;
  for (int i = 0; i < 8; ++i) {
    ts.emplace_back([&]{
      for (int j = 0; j < 1000; ++j) {
        GoalResult<GetPathResult> result{};
        NavigationCallbacks::on_get_path_result(result);
      }
    });
  }
  for (auto& t : ts) t.join();
  EXPECT_EQ(hits.load(), 8000);
  NavigationCallbacks::on_get_path_result.remove(id);
}

TEST(NavigationCallbacksTest, subscriptionHandleUnsubscribesOnDestruction) {
  // RAII lifetime: destroying a CallbackSubscription unsubscribes; later emits are no longer delivered;
  // move assignment keeps only the newest subscription (the old one is replaced and unsubscribed).
  std::atomic<int> hits{0};
  {
    ayanami::CallbackSubscription sub =
        NavigationCallbacks::on_get_path_result.subscribe(
            [&](const auto&) { hits++; });
    GoalResult<GetPathResult> result{};
    NavigationCallbacks::on_get_path_result(result);
    EXPECT_EQ(hits.load(), 1);
  }  // sub destructor -> unsubscribe
  GoalResult<GetPathResult> result{};
  NavigationCallbacks::on_get_path_result(result);
  EXPECT_EQ(hits.load(), 1);

  // Move assignment: the old subscription is unsubscribed, the new one takes effect.
  ayanami::CallbackSubscription outer;
  {
    ayanami::CallbackSubscription first =
        NavigationCallbacks::on_get_path_result.subscribe(
            [&](const auto&) { hits++; });
    NavigationCallbacks::on_get_path_result(result);
    EXPECT_EQ(hits.load(), 2);
    outer = std::move(first);  // first becomes empty; the subscription was moved to outer
  }
  NavigationCallbacks::on_get_path_result(result);
  EXPECT_EQ(hits.load(), 3);
  outer.reset();  // Explicit unsubscribe (equivalent to destruction)
  NavigationCallbacks::on_get_path_result(result);
  EXPECT_EQ(hits.load(), 3);
}

TEST(NavigationCallbacksTest, callbackVisibleAcrossSoBoundary) {
  // dlopen echo_extension.so (linked against libayanami): its constructor adds itself to
  // NavigationCallbacks::on_get_path_result. After this test emits the slot, the counter inside the echo .so
  // must have increased -- proving the slot object is the single instance inside libayanami
  // (key discipline: an inline static must not be used).
  auto ext = ayanami::ExtensionModule::load_module(g_echo_so);
  ASSERT_NE(ext, nullptr);
  auto hits_fn =
      ayanami::loadModuleFromSo<int(*)()>(g_echo_so, "echoExtensionHits");
  ASSERT_NE(hits_fn, nullptr);
  const int before = hits_fn();
  const GoalResult<GetPathResult> result{};
  NavigationCallbacks::on_get_path_result(result);
  EXPECT_EQ(hits_fn(), before + 1);
  NavigationCallbacks::on_get_path_result.remove(0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 1) {
    g_echo_so = argv[1];
  }
  return RUN_ALL_TESTS();
}
