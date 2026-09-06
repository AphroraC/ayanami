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

// 由 main 从 argv 读取 (ctest 传 $<TARGET_FILE:echo_extension> 路径)
std::string g_echo_so;

} // namespace

TEST(NavigationCallbacksTest, addAndCallAcrossCompilationUnits) {
  // 槽对象实体定义在 libayanami 的 src/interfaces/callbacks.cpp (不 inline),
  // 测试编译单元 (link libayanami) 引用到同一实例 → 跨编译单元一致。
  int received = 0;
  const int id = NavigationCallbacks::on_get_path_result.add(
      [&](const auto&){ received++; });
  // 右值防护: 非平凡类型必须传左值, 避免多观察者下第一个 observer 搬空参数。
  const GoalResult<GetPathResult> result{};
  NavigationCallbacks::on_get_path_result(result);
  EXPECT_EQ(received, 1);
  NavigationCallbacks::on_get_path_result.remove(id);
}

TEST(NavigationCallbacksTest, threadSafeAcrossAddRemoveCall) {
  // 同 的并发测试, 但挂在 NavigationCallbacks 槽上。
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
  // RAII 生命周期: CallbackSubscription 析构即退订, 其后的 emit 不再送达;
  // 移动赋值只保留最新一份订阅 (旧的被替换退订)。
  std::atomic<int> hits{0};
  {
    ayanami::CallbackSubscription sub =
        NavigationCallbacks::on_get_path_result.subscribe(
            [&](const auto&) { hits++; });
    GoalResult<GetPathResult> result{};
    NavigationCallbacks::on_get_path_result(result);
    EXPECT_EQ(hits.load(), 1);
  }  // sub 析构 → 退订
  GoalResult<GetPathResult> result{};
  NavigationCallbacks::on_get_path_result(result);
  EXPECT_EQ(hits.load(), 1);

  // 移动赋值: 旧订阅退订, 新订阅生效。
  ayanami::CallbackSubscription outer;
  {
    ayanami::CallbackSubscription first =
        NavigationCallbacks::on_get_path_result.subscribe(
            [&](const auto&) { hits++; });
    NavigationCallbacks::on_get_path_result(result);
    EXPECT_EQ(hits.load(), 2);
    outer = std::move(first);  // first 变空, 订阅转移给 outer
  }
  NavigationCallbacks::on_get_path_result(result);
  EXPECT_EQ(hits.load(), 3);
  outer.reset();  // 显式退订 (等价析构)
  NavigationCallbacks::on_get_path_result(result);
  EXPECT_EQ(hits.load(), 3);
}

TEST(NavigationCallbacksTest, callbackVisibleAcrossSoBoundary) {
  // dlopen echo_extension.so (链接 libayanami): 其构造期 add 到
  // NavigationCallbacks::on_get_path_result。本测试 emit 槽后, echo .so
  // 内部的计数必须增加 —— 证明槽对象是 libayanami 内的同一实例
  // (关键纪律, 不能用 inline static)。
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
