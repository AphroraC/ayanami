#include <gtest/gtest.h>
#include <ayanami/utility/callback_slot.hpp>

#include <atomic>
#include <thread>
#include <vector>

TEST(CallbackSlotTest, AddCallRemove) {
  ayanami::CallbackSlot<void(int)> slot;
  int sum = 0;
  int id1 = slot.add([&](int v){ sum += v; });
  int id2 = slot.add([&](int v){ sum += v*10; });
  slot(5);  // sum = 5 + 50 = 55
  EXPECT_EQ(sum, 55);
  slot.remove(id1);
  slot(5);  // sum = 55 + 50 = 105
  EXPECT_EQ(sum, 105);
}
TEST(CallbackSlotTest, ThreadSafe) {
  ayanami::CallbackSlot<void(int)> slot;
  std::atomic<int> hits{0};
  slot.add([&](int){ hits++; });
  std::vector<std::thread> ts;
  for (int i=0;i<8;++i) ts.emplace_back([&]{ for (int j=0;j<1000;++j) slot(j); });
  for (auto& t: ts) t.join();
  EXPECT_EQ(hits.load(), 8000);
}
TEST(CallbackSlotTest, EmptySlotIsFalse) {
  ayanami::CallbackSlot<void()> slot;
  EXPECT_FALSE(bool(slot));
  slot.add([]{});
  EXPECT_TRUE(bool(slot));
}

TEST(CallbackSlotTest, CallbackCanRemoveItself) {
  ayanami::CallbackSlot<void()> slot;
  int calls = 0;
  int id = -1;
  id = slot.add([&] {
    ++calls;
    slot.remove(id);
  });

  slot();
  slot();

  EXPECT_EQ(calls, 1);
  EXPECT_FALSE(bool(slot));
}

TEST(CallbackSlotTest, CallbackCanEmitRecursively) {
  ayanami::CallbackSlot<void(int)> slot;
  int calls = 0;
  slot.add([&](int depth) {
    ++calls;
    if (depth > 0) {
      slot(depth - 1);
    }
  });

  slot(2);

  EXPECT_EQ(calls, 3);
}

TEST(CallbackSlotTest, RemovalTakesEffectOnNextSnapshot) {
  ayanami::CallbackSlot<void()> slot;
  int second_calls = 0;
  int second_id = -1;
  slot.add([&] { slot.remove(second_id); });
  second_id = slot.add([&] { ++second_calls; });

  slot();
  slot();

  EXPECT_EQ(second_calls, 1);
}
