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
