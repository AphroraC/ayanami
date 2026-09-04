#include <gtest/gtest.h>
#include <ayanami/utility/callback_slot.hpp>

#include <atomic>
#include <cstdlib>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<long long> g_allocation_count{0};
std::atomic<bool> g_counting_enabled{false};

// Deterministic allocation count check (no timing): only allocations performed
// while g_counting_enabled is true are counted, and they happen on the single
// test thread that brackets the loop.
void* counting_new(std::size_t size) {
  if (void* p = std::malloc(size)) {
    if (g_counting_enabled.load(std::memory_order_relaxed)) {
      g_allocation_count.fetch_add(1, std::memory_order_relaxed);
    }
    return p;
  }
  throw std::bad_alloc();
}

struct CountingGuard {
  long long count;
  CountingGuard() : count(0) {
    g_counting_enabled.store(true, std::memory_order_relaxed);
    g_allocation_count.store(0, std::memory_order_relaxed);
  }
  ~CountingGuard() {
    g_counting_enabled.store(false, std::memory_order_relaxed);
    count = g_allocation_count.load(std::memory_order_relaxed);
  }
};

std::atomic<int> g_exception_reports{0};

}  // namespace

void* operator new(std::size_t size) { return counting_new(size); }
void* operator new[](std::size_t size) { return counting_new(size); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t size, std::align_val_t al) {
  void* p = std::aligned_alloc(static_cast<std::size_t>(al), (size + static_cast<std::size_t>(al) - 1) & ~(static_cast<std::size_t>(al) - 1));
  if (!p) throw std::bad_alloc();
  if (g_counting_enabled.load(std::memory_order_relaxed)) {
    g_allocation_count.fetch_add(1, std::memory_order_relaxed);
  }
  return p;
}
void* operator new[](std::size_t size, std::align_val_t al) {
  return ::operator new(size, al);
}
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

TEST(CallbackSlotTest, addCallRemove) {
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
TEST(CallbackSlotTest, threadSafe) {
  ayanami::CallbackSlot<void(int)> slot;
  std::atomic<int> hits{0};
  slot.add([&](int){ hits++; });
  std::vector<std::thread> ts;
  for (int i=0;i<8;++i) ts.emplace_back([&]{ for (int j=0;j<1000;++j) slot(j); });
  for (auto& t: ts) t.join();
  EXPECT_EQ(hits.load(), 8000);
}
TEST(CallbackSlotTest, emptySlotIsFalse) {
  ayanami::CallbackSlot<void()> slot;
  EXPECT_FALSE(bool(slot));
  slot.add([]{});
  EXPECT_TRUE(bool(slot));
}

TEST(CallbackSlotTest, callbackCanRemoveItself) {
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

TEST(CallbackSlotTest, callbackCanEmitRecursively) {
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

TEST(CallbackSlotTest, removalTakesEffectOnNextSnapshot) {
  ayanami::CallbackSlot<void()> slot;
  int second_calls = 0;
  int second_id = -1;
  slot.add([&] { slot.remove(second_id); });
  second_id = slot.add([&] { ++second_calls; });

  slot();
  slot();

  EXPECT_EQ(second_calls, 1);
}

TEST(CallbackSlotTest, idsAreStableAndHolesAreNotReused) {
  ayanami::CallbackSlot<void()> slot;
  std::string order;
  int a = slot.add([&]{ order += "a"; });
  int b = slot.add([&]{ order += "b"; });
  int c = slot.add([&]{ order += "c"; });
  EXPECT_EQ(a, 0);
  EXPECT_EQ(b, 1);
  EXPECT_EQ(c, 2);

  slot.remove(b);
  int d = slot.add([&]{ order += "d"; });
  EXPECT_EQ(d, 3);  // ids keep increasing; a hole is left at 1

  slot();  // a, c, d run; the hole at 1 is skipped
  EXPECT_EQ(order, "acd");

  slot.remove(c);
  slot();  // a, d
  EXPECT_EQ(order, "acdad");
}

TEST(CallbackSlotTest, doubleRemoveAndInvalidIdsAreNoops) {
  ayanami::CallbackSlot<void()> slot;
  int calls = 0;
  int id = slot.add([&]{ ++calls; });
  slot.remove(id);
  slot.remove(id);  // double remove
  slot.remove(-1);  // negatives
  slot.remove(999); // out of range
  slot();
  EXPECT_EQ(calls, 0);
  EXPECT_FALSE(bool(slot));
}

TEST(CallbackSlotTest, reentrantAddOnlyVisibleOnNextEmit) {
  ayanami::CallbackSlot<void()> slot;
  std::string order;
  bool added = false;
  slot.add([&] {
    order += "outer";
    if (!added) {
      added = true;
      slot.add([&]{ order += "inner"; });
    }
  });

  slot();  // newly added callback must not run during this emission
  EXPECT_EQ(order, "outer");
  slot();  // outer then inner: the mid-emit add is now visible
  EXPECT_EQ(order, "outerouterinner");
}

TEST(CallbackSlotTest, emitSnapshotImmutableToMidEmitRemovalAndAdd) {
  ayanami::CallbackSlot<void()> slot;
  std::string order;
  int doomed = -1;
  bool armed = true;
  slot.add([&] {
    if (armed) {
      armed = false;
      slot.remove(doomed);          // in-flight snapshot still contains doomed
      slot.add([&]{ order += "D"; });  // only visible from the next emission
    }
    order += "A";
  });
  doomed = slot.add([&]{ order += "B"; });

  slot();  // A, then B (doomed removed mid-emit, D not yet visible)
  EXPECT_EQ(order, "AB");

  slot();  // A again, then D (B gone, D now visible)
  EXPECT_EQ(order, "ABAD");
}

TEST(CallbackSlotTest, throwingObserverDoesNotInterruptRemainingObservers) {
  ayanami::CallbackSlot<void()> slot;
  std::string order;
  slot.add([&]{ order += "1"; throw std::runtime_error("obs1"); });
  slot.add([&]{ order += "2"; });
  slot.add([&]{ order += "3"; throw std::runtime_error("obs3"); });

  const ayanami::callback_exception_handler prev =
      ayanami::getCallbackExceptionHandler();
  ayanami::setCallbackExceptionHandler(
      +[](std::exception_ptr) noexcept { ++g_exception_reports; });
  g_exception_reports.store(0);

  slot();  // must not propagate

  ayanami::setCallbackExceptionHandler(prev);
  EXPECT_EQ(order, "123");
  EXPECT_EQ(g_exception_reports.load(), 2);
  EXPECT_TRUE(bool(slot));
}

TEST(CallbackSlotTest, noAllocationOnHotEmitPath) {
  ayanami::CallbackSlot<void(int)> slot;
  long long total = 0;
  slot.add([&](int v){ total += v; });
  slot.add([&](int v){ total += v * 2; });

  slot(1);  // warm-up: snapshot is fully published

  long long allocs = 0;
  {
    CountingGuard guard;
    for (int i = 0; i < 100000; ++i) {
      slot(i);
    }
    allocs = guard.count;
  }

  EXPECT_EQ(allocs, 0);
  EXPECT_GT(total, 0);
}

TEST(CallbackSlotTest, concurrentEmitAndMutateIsSafe) {
  ayanami::CallbackSlot<void(int)> slot;
  std::atomic<bool> start{false};
  std::atomic<long long> hits{0};
  std::atomic<bool> ids_monotonic{true};

  std::vector<std::thread> emitters;
  for (int t = 0; t < 4; ++t) {
    emitters.emplace_back([&] {
      while (!start.load(std::memory_order_acquire)) {}
      for (int i = 0; i < 5000; ++i) {
        slot(1);
      }
    });
  }

  std::thread mutator([&] {
    int last_id = -1;
    while (!start.load(std::memory_order_acquire)) {}
    for (int i = 0; i < 1000; ++i) {
      int id = slot.add([&](int){ hits.fetch_add(1); });
      if (id <= last_id) ids_monotonic.store(false);
      last_id = id;
      slot.remove(id);
    }
  });

  start.store(true, std::memory_order_release);
  for (auto& t : emitters) t.join();
  mutator.join();

  EXPECT_TRUE(ids_monotonic.load());
  EXPECT_GE(hits.load(), 0);
  EXPECT_FALSE(bool(slot));
}

namespace {
struct Payload {
  int x{0};
  double y{0.0};
};
}  // namespace

// 右值防护放行路径: trivially-copyable 按值实参 (move == copy), 每个 observer
// 都拿到完整值, 不会出现第一个 observer 搬空参数的情况。
TEST(CallbackSlotTest, trivialByValueArgumentReachesAllObservers) {
  ayanami::CallbackSlot<void(Payload)> slot;
  std::vector<int> seen;
  slot.add([&](Payload p) { seen.push_back(p.x); });
  slot.add([&](Payload p) { seen.push_back(p.x); });
  slot.add([&](Payload p) { seen.push_back(p.x); });

  slot(Payload{7, 1.5});

  ASSERT_EQ(seen.size(), 3u);
  EXPECT_EQ(seen[0], 7);
  EXPECT_EQ(seen[1], 7);
  EXPECT_EQ(seen[2], 7);
}

// 非平凡类型以左值传入 (Args 推为引用), 不触碰 move-from 防护, 全部 observer
// 可见完整内容。
TEST(CallbackSlotTest, nontrivialLvalueArgumentReachesAllObservers) {
  ayanami::CallbackSlot<void(const std::string&)> slot;
  std::string order;
  slot.add([&](const std::string& s) { order += s; });
  slot.add([&](const std::string& s) { order += s; });

  const std::string msg = "ok";
  slot(msg);

  EXPECT_EQ(order, "okok");
}

TEST(CallbackSlotTest, subscriptionUnregistersOnDestruction) {
  ayanami::CallbackSlot<void()> slot;
  int calls = 0;
  {
    auto subscription = slot.subscribe([&] { ++calls; });
    slot();
    EXPECT_EQ(calls, 1);
  }
  slot();
  EXPECT_EQ(calls, 1);
}
