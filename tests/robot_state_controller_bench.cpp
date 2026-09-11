// RobotStateController TF query performance benchmark.
//
// Scenario (production-shaped, no roscore needed):
// - 10 TF edges, each published at 500 Hz (aggregate 5000 msg/s), forming a
//   10-hop chain map<-f1<-...<-f10 (worst-case composition depth).
// - ONE writer thread ingests all edges per 2 ms cycle (mirrors the
//   TF2Listener spin_thread that owns /tf ingestion in production).
// - Query threads call the REAL bridge entry
//   RobotStateController::lookupTransform (including its internal locking
//   and the injected std::function hop) in two modes:
//     * latest queries (the dominant production case),
//     * stamp queries with 0..2 ms age jitter (sensor-stamp semantics; the
//       simulated store accepts a query within a +-4 ms advance/hold
//       tolerance of an edge's samples, mirroring transform_tolerance).
// - Gates (the optimization loop target):
//     success rate > 98 %, mean latency < 3 ms, p99 latency < 3 ms.

#include <gtest/gtest.h>

#include <iostream>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/messages/timestamp.hpp>

namespace {

using Clock = std::chrono::steady_clock;
using ayanami::Timestamp;

constexpr int k_edges = 10;
constexpr int k_edge_period_ms = 2;          // 500 Hz per edge
constexpr int k_query_threads = 3;
constexpr int k_query_period_ms = 5;         // 200 Hz per thread (~600 qps)
constexpr double k_measure_seconds = 5.0;
constexpr double k_warmup_seconds = 0.5;
constexpr double k_min_success_rate = 0.98;  // gate 1
constexpr double k_max_latency_ms = 3.0;     // gate 2 (mean AND p99)
constexpr int64_t k_advance_tolerance_ns = 4000000LL;  // 4 ms future allowance
constexpr std::size_t k_ring = 64;                     // ~128 ms of history

int64_t nowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             Clock::now().time_since_epoch())
      .count();
}

/// @brief Simulated tf2-like store: one shared_mutex (tf2 keeps a single
/// buffer lock), per-edge stamp ring, chain composition. Writers append
/// stamp-ordered samples; readers compose target<-source over the chain.
class SimTfSource {
 public:
  void configure(const std::vector<std::pair<std::string, std::string>>& chain) {
    edges_.reserve(chain.size());
    for (const auto& [parent, child] : chain) {
      Edge e;
      e.parent = parent;
      e.child = child;
      edges_.push_back(std::move(e));
    }
  }

  /// @brief Producer side: append one sample for chain position `pos`.
  void publish(unsigned pos, int64_t stamp_ns,
               const Eigen::Isometry3d& transform) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    Edge& e = edges_[pos];
    if (e.samples.size() < k_ring) {
      e.samples.push_back(Sample{stamp_ns, transform});
    } else {
      e.samples[e.head] = Sample{stamp_ns, transform};
      e.head = (e.head + 1) % e.samples.size();
    }
    newest_stamp_.store(stamp_ns, std::memory_order_release);
  }

  int64_t newestStamp() const {
    return newest_stamp_.load(std::memory_order_acquire);
  }

  /// @brief tf2-shaped query: compose target<-source at `stamp`. Each chain
  /// edge must cover the stamp within [oldest, newest + advance]; the value
  /// comes from the nearest sample (tf2 interpolation stand-in).
  bool lookupTransform(const std::string& target, const std::string& source,
                       const Timestamp& stamp, Eigen::Isometry3d& out) const {
    const int64_t q = stamp.isLatest() ? newestStamp() : stamp.count();
    std::shared_lock<std::shared_mutex> lock(mutex_);
    // Walk the chain: target<-f1, f1<-f2, ...
    Eigen::Isometry3d acc = Eigen::Isometry3d::Identity();
    std::string current = target;
    for (const auto& edge : edges_) {
      if (edge.parent != current) {
        continue;
      }
      const Sample* nearest = nearestWithin(edge, q);
      if (nearest == nullptr) {
        return false;  // extrapolation beyond tolerance -> tf2 would fail
      }
      acc = acc * nearest->tf;
      current = edge.child;
      if (current == source) {
        out = acc;
        return true;
      }
    }
    return false;
  }

 private:
  struct Sample {
    int64_t stamp_ns;
    Eigen::Isometry3d tf;
  };
  struct Edge {
    std::string parent;
    std::string child;
    std::vector<Sample> samples;
    std::size_t head{0};
  };

  static const Sample* nearestWithin(const Edge& e, int64_t q) {
    const Sample* best = nullptr;
    int64_t best_distance = INT64_MAX;
    for (const auto& s : e.samples) {
      const int64_t distance = std::llabs(s.stamp_ns - q);
      if (distance < best_distance) {
        best = &s;
        best_distance = distance;
      }
    }
    if (best == nullptr || best_distance > k_advance_tolerance_ns) {
      return nullptr;
    }
    return best;
  }

  std::vector<Edge> edges_;
  std::atomic<int64_t> newest_stamp_{0};
  mutable std::shared_mutex mutex_;
};

/// @brief Query-side counters.
struct QueryStats {
  std::atomic<uint64_t> total{0};
  std::atomic<uint64_t> success{0};
  std::mutex latency_mutex;        // guards the latency samples
  std::vector<double> latency_ms;  // successful-query latencies, merged at end
};

}  // namespace

TEST(RobotStateControllerBench, tenEdges500HzSuccessAndLatencyGates) {
  // ---- 1. Build the chain: map <- f1 <- ... <- f10 ----
  std::vector<std::pair<std::string, std::string>> chain;
  chain.emplace_back("map", "f1");
  for (int i = 1; i < k_edges; ++i) {
    chain.emplace_back("f" + std::to_string(i), "f" + std::to_string(i + 1));
  }
  const std::string deepest = "f" + std::to_string(k_edges);

  SimTfSource sim;
  sim.configure(chain);

  // ---- 2. Install through the REAL bridge ----
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", deepest, "base_link", 0.25);
  rsc.setTransformSource(
      [&sim](const std::string& target, const std::string& source,
             const Timestamp& stamp, Eigen::Isometry3d& transform,
             double /*timeout: the sim is zero-wait, like a warm buffer*/) {
        return sim.lookupTransform(target, source, stamp, transform);
      });

  // ---- 3. Producer: one writer, all k_edges per 2 ms cycle (500 Hz/edge) ----
  std::atomic<bool> running{true};
  std::thread writer([&] {
    while (running.load(std::memory_order_relaxed)) {
      const auto cycle_start = Clock::now();
      const int64_t stamp = nowNs();
      for (unsigned pos = 0; pos < static_cast<unsigned>(k_edges); ++pos) {
        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        tf.linear() =
            Eigen::AngleAxisd(1e-6 * static_cast<double>(stamp % 1000),
                              Eigen::Vector3d::UnitZ())
                .toRotationMatrix();
        sim.publish(pos, stamp, tf);
      }
      std::this_thread::sleep_until(cycle_start +
                                    std::chrono::milliseconds(k_edge_period_ms));
    }
  });

  // ---- 4. Warm-up (buffer fills, chains resolve) ----
  std::this_thread::sleep_for(std::chrono::duration<double>(k_warmup_seconds));
  {
    Eigen::Isometry3d out;
    ASSERT_TRUE(rsc.lookupTransform("map", deepest, Timestamp::latest(), out))
        << "chain must resolve after warm-up";
  }

  // ---- 5. Query threads: latest + stamp-jitter modes, measured ----
  QueryStats stats;
  std::vector<std::thread> queries;
  for (int t = 0; t < k_query_threads; ++t) {
    queries.emplace_back([&, t] {
      uint64_t seed = 0x9e3779b97f4a7c15ull *
                      static_cast<uint64_t>(t + 1);
      auto next_jitter = [&seed] {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        return static_cast<double>(seed >> 11) / 9007199254740992.0;
      };
      std::vector<double> local_latencies;
      local_latencies.reserve(4000);
      const auto deadline =
          Clock::now() + std::chrono::duration<double>(k_measure_seconds);
      while (Clock::now() < deadline) {
        // All queries compose the full 10-hop chain; half use the latest
        // sentinel, half use a stamp with 0..2 ms age jitter (sensor stamps).
        const bool latest_mode = (t % 2 == 0);
        const Timestamp stamp =
            latest_mode
                ? Timestamp::latest()
                : Timestamp(static_cast<int64_t>(
                      sim.newestStamp() -
                      static_cast<int64_t>(next_jitter() * 2000000.0)));
        const auto t0 = Clock::now();
        Eigen::Isometry3d out;
        const bool ok =
            rsc.lookupTransform("map", deepest, stamp, out);
        const double ms =
            std::chrono::duration<double, std::milli>(Clock::now() - t0)
                .count();
        stats.total.fetch_add(1, std::memory_order_relaxed);
        if (ok) {
          stats.success.fetch_add(1, std::memory_order_relaxed);
          local_latencies.push_back(ms);
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(k_query_period_ms));
      }
      std::lock_guard<std::mutex> lock(stats.latency_mutex);
      stats.latency_ms.insert(stats.latency_ms.end(),
                              local_latencies.begin(), local_latencies.end());
    });
  }

  for (auto& q : queries) {
    q.join();
  }
  running.store(false);
  writer.join();

  // ---- 6. Metrics + gates ----
  const uint64_t total = stats.total.load();
  const uint64_t success = stats.success.load();
  ASSERT_GT(total, 100u) << "benchmark ran too few queries";
  const double success_rate = static_cast<double>(success) /
                              static_cast<double>(total);

  std::vector<double> lat = stats.latency_ms;
  std::sort(lat.begin(), lat.end());
  const auto pct = [&](double p) {
    if (lat.empty()) {
      return 0.0;
    }
    const std::size_t idx = std::min(
        lat.size() - 1,
        static_cast<std::size_t>(p * static_cast<double>(lat.size())));
    return lat[idx];
  };
  double mean = 0.0;
  for (const double v : lat) mean += v;
  mean = lat.empty() ? 0.0 : mean / static_cast<double>(lat.size());
  const double max_ms = lat.empty() ? 0.0 : lat.back();

  std::cout << "[BENCH] queries=" << total << " success_rate="
            << success_rate * 100.0 << "% mean=" << mean << "ms p50="
            << pct(0.50) << "ms p99=" << pct(0.99) << "ms max=" << max_ms
            << "ms" << std::endl;

  EXPECT_GT(success_rate, k_min_success_rate)
      << "TF query success rate below gate";
  EXPECT_LT(mean, k_max_latency_ms) << "mean latency above gate";
  EXPECT_LT(pct(0.99), k_max_latency_ms) << "p99 latency above gate";
}


// ---- Stress variant: 6 query threads @ 1 ms (~6000 qps, 10x the gate
// configuration) to probe the design headroom beyond the gates. ----
TEST(RobotStateControllerBench, stressTenTimesQueryPressureStillHolds) {
  std::vector<std::pair<std::string, std::string>> chain;
  chain.emplace_back("map", "f1");
  for (int i = 1; i < k_edges; ++i) {
    chain.emplace_back("f" + std::to_string(i), "f" + std::to_string(i + 1));
  }
  const std::string deepest = "f" + std::to_string(k_edges);

  SimTfSource sim;
  sim.configure(chain);
  auto& rsc = ayanami::components::RobotStateController::instance();
  rsc.initialize("map", deepest, "base_link", 0.25);
  rsc.setTransformSource(
      [&sim](const std::string& target, const std::string& source,
             const Timestamp& stamp, Eigen::Isometry3d& transform, double) {
        return sim.lookupTransform(target, source, stamp, transform);
      });

  std::atomic<bool> running{true};
  std::thread writer([&] {
    while (running.load(std::memory_order_relaxed)) {
      const auto cycle_start = Clock::now();
      const int64_t stamp = nowNs();
      for (unsigned pos = 0; pos < static_cast<unsigned>(k_edges); ++pos) {
        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        sim.publish(pos, stamp, tf);
      }
      std::this_thread::sleep_until(cycle_start +
                                    std::chrono::milliseconds(k_edge_period_ms));
    }
  });
  std::this_thread::sleep_for(std::chrono::duration<double>(k_warmup_seconds));

  QueryStats stats;
  constexpr int k_stress_threads = 6;
  std::vector<std::thread> queries;
  for (int t = 0; t < k_stress_threads; ++t) {
    queries.emplace_back([&] {
      std::vector<double> local_latencies;
      local_latencies.reserve(12000);
      const auto deadline =
          Clock::now() + std::chrono::duration<double>(k_measure_seconds);
      while (Clock::now() < deadline) {
        const auto t0 = Clock::now();
        Eigen::Isometry3d out;
        const bool ok = rsc.lookupTransform("map", deepest,
                                            Timestamp::latest(), out);
        const double ms =
            std::chrono::duration<double, std::milli>(Clock::now() - t0)
                .count();
        stats.total.fetch_add(1, std::memory_order_relaxed);
        if (ok) {
          stats.success.fetch_add(1, std::memory_order_relaxed);
          local_latencies.push_back(ms);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      std::lock_guard<std::mutex> lock(stats.latency_mutex);
      stats.latency_ms.insert(stats.latency_ms.end(),
                              local_latencies.begin(), local_latencies.end());
    });
  }
  for (auto& q : queries) q.join();
  running.store(false);
  writer.join();

  const uint64_t total = stats.total.load();
  const uint64_t success = stats.success.load();
  ASSERT_GT(total, 1000u);
  const double success_rate = static_cast<double>(success) /
                              static_cast<double>(total);
  std::vector<double> lat = stats.latency_ms;
  std::sort(lat.begin(), lat.end());
  const auto pct = [&](double p) {
    if (lat.empty()) return 0.0;
    return lat[std::min(lat.size() - 1,
                        static_cast<std::size_t>(p * lat.size()))];
  };
  double mean = 0.0;
  for (const double v : lat) mean += v;
  mean = lat.empty() ? 0.0 : mean / static_cast<double>(lat.size());
  std::cout << "[STRESS] queries=" << total << " success_rate="
            << success_rate * 100.0 << "% mean=" << mean << "ms p50="
            << pct(0.50) << "ms p99=" << pct(0.99) << "ms max=" << lat.back()
            << "ms" << std::endl;

  // Same gates under 10x query pressure.
  EXPECT_GT(success_rate, k_min_success_rate);
  EXPECT_LT(mean, k_max_latency_ms);
  EXPECT_LT(pct(0.99), k_max_latency_ms);
}
