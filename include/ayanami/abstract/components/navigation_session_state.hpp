#pragma once

#include <atomic>
#include <cstdint>

namespace ayanami {
namespace components {

/**
 * @brief 导航会话/版本状态的单一来源 (BehaviorTreePilot 与 NavigationServerBase
 * 共享同一对象)。
 *
 * 背景: pilot 原先持有独立的 session_id/plan_version 原子, server 另持一份,
 * 两份计数器各自自增会发散 —— 黑板 session 与实际 goal 的 session 不一致。
 * 本对象作为单一来源: pilot 在 tick 线程 applyPendingGoal 里 startNewSession()
 * (session+1 且 plan_version 归零), server 的 currentSessionId/currentPlanVersion
 * 都委托到这里, 二者看到同一会话编号。
 *
 * 线程安全: session/plan_version 用 std::atomic, 任意线程可读; startNewSession /
 * incrementPlanVersion / sessionIsCurrent / planVersionIsCurrent 均无锁。
 */
class NavigationSessionState {
 public:
  NavigationSessionState() = default;

  /// @brief 当前会话编号 (初始 0; 每次 startNewSession +1)。
  uint64_t currentSessionId() const { return session_id.load(); }

  /// @brief 当前规划版本 (初始 0; startNewSession 归零)。
  uint64_t currentPlanVersion() const { return plan_version.load(); }

  /// @brief 开始新会话: session_id+1 且 plan_version 归零 (每轮导航起点)。
  void startNewSession() {
    session_id.fetch_add(1, std::memory_order_acq_rel);
    plan_version.store(0, std::memory_order_release);
  }

  /// @brief 规划版本自增 (平滑/重规划产生新路径时)。
  void incrementPlanVersion() {
    plan_version.fetch_add(1, std::memory_order_acq_rel);
  }

  /// @brief 结果 session 是否仍属当前会话。
  bool sessionIsCurrent(uint64_t session_id) const {
    return session_id == this->session_id.load();
  }

  /// @brief 结果 plan_version 是否仍为当前版本。
  bool planVersionIsCurrent(uint64_t plan_version) const {
    return plan_version == this->plan_version.load();
  }

 private:
  std::atomic<uint64_t> session_id{0};
  std::atomic<uint64_t> plan_version{0};
};

}  // namespace components
}  // namespace ayanami