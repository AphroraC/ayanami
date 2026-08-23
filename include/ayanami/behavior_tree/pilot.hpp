#pragma once

#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/abstract/components/navigation_server_base.hpp"
#include "ayanami/messages/geometry_msgs.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief 常驻树 BT 执行器 (spec §6.1, 无地图依赖)。
 *
 * 关键纪律:
 * - **树常驻不重建**: `loadFromFile` 只建一次树; 新 goal 经 `beginNavigation`
 *   写 `pending_goal_`, run 循环 `applyPendingGoal` 只重写黑板 (goal / session /
 *   plan_version / reset_stuck_token), 规避 recreateTree 重量级 + IsRobotStuck
 *   静态状态跨任务残留坑 (spec §6.5)。
 * - 新 goal 抢占 (spec §6.1): `beginNavigation` 内 `tree_.haltTree()` 停旧
 *   RUNNING action (onHalted -> task.cancel 透传) + `session_id_++` 使旧
 *   TaskHandle 结果失效 + 写 pending_goal_ + `emitWakeUpSignal` 唤醒 run。
 *   零速度发布 / controllerAction()->cancelAll() / join 旧导航线程属宿主页
 *   (Task 25 CostmapNavigation) 职责, 本类不涉足。
 * - `run()` 为阻塞 tick 循环 (tick_frequency_ Hz, 默认 100Hz), 由宿主在独立
 *   线程调用; `tree_.sleep` 可被 `emitWakeUpSignal` 唤醒。
 */
class BehaviorTreePilot {
 public:
  BehaviorTreePilot() = default;
  ~BehaviorTreePilot();

  BehaviorTreePilot(const BehaviorTreePilot&) = delete;
  BehaviorTreePilot& operator=(const BehaviorTreePilot&) = delete;

  /// @brief 读 config 的 `navigation` 段 (behavior_tree_xml/id/频率/ids, spec
  /// §9.3), 缓存供 loadFromFile 写黑板。setNavigationServer 由 caller 显式调。
  void configure(const std::string& config_path);

  /// @brief 注入 NavigationServerBase (spec §6.1 "构造期注入")。可在
  /// loadFromFile 前后调用: 若 loadFromFile 已请求建树而本方法后到, 此刻建树。
  void setNavigationServer(components::NavigationServerBase* s);

  /// @brief 载入 BT XML 并建常驻树 (只调一次)。xml_path 为空时用内嵌默认
  /// XML (spec §6.3); tree_id 为空时用默认树名。树在 nav_server_ 就绪后才建
  /// (action 节点构造期从黑板读 server), 故可在 setNavigationServer 前调用。
  void loadFromFile(const std::string& xml_path, const std::string& tree_id);

  /// @brief 新 goal: halt 旧树取消 RUNNING action + session_id_++ + 写
  /// pending_goal_ + emitWakeUpSignal 唤醒 run (spec §6.1 抢占第一步)。
  void beginNavigation(const geometry_msgs::PoseStamped& goal);

  /// @brief tick 循环 (tick_frequency_ Hz, 默认 100Hz): 每 tick 前
  /// applyPendingGoal (重写黑板 goal + plan_version=0 + session + 令牌复位),
  /// 然后 tree_.tickOnce() + tree_.sleep(周期)。阻塞, 由宿主起线程调用。
  void run();

  /// @brief 停所有 RUNNING action (tree_.haltTree; 若 run 在 sleep 则唤醒)。
  void halt();

  /// @brief 终止: kill_switch_ + 唤醒 + join 内部 tick_thread_ (若有)。
  void stop();

  // ---- 测试/宿主支撑访问器 ----
  const BT::Tree& tree() const { return tree_; }
  const BT::Blackboard::Ptr& blackboard() const { return blackboard_; }

 private:
  /// @brief 把 pending_goal_ 写入黑板: goal + kPlanVersion=0 + kSessionId +
  /// kResetStuckToken (令牌变化 -> IsRobotStuck 静态状态复位, spec §6.5)。
  void applyPendingGoal();

  /// @brief 建树: 注册节点 -> 载入 XML (文件/内嵌) -> createTree -> 校验 16 板项。
  void buildTree(const std::string& xml_path, const std::string& tree_id);

  /// @brief 写 16 个必需黑板项 (spec §6.5, 值来自 configure 缓存)。
  void writeRequiredBlackboardEntries();

  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  components::NavigationServerBase* nav_server_ = nullptr;
  std::atomic<bool> running_{false};
  std::atomic<bool> kill_switch_{false};
  std::atomic<uint64_t> session_id_{0};
  std::thread tick_thread_;
  double tick_frequency_ = 100.0;
  std::mutex goal_mutex_;
  std::optional<geometry_msgs::PoseStamped> pending_goal_;

  // configure() 缓存 (loadFromFile 写黑板所需, spec §9.3)
  std::string planner_id_, smoother_id_, controller_id_, recovery_id_;
  std::string clear_costmap_id_, spin_id_, wait_id_, backup_id_;
  double planner_frequency_ = 10.0;
  double controller_frequency_ = 20.0;

  std::string xml_path_;
  std::string tree_id_;
  bool load_requested_ = false;
  bool tree_built_ = false;
};

} // namespace behavior_tree
} // namespace ayanami
