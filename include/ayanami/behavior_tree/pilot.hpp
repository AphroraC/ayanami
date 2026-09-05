#pragma once

#include "ayanami/behavior_tree/blackboard_protocol.hpp"
#include "ayanami/behavior_tree/extension.hpp"
#include "ayanami/abstract/components/navigation_server_base.hpp"
#include "ayanami/messages/geometry_msgs.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace ayanami {
namespace behavior_tree {

/**
 * @brief 常驻树 BT 执行器 (spec §6.1, 无地图依赖)。
 *
 * 关键纪律:
 * - **树常驻不重建**: `loadFromFile` 只建一次树; 新 goal 经 `beginNavigation`
 *   写 `pending_goal`, run 循环 `applyPendingGoal` 只重写黑板 (goal / session /
 *   plan_version / reset_stuck_token), 规避 recreateTree 重量级 + IsRobotStuck
 *   静态状态跨任务残留坑 (spec §6.5)。
 * - 新 goal 通过 latest-wins 邮箱交给 tick 线程。首个 goal 开启 session；后续
 *   goal 只推进 goal_revision，planner 原位更新，controller 在新路径可用前继续
 *   跟踪旧路径。不会因普通目标修订 halt 整树或发布零速度。
 * - `halt()` 仅请求位 (atomic halt_requested), run 线程消费后在本地 halt,
 *   供 stop/shutdown 路径安全停树 (析构前 join 已保证无并发 tick)。
 * - `run()` 为阻塞 tick 循环 (tick_frequency Hz, 默认 100Hz), 由宿主在独立
 *   线程调用; `bt_tree.sleep` 可被 `emitWakeUpSignal` 唤醒。
 *
 * 泛型树 (requirement 3): **核心不含默认导航 XML/策略**。configure 缓存
 * navigation.behavior_tree_extension 指定的扩展 .so 名(列表); buildTree 经
 * ExtensionModule::load_module 装载并持有 BehaviorTreeExtension(s), 依次
 * registerNodes + registerTrees, 再按配置 tree_id (缺省用扩展 defaultTreeId)
 * createTree。黑板块协议 (blackboard_protocol.hpp) 与运行期核心保持在核心内。
 */
class BehaviorTreePilot {
 public:
  BehaviorTreePilot();
  ~BehaviorTreePilot();

  BehaviorTreePilot(const BehaviorTreePilot&) = delete;
  BehaviorTreePilot& operator=(const BehaviorTreePilot&) = delete;

  /// @brief 读 config 的 `navigation` 段 (behavior_tree_xml/id/频率/ids, spec
  /// §9.3), 缓存供 loadFromFile 写黑板。setNavigationRuntime 由 caller 显式调。
  /// xml 缺省时生产路径为空 → loadFromFile("") 用 BehaviorTreeExtension 内嵌
  /// 默认树 (production 单一来源); 测试可显式给 behavior_tree_xml 自定义 XML。
  void configure(const std::string& config_path);

  /// @brief 注入 NavigationRuntime (spec §6.1 "构造期注入")。可在
  /// loadFromFile 前后调用: 若 loadFromFile 已请求建树而本方法后到, 此刻建树。
  /// 黑板只写 NavigationRuntime::Ptr (shared_ptr), 不写裸指针兼容层。
  void setNavigationRuntime(components::NavigationRuntime::Ptr runtime);

  /// @brief 载入 BT XML 并建常驻树 (只调一次)。xml_path 为空时用扩展注册的
  /// 默认树 (spec §6.3); tree_id 为空时用扩展 defaultTreeId。树在 nav_server
  /// 就绪后才建 (action 节点构造期从黑板读 runtime), 故可在 setNavigationRuntime
  /// 前调用。节点注册与树 XML 全部来自配置的 BehaviorTreeExtension (requirement 3)。
  void loadFromFile(const std::string& xml_path, const std::string& tree_id);

  /// @brief 新 goal 邮箱投递 (latest-wins): 覆写 pending_goal +
  /// emitWakeUpSignal 唤醒 run。**不再跨线程 haltTree** —— 实际抢占
  /// (halt 旧树 / controller cancelAll / 零速度 / 会话推进) 全部在 tick 线程
  /// applyPendingGoal 内完成 (spec §6.1 抢占纪律)。
  void beginNavigation(const geometry_msgs::PoseStamped& goal);

  /// @brief tick 循环 (tick_frequency Hz, 默认 100Hz): 每 tick 前先消费
  /// halt_requested / pending_goal (applyPendingGoal: turn+halt 旧树 +
  /// controller cancelAll + 一条零速度 + startNewSession + 重写黑板),
  /// 然后 bt_tree.tickOnce() + bt_tree.sleep(周期)。阻塞, 由宿主起线程调用。
  void run();

  /// @brief 请求停所有 RUNNING action (bt_tree.haltTree 在 tick 线程执行; 若 run
  /// 在 sleep 则唤醒)。主要供 stop/shutdown 前请求。
  void halt();

  /// @brief 终止: kill_switch + 唤醒 + join 内部 tick_thread (若有)。
  void stop();

  // ---- 测试/宿主支撑访问器 ----
  const BT::Tree& tree() const { return bt_tree; }
  const BT::Blackboard::Ptr& blackboard() const { return bt_blackboard; }

  /// @brief 共享会话/版本状态 (与 NavigationServerBase 同一对象)。
  std::shared_ptr<components::NavigationSessionState> sessionState() const {
    return session_state;
  }

 private:
  /// @brief run 线程专用: 若 halt_requested 置位则本地 halt 旧树 + controller
  /// cancelAll (停 RUNNING action, 不跨线程)。
  void consumeHaltRequest();

  /// @brief run 线程专用: 首个 goal 开启 session；后续 goal 推进 revision，
  /// 并重写 goal / revision / failure context 黑板项。
  void applyPendingGoal();

  /// @brief run 线程专用: pending_goal 邮箱是否非空 (消除 tickOnce 内层
  /// 用旧黑板消费唤醒信号的竞态, 见 run() 注释)。
  bool hasPendingGoal();

  /// @brief 建树: 装载/持有 BehaviorTreeExtension(s) → 注册节点 → 注册树
  /// (文件或扩展默认) → createTree → 校验 17 板项。
  void buildTree(const std::string& xml_path, const std::string& tree_id);

  /// @brief 写 17 个必需黑板项 (spec §6.5, 值来自 configure 缓存)。
  void writeRequiredBlackboardEntries();

  BT::Tree bt_tree;
  BT::Blackboard::Ptr bt_blackboard;
  components::NavigationRuntime::Ptr navigation_runtime;
  // 泛型树: 装载并持有的 BehaviorTreeExtension(s) (要求 3)。注册节点/树后再
  // createTree。析构顺序: extensions 先于 bt_tree 释放, 无循环依赖。
  std::vector<ayanami::behavior_tree::BehaviorTreeExtension::Ptr>
      tree_extensions;
  std::atomic<bool> running{false};
  std::atomic<bool> kill_switch{false};
  // halt 请求位: halt() 只置位 + 唤醒, run 线程消费 (tick 线程内 haltTree)。
  std::atomic<bool> halt_requested{false};
  // 会话/版本单一来源: 与 NavigationServerBase 共享同一对象 (setNavigationRuntime
  // 注入), 取代原本各自独立的 session 计数 (spec 单一来源纪律)。
  std::shared_ptr<components::NavigationSessionState> session_state;
  std::thread tick_thread;
  double tick_frequency = 100.0;
  std::mutex goal_mutex;
  std::optional<geometry_msgs::PoseStamped> pending_goal;
  uint64_t goal_revision{0};
  bool navigation_started{false};

  // configure() 缓存 (loadFromFile 写黑板所需, spec §9.3)
  std::string clear_costmap_id, spin_id, wait_id, backup_id;
  double planner_frequency = 10.0;
  double smoother_frequency = -1.0;
  double controller_frequency = 20.0;

  // 泛型树 (requirement 3): navigation.behavior_tree_extension 指定的扩展 so。
  std::vector<std::string> tree_extension_so_names;

  std::string xml_path;
  std::string tree_id;
  bool load_requested = false;
  bool tree_built = false;
};

} // namespace behavior_tree
} // namespace ayanami
