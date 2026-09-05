// BehaviorTreePilot 实现 (spec §6.1 常驻树 + pending_goal 黑板覆盖 + 抢占归 tick
// 线程)。
//
// 抢占纪律: beginNavigation 只做 latest-wins 邮箱投递 (覆写 pending_goal +
// 唤醒), **绝不跨线程 halt BT 树**。实际抢占全部落在 run 线程:
//   applyPendingGoal: 本地 haltTree (tick 线程) → controllerAction()->cancelAll()
//   → 发一条零速度 (Global Constraint P9) → startNewSession (session+1 /
//   plan_version 归零, 与 NavigationServerBase 共享同一对象) → 重写黑板。
// halt() 只是请求位 (halt_requested), run 线程 consumeHaltRequest 本地消费
// (供停止/析构前停树; join 之后不再有并发 tick)。树常驻不重建 (spec §6.1)。

#include "ayanami/behavior_tree/pilot.hpp"

#include "ayanami/behavior_tree/action_nodes.hpp"
#include "ayanami/behavior_tree/navigation.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/utility/config.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace ayanami {
namespace behavior_tree {

// 泛型树缺省扩展 .so (requirement 3): navigation.behavior_tree_extension 缺省
// 时回退到本名字 (生产配置始终显式给出)。
namespace {
constexpr const char* k_default_tree_extension_so =
    "libayanami_navigation_behavior_tree.so";
}  // namespace

BehaviorTreePilot::BehaviorTreePilot()
    : session_state(
          std::make_shared<components::NavigationSessionState>()) {}

BehaviorTreePilot::~BehaviorTreePilot() { stop(); }

void BehaviorTreePilot::configure(const std::string& config_path) {
  GlobalConfig::instance(config_path, /*override_path=*/true);
  Config cfg(GlobalConfig::getConfigPath("config_navigation"));

  const std::string k_sec = "navigation";
  planner_frequency = cfg.param<double>(k_sec, "planner_frequency", 10.0);
  smoother_frequency = cfg.param<double>(k_sec, "smoother_frequency", -1.0);
  controller_frequency =
      cfg.param<double>(k_sec, "controller_frequency", 20.0);
  tick_frequency = cfg.param<double>(k_sec, "tick_frequency", 100.0);

// 泛型树扩展 (requirement 3): 配置指定扩展 .so 名(列表), configure 期缓存,
  // buildTree 期经 ExtensionModule::load_module 装载。navigation 段的值可为
  // 单字符串或字符串数组。
  tree_extension_so_names.clear();
  try {
    tree_extension_so_names = cfg.param<std::vector<std::string>>(
        k_sec, "behavior_tree_extension", {});
  } catch (const std::exception&) {
    // behavior_tree_extension 是单字符串。
  }
  if (tree_extension_so_names.empty()) {
    const std::string single =
        cfg.param<std::string>(k_sec, "behavior_tree_extension", "");
    if (!single.empty()) {
      tree_extension_so_names.push_back(single);
    }
  }
  if (tree_extension_so_names.empty()) {
    tree_extension_so_names.push_back(k_default_tree_extension_so);
  }

  // 模块 id 单一来源 = 各模块 json 的 name 字段 (无独立 *_id 配置键)。
  // recovery 角色映射 (clear_costmap/spin/wait/backup) 由 config_recovery
  // 有序文件族内每个文件的 recovery.role 字段决定, 供默认树按角色引用。

  clear_costmap_id.clear();
  spin_id.clear();
  wait_id.clear();
  backup_id.clear();
  std::vector<std::string> recovery_files;
  try {
    recovery_files = cfg.param<std::vector<std::string>>(
        k_sec, "config_recovery", {});
  } catch (const std::exception&) {
  }
  if (recovery_files.empty()) {
    recovery_files.push_back(cfg.param<std::string>(k_sec, "config_recovery", ""));
  }
  const std::string directory =
      GlobalConfig::instance()->param<std::string>("global", "config_path", ".");
  for (const auto& file : recovery_files) {
    if (file.empty()) {
      continue;
    }
    Config recovery_cfg(std::filesystem::path(file).is_absolute()
                            ? file
                            : (directory + "/" + file));
    const std::string name =
        recovery_cfg.param<std::string>("recovery", "name", "");
    const std::string role =
        recovery_cfg.param<std::string>("recovery", "role", "");
    if (role == "clear_costmap") clear_costmap_id = name;
    if (role == "spin") spin_id = name;
    if (role == "wait") wait_id = name;
    if (role == "backup") backup_id = name;
  }
}

void BehaviorTreePilot::setNavigationRuntime(
    components::NavigationRuntime::Ptr runtime) {
  navigation_runtime = std::move(runtime);
  if (navigation_runtime) {
    // 共享会话/版本单一来源: pilot 与 server 用同一 NavigationSessionState,
    // 黑板 session 与实际 goal session 不再发散。
    navigation_runtime->setSessionState(session_state);
  }
  if (bt_blackboard) {
    bt_blackboard->set<components::NavigationRuntime::Ptr>(
        k_navigation_runtime, navigation_runtime);
  }
  if (load_requested && !tree_built) {
    buildTree(xml_path, tree_id);
  }
}

void BehaviorTreePilot::loadFromFile(const std::string& xml_path,
                                     const std::string& tree_id) {
  if (tree_built || load_requested) {
    throw std::runtime_error(
        "BehaviorTreePilot: loadFromFile must be called only once");
  }
  this->xml_path = xml_path;
  this->tree_id = tree_id;

  bt_blackboard = BT::Blackboard::create();
  writeRequiredBlackboardEntries();
  load_requested = true;

  if (navigation_runtime) {
    buildTree(xml_path, tree_id);
  }
}

void BehaviorTreePilot::beginNavigation(
    const geometry_msgs::PoseStamped& goal) {
  if (!tree_built) {
    throw std::runtime_error(
        "BehaviorTreePilot: beginNavigation before loadFromFile/"
        "setNavigationRuntime");
  }
  {
    std::lock_guard<std::mutex> lk(goal_mutex);
    // latest wins: 覆写 pending_goal (新 goal 覆盖旧 goal, 不排队)。
    pending_goal = goal;
  }
  bt_tree.emitWakeUpSignal();  // 唤醒 run 循环 (可能在 bt_tree.sleep)
}

void BehaviorTreePilot::run() {
  if (!tree_built) {
    spdlog::error("BehaviorTreePilot::run: bt_tree not loaded");
    return;
  }
  running.store(true);
  const double hz = tick_frequency > 0.0 ? tick_frequency : 100.0;
  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / hz));
  const auto safe_period = std::max(period, std::chrono::milliseconds(1));
  while (true) {
    // 抢占归 tick 线程: halt 请求 / pending goal 都在本地消费 (绝不跨线程
    // haltTree)。
    consumeHaltRequest();
    if (kill_switch.load()) break;
    applyPendingGoal();
    const BT::NodeStatus tick_status = bt_tree.tickOnce();
    // 树生命终端复位 (v4 halt 语义适配): BT.CPP v4 的 halt 只传播到 RUNNING
    // 子节点, 终态缓存的子树 (RateController 单次缓存 / RecoveryNode 重试计数
    // / RecoverySequence 游标) 不会被重置 —— 不复位则 planner 永不重规划、恢
    // 复风暴跨树生命持续 (rviz 恢复风暴 bug 根因, 见
    // production_tree_recovery_test)。SUCCESS/FAILURE 后等价 nav2 v3
    // haltAllActions: 下一 tick 全新树生命 (planner 重规划)。
    if (tick_status != BT::NodeStatus::RUNNING) {
      resetNavigationTreeState(bt_tree);
    }
    // 唤醒竞态修复: tickOnce 内层可能把 beginNavigation 的唤醒信号用旧黑板
    // 消费掉, 使随后 sleep 睡满一个周期。若 tick 期间又有新 goal 投递, 立即
    // 消费并补一次 tick, 保证新 goal 至迟一个 tick 周期内生效。
    if (hasPendingGoal()) {
      applyPendingGoal();
      bt_tree.tickOnce();
    }
    bt_tree.sleep(safe_period);
  }
  running.store(false);
}

void BehaviorTreePilot::halt() {
  if (!tree_built) {
    return;
  }
  // 请求位: 实际 haltTree 在 run 线程 consumeHaltRequest 内执行 (不跨线程)。
  halt_requested.store(true);
  bt_tree.emitWakeUpSignal();  // 若 run 在 sleep, 唤醒让其尽快进入下一 tick
}

void BehaviorTreePilot::consumeHaltRequest() {
  if (!halt_requested.exchange(false)) {
    return;
  }
  // 本线程 (tick 线程) 持有 bt_tree 访问权, halt 安全。
  bt_tree.haltTree();
  if (navigation_runtime) {
    // 抢占纪律 (review 修复): 取消全部 controller action map (多 controller
    // 配置下不止默认 id), 防止非默认控制器的残留速度输出。
    navigation_runtime->cancelAllControllers();
  }
}

void BehaviorTreePilot::stop() {
  halt_requested.store(true);
  kill_switch.store(true);
  if (tree_built) {
    bt_tree.emitWakeUpSignal();  // 唤醒 sleep, 使 run 尽快退出
  }
  if (tick_thread.joinable()) {
    tick_thread.join();
  }
}

bool BehaviorTreePilot::hasPendingGoal() {
  std::lock_guard<std::mutex> lk(goal_mutex);
  return pending_goal.has_value();
}

void BehaviorTreePilot::applyPendingGoal() {
  std::optional<geometry_msgs::PoseStamped> goal;
  {
    std::lock_guard<std::mutex> lk(goal_mutex);
    if (!pending_goal) {
      return;
    }
    goal = std::move(pending_goal);
    pending_goal.reset();
  }
  // The first goal starts a session. Later goals revise the active task: the
  // controller keeps following its current path until the revised path is
  // available, while the planner updates in place.
  if (!navigation_started) {
    session_state->startNewSession();
    navigation_started = true;
    goal_revision = 0;
  } else {
    ++goal_revision;
  }

  bt_blackboard->set<geometry_msgs::PoseStamped>(k_goal, *goal);
  // 每轮导航: plan_version 清零 (新 goal 从 version 0 起算)
  bt_blackboard->set<uint64_t>(k_plan_version, session_state->currentPlanVersion());
  // 黑板 session 反映共享会话
  bt_blackboard->set<uint64_t>(k_session_id, session_state->currentSessionId());
  // reset_stuck_token 令牌随会话变化 -> IsRobotStuck 静态状态每轮复位 (spec §6.5)
  bt_blackboard->set<uint64_t>(k_reset_stuck_token, session_state->currentSessionId());
  bt_blackboard->set<uint64_t>(k_goal_revision, goal_revision);
  // failure_context 每轮导航清零 (失败上下文只对本会话有效)。
  bt_blackboard->set<FailureContext>(k_failure_context, FailureContext{});
}

void BehaviorTreePilot::buildTree(const std::string& xml_path,
                                  const std::string& tree_id) {
  if (!bt_blackboard) {
    bt_blackboard = BT::Blackboard::create();
    writeRequiredBlackboardEntries();
  }
  bt_blackboard->set<components::NavigationRuntime::Ptr>(
      k_navigation_runtime, navigation_runtime);

  BT::BehaviorTreeFactory factory;
  std::string fallback_tree_id;
  // 泛型树 (requirement 3): 装载并持有配置的 BehaviorTreeExtension(s), 由扩展
  // 负责注册全部自定义节点与默认树 XML; 核心不再内嵌任何导航 XML/策略。
  for (const auto& so_name : tree_extension_so_names) {
    auto ext_module = ExtensionModule::load_module(so_name);
    if (!ext_module) {
      throw std::runtime_error("BehaviorTreePilot: failed to load BT extension '" +
                               so_name + "'");
    }
    auto ext =
        std::dynamic_pointer_cast<BehaviorTreeExtension>(std::move(ext_module));
    if (!ext) {
      throw std::runtime_error("BehaviorTreePilot: '" + so_name +
                               "' is not a BehaviorTreeExtension");
    }
    ext->registerNodes(factory);
    ext->registerTrees(factory);
    if (fallback_tree_id.empty()) {
      fallback_tree_id = ext->defaultTreeId();
    }
    tree_extensions.push_back(std::move(ext));
  }
  if (xml_path.empty()) {
    // 默认树由扩展注册 (spec §6.3)。
  } else {
    factory.registerBehaviorTreeFromFile(xml_path);
  }
  const std::string resolved_tree_id =
      tree_id.empty() ? fallback_tree_id : tree_id;
  if (resolved_tree_id.empty()) {
    throw std::runtime_error(
        "BehaviorTreePilot: no bt_tree id configured and extension provides no "
        "default");
  }
  bt_tree = factory.createTree(resolved_tree_id, bt_blackboard);
  validateRequiredBlackboardEntries(bt_blackboard);
  tree_built = true;
}

void BehaviorTreePilot::writeRequiredBlackboardEntries() {
  bt_blackboard->set<geometry_msgs::PoseStamped>(k_goal,
                                               geometry_msgs::PoseStamped{});
  bt_blackboard->set<nav_msgs::Path>(k_raw_path, nav_msgs::Path{});
  bt_blackboard->set<uint64_t>(k_path_id, 0u);
  bt_blackboard->set<nav_msgs::Path>(k_smoothed_path, nav_msgs::Path{});
  bt_blackboard->set<double>(k_planner_frequency, planner_frequency);
  bt_blackboard->set<double>(k_smoother_frequency, smoother_frequency);
  bt_blackboard->set<double>(k_controller_frequency, controller_frequency);
  bt_blackboard->set<std::string>(k_clear_costmap_id, clear_costmap_id);
  bt_blackboard->set<std::string>(k_spin_id, spin_id);
  bt_blackboard->set<std::string>(k_wait_id, wait_id);
  bt_blackboard->set<std::string>(k_backup_id, backup_id);
  bt_blackboard->set<uint64_t>(k_session_id, session_state->currentSessionId());
  bt_blackboard->set<uint64_t>(k_plan_version, 0u);
  bt_blackboard->set<uint64_t>(k_reset_stuck_token,
                              session_state->currentSessionId());
  bt_blackboard->set<uint64_t>(k_goal_revision, 0u);
  bt_blackboard->set<FailureContext>(k_failure_context, FailureContext{});
}

} // namespace behavior_tree
} // namespace ayanami
