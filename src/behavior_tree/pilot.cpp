// BehaviorTreePilot 实现 (spec §6.1 常驻树 + pending_goal 黑板覆盖)。
//
// 抢占纪律: 本类 beginNavigation 内只做 "halt 树 + session_id_++ + 写
// pending_goal_ + 唤醒" —— 零速度发布 / controllerAction()->cancelAll() /
// join 旧导航线程是宿主页 (Task 25 CostmapNavigation) 职责; 树 stop 之前旧
// RUNNING 节点已被 halt 取消 (onHalted -> task.cancel), 迟到的结果经
// session_id 失效。树常驻不重建 (spec §6.1)。

#include "ayanami/behavior_tree/pilot.hpp"

#include "ayanami/behavior_tree/navigation.hpp"
#include "ayanami/utility/config.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace ayanami {
namespace behavior_tree {

namespace {
constexpr const char* kDefaultTreeId =
    "NavigateWithReplanningAndSmoothing";
} // namespace

BehaviorTreePilot::~BehaviorTreePilot() { stop(); }

void BehaviorTreePilot::configure(const std::string& config_path) {
  GlobalConfig::instance(config_path, /*override_path=*/true);
  Config cfg(GlobalConfig::get_config_path("config_navigation"));

  const std::string kSec = "navigation";
  planner_frequency_ = cfg.param<double>(kSec, "planner_frequency", 10.0);
  controller_frequency_ =
      cfg.param<double>(kSec, "controller_frequency", 20.0);
  tick_frequency_ = cfg.param<double>(kSec, "tick_frequency", 100.0);

  Config planner_cfg(GlobalConfig::get_config_path("config_planner"));
  Config smoother_cfg(GlobalConfig::get_config_path("config_smoother"));
  Config controller_cfg(GlobalConfig::get_config_path("config_controller"));
  planner_id_ = planner_cfg.param<std::string>("planner", "name", "");
  smoother_id_ = smoother_cfg.param<std::string>("smoother", "name", "");
  controller_id_ = controller_cfg.param<std::string>("controller", "name", "");

  recovery_id_.clear();
  clear_costmap_id_.clear();
  spin_id_.clear();
  wait_id_.clear();
  backup_id_.clear();
  for (const auto& path : GlobalConfig::get_config_paths("config_recovery")) {
    Config recovery_cfg(path);
    const std::string name =
        recovery_cfg.param<std::string>("recovery", "name", "");
    const std::string role =
        recovery_cfg.param<std::string>("recovery", "role", "");
    if (recovery_id_.empty() || role == "clear_costmap") recovery_id_ = name;
    if (role == "clear_costmap") clear_costmap_id_ = name;
    if (role == "spin") spin_id_ = name;
    if (role == "wait") wait_id_ = name;
    if (role == "backup") backup_id_ = name;
  }
}

void BehaviorTreePilot::setNavigationServer(
    components::NavigationServerBase* s) {
  nav_server_ = s;
  if (blackboard_) {
    blackboard_->set<components::NavigationServerBase*>(kNavigationServer, s);
  }
  if (load_requested_ && !tree_built_) {
    buildTree(xml_path_, tree_id_);
  }
}

void BehaviorTreePilot::loadFromFile(const std::string& xml_path,
                                     const std::string& tree_id) {
  if (tree_built_ || load_requested_) {
    throw std::runtime_error(
        "BehaviorTreePilot: loadFromFile must be called only once");
  }
  xml_path_ = xml_path;
  tree_id_ = tree_id.empty() ? kDefaultTreeId : tree_id;

  blackboard_ = BT::Blackboard::create();
  writeRequiredBlackboardEntries();
  load_requested_ = true;

  if (nav_server_) {
    buildTree(xml_path_, tree_id_);
  }
}

void BehaviorTreePilot::beginNavigation(
    const geometry_msgs::PoseStamped& goal) {
  if (!tree_built_) {
    throw std::runtime_error(
        "BehaviorTreePilot: beginNavigation before loadFromFile/"
        "setNavigationServer");
  }
  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    tree_.haltTree();          // 停旧 RUNNING action (onHalted -> task.cancel)
    session_id_.fetch_add(1);  // 使旧 TaskHandle 结果失效
    pending_goal_ = goal;
  }
  tree_.emitWakeUpSignal();  // 唤醒 run 循环 (可能在 tree_.sleep)
}

void BehaviorTreePilot::run() {
  if (!tree_built_) {
    spdlog::error("BehaviorTreePilot::run: tree not loaded");
    return;
  }
  running_.store(true);
  const double hz = tick_frequency_ > 0.0 ? tick_frequency_ : 100.0;
  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / hz));
  const auto safe_period = std::max(period, std::chrono::milliseconds(1));
  while (!kill_switch_.load()) {
    applyPendingGoal();
    tree_.tickOnce();
    tree_.sleep(safe_period);
  }
  running_.store(false);
}

void BehaviorTreePilot::halt() {
  if (!tree_built_) {
    return;
  }
  tree_.haltTree();
  tree_.emitWakeUpSignal();  // 若 run 在 sleep, 唤醒让其尽快进入下一 tick
}

void BehaviorTreePilot::stop() {
  kill_switch_.store(true);
  if (tree_built_) {
    tree_.emitWakeUpSignal();  // 唤醒 sleep, 使 run 尽快退出
  }
  if (tick_thread_.joinable()) {
    tick_thread_.join();
  }
}

void BehaviorTreePilot::applyPendingGoal() {
  std::optional<geometry_msgs::PoseStamped> goal;
  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    if (!pending_goal_) {
      return;
    }
    goal = std::move(pending_goal_);
    pending_goal_.reset();
  }
  blackboard_->set<geometry_msgs::PoseStamped>(kGoal, *goal);
  // 每轮导航: plan_version 清零 (新 goal 从 version 0 起算)
  blackboard_->set<uint64_t>(kPlanVersion, 0u);
  // 黑板 session 反映 pilot 当前会话
  blackboard_->set<uint64_t>(kSessionId, session_id_.load());
  // reset_stuck_token 令牌随会话变化 -> IsRobotStuck 静态状态每轮复位 (spec §6.5)
  blackboard_->set<uint64_t>(kResetStuckToken, session_id_.load());
}

void BehaviorTreePilot::buildTree(const std::string& xml_path,
                                  const std::string& tree_id) {
  if (!blackboard_) {
    blackboard_ = BT::Blackboard::create();
    writeRequiredBlackboardEntries();
  }
  blackboard_->set<components::NavigationServerBase*>(kNavigationServer,
                                                      nav_server_);

  BT::BehaviorTreeFactory factory;
  registerNavigationBehaviorTreeNodes(factory);
  if (xml_path.empty()) {
    factory.registerBehaviorTreeFromText(kDefaultNavigationXml);
  } else {
    factory.registerBehaviorTreeFromFile(xml_path);
  }
  tree_ = factory.createTree(tree_id, blackboard_);
  validateRequiredBlackboardEntries(blackboard_);
  tree_built_ = true;
}

void BehaviorTreePilot::writeRequiredBlackboardEntries() {
  blackboard_->set<geometry_msgs::PoseStamped>(kGoal,
                                               geometry_msgs::PoseStamped{});
  blackboard_->set<nav_msgs::Path>(kRawPath, nav_msgs::Path{});
  blackboard_->set<nav_msgs::Path>(kSmoothedPath, nav_msgs::Path{});
  blackboard_->set<double>(kPlannerFrequency, planner_frequency_);
  blackboard_->set<double>(kControllerFrequency, controller_frequency_);
  blackboard_->set<std::string>(kPlannerId, planner_id_);
  blackboard_->set<std::string>(kSmootherId, smoother_id_);
  blackboard_->set<std::string>(kControllerId, controller_id_);
  blackboard_->set<std::string>(kRecoveryId, recovery_id_);
  blackboard_->set<std::string>(kClearCostmapId, clear_costmap_id_);
  blackboard_->set<std::string>(kSpinId, spin_id_);
  blackboard_->set<std::string>(kWaitId, wait_id_);
  blackboard_->set<std::string>(kBackupId, backup_id_);
  blackboard_->set<uint64_t>(kSessionId, session_id_.load());
  blackboard_->set<uint64_t>(kPlanVersion, 0u);
  blackboard_->set<uint64_t>(kResetStuckToken, session_id_.load());
}

} // namespace behavior_tree
} // namespace ayanami
