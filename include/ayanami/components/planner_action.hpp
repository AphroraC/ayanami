#pragma once

#include "ayanami/components/action_base.hpp"
#include "ayanami/components/action_server.hpp"
#include "ayanami/components/planner_execution.hpp"
#include "ayanami/core/abstract_planner.hpp"
#include "ayanami/interfaces/callbacks.hpp"
#include "ayanami/messages/actions.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace ayanami {
namespace components {

/**
 * @brief 全局路径规划 Action (对标 MBF PlannerAction, spec §5.1)。
 *
 * acceptGoal 返回 TaskHandle 直连调用者 (主流, BT 节点); 结果同时 emit 到
 * NavigationCallbacks::on_get_path_result 作旁路 (rviz 扩展/回退/监控)。
 * ExecutionState→GoalStatus 映射 (Ruling #2): FOUND_PLAN→SUCCEEDED,
 * CANCELED→CANCELED, 其余 terminal (MAX_RETRIES/PAT_EXCEEDED/NO_PLAN_FOUND/
 * STOPPED/INTERNAL_ERROR)→ABORTED + outcome。
 */
class PlannerAction : public ActionBase<ayanami::actions::GetPathGoal,
                                        PlannerExecution> {
public:
  using Base = ActionBase<ayanami::actions::GetPathGoal, PlannerExecution>;
  using Goal = ayanami::actions::GetPathGoal;
  using Result = ayanami::actions::GetPathResult;
  using Feedback = ayanami::actions::GetPathFeedback;
  using Server = ActionServer<Goal, Result, Feedback>;
  using ServerPtr = std::shared_ptr<Server>;
  using Task = Server::Task;

  explicit PlannerAction(ayanami::abstract::AbstractPlanner::Ptr planner)
      : planner_(std::move(planner)),
        server_(std::make_shared<Server>()) {
    server_->setGoalCallback([](const Server::GoalHandleType& gh) {
      ayanami::interfaces::NavigationCallbacks::on_get_path_goal(gh.goal);
    });
    server_->setCancelCallback([this](uint64_t goal_id) {
      Base::cancelGoal(goal_id);
    });
    server_->setFeedbackCallback([](const GoalFeedback<Feedback>& gf) {
      ayanami::interfaces::NavigationCallbacks::on_get_path_feedback(gf.feedback);
    });
    server_->setResultCallback([](const Server::GoalResultType& gr) {
      ayanami::interfaces::NavigationCallbacks::on_get_path_result(gr);
    });
  }

  ~PlannerAction() override { cancelAll(); }

  /// @brief 主流: 接受 goal 并拿 TaskHandle 直连 (spec §3.4)。
  Task acceptGoal(const Goal& goal) {
    auto task = server_->acceptGoal(goal);
    auto gh = server_->getGoal(task.goal_id());
    if (!gh) {
      return task;
    }
    if (!planner_) {
      Result result;
      result.outcome = Result::INTERNAL_ERROR;
      result.message = "Planner plugin is null";
      result.session_id = goal.session_id;
      result.plan_version = goal.plan_version;
      server_->publishResult(gh->id, ayanami::actions::GoalStatus::ABORTED,
                             result);
      return task;
    }
    Base::start(*gh, createExecution(), 0);
    return task;
  }

  bool cancelGoal(uint64_t goal_id) { return server_->cancelGoal(goal_id); }

  void cancelAll() {
    server_->cancelAll();
    Base::cancelAll();
  }

  ServerPtr server() const { return server_; }

protected:
  void runImpl(const GoalHandleType& gh, PlannerExecution& exec) override {
    server_->setActive(gh.id);
    exec.start(gh.goal.start, gh.goal.goal, gh.goal.tolerance);
    runImplCommon(
        gh, exec,
        [this](const GoalHandleType& gh, PlannerExecution& exec) -> bool {
          if (server_->isCancelRequested(gh.id)) {
            exec.cancel();
          }
          const bool canceled =
              server_->isCancelRequested(gh.id) ||
              exec.getState() == PlannerExecution::CANCELED;
          if (canceled) {
            finishResult(gh, exec, ayanami::actions::GoalStatus::CANCELED);
            return true;
          }
          if (exec.getState() == PlannerExecution::FOUND_PLAN) {
            finishResult(gh, exec, ayanami::actions::GoalStatus::SUCCEEDED);
            return true;
          }
          switch (exec.getState()) {
            case PlannerExecution::MAX_RETRIES:
            case PlannerExecution::PAT_EXCEEDED:
            case PlannerExecution::NO_PLAN_FOUND:
            case PlannerExecution::STOPPED:
            case PlannerExecution::INTERNAL_ERROR:
              finishResult(gh, exec, ayanami::actions::GoalStatus::ABORTED);
              return true;
            default:
              break;
          }
          return false;
        });
  }

  virtual PlannerExecution::Ptr createExecution() {
    return std::make_shared<PlannerExecution>(planner_);
  }

protected:
  ayanami::abstract::AbstractPlanner::Ptr planner_;

private:
  void finishResult(const GoalHandleType& gh, const PlannerExecution& exec,
                    ayanami::actions::GoalStatus status) {
    Result result;
    result.outcome = exec.getOutcome();
    result.message = exec.getMessage();
    result.path.header = gh.goal.goal.header;
    result.path.poses = exec.getPlan().poses;
    result.cost = exec.getCost();
    result.session_id = gh.goal.session_id;
    result.plan_version = gh.goal.plan_version;
    server_->publishResult(gh.id, status, result);
  }

  ServerPtr server_;
};

} // namespace components
} // namespace ayanami
