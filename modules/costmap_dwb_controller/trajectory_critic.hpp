// trajectory_critic.hpp — 迁移自 dwb_core/trajectory_critic.hpp (node 参数读取
// 换 ayanami::Config; costmap_ros 换 costmap_2d::Costmap2D* + footprint 注入)。
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <costmap_2d/costmap_2d.hpp>

#include "ayanami/utility/config.hpp"

#include "dwb_types.hpp"

namespace dwb_core {

/**
 * @class TrajectoryCritic
 * @brief Evaluates a Trajectory2D to produce a score
 *
 * This class defines the plugin interface for the TrajectoryCritic which
 * gives scores to trajectories, where lower numbers are better, but negative
 * scores are considered invalid.
 *
 * The general lifecycle is
 *  1) initialize is called once at the beginning which in turn calls onInit.
 *  2) prepare is called once before each set of trajectories.
 *  3) scoreTrajectory is called once per trajectory and returns the score.
 *  4) debrief is called after each set of trajectories with the chosen
 *     trajectory.
 */
class TrajectoryCritic {
public:
  using Ptr = std::shared_ptr<dwb_core::TrajectoryCritic>;

  virtual ~TrajectoryCritic() {}

  /**
   * @brief Initialize the critic with appropriate pointers and parameters
   *
   * The name and costmap are stored as member variables.
   * scale 参数读 <plugin_name>.<name>.scale (默认 1.0, 与 nav2 语义一致)。
   *
   * @param cfg 模块配置 (json 段 dwb_plugin_name)
   * @param name The name of this critic
   * @param ns The namespace of the planner (dwb_plugin_name)
   * @param costmap Pointer to the local costmap
   * @param footprint Robot footprint spec (ObstacleFootprint 等 critic 使用)
   */
  void initialize(const ayanami::Config& cfg, const std::string& name,
                  const std::string& ns, const costmap_2d::Costmap2D* costmap,
                  const std::vector<geometry_msgs::Point>* footprint) {
    this->name = name;
    this->costmap = costmap;
    this->footprint = footprint;
    dwb_plugin_name = ns;
    this->cfg = &cfg;
    scale = cfg.param<double>(dwb_plugin_name, name + ".scale", 1.0);
    onInit();
  }
  virtual void onInit() {}

  /// @brief 快照纪律: 每 cycle 由控制器在刷新 local 快照后调用, 把 critic
  /// 绑定到当前不可变 epoch 的 grid (快照由控制器持有存活整个打分周期)。
  /// 默认只换指针; 缓存了 &grid 引用/队列的 critic 覆写为重建缓存。
  virtual void setCostmap(const costmap_2d::Costmap2D* costmap) {
    this->costmap = costmap;
  }

  /// @brief 子类 onInit 读取参数用 (Config 不可拷贝, 保存指针)。
  const ayanami::Config& config() const { return *cfg; }

  /**
   * @brief Reset the state of the critic
   */
  virtual void reset() {}

  /**
   * @brief Prior to evaluating any trajectories, look at contextual information
   * constant across all trajectories
   */
  virtual bool prepare(const geometry_msgs::Pose2D&,
                       const nav_2d_msgs::Twist2D&,
                       const geometry_msgs::Pose2D&,
                       const nav_2d_msgs::Path2D&) {
    return true;
  }

  /**
   * @brief Return a raw score for the given trajectory.
   *
   * scores < 0 are considered invalid/errors, such as collisions
   */
  virtual double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) = 0;

  /**
   * @brief debrief informs the critic what the chosen cmd_vel was (if it cares)
   */
  virtual void debrief(const nav_2d_msgs::Twist2D&) {}

  /**
   * @brief Add information to the given pointcloud for debugging costmap-grid
   * based scores
   */
  virtual void addCriticVisualization(
      std::vector<std::pair<std::string, std::vector<float>>>&) {}

  std::string getName() { return name; }

  virtual double getScale() const { return scale; }
  void setScale(const double scale) { this->scale = scale; }

protected:
  std::string name;
  std::string dwb_plugin_name;
  const costmap_2d::Costmap2D* costmap;
  const std::vector<geometry_msgs::Point>* footprint{nullptr};
  const ayanami::Config* cfg{nullptr};
  double scale;
};

}  // namespace dwb_core
