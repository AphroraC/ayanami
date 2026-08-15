// Task 21: modules/costmap_smac_planner — A* 全局路径规划模块。
//
// 纯 C++ (零 ROS): 在注入的 CostmapContext global LayeredCostmap 上做
// 标准 4/8 连通 A*。参考 _legacy SmacPlannerAStar 语义但重写:
// - 阻断: cell cost >= INSCRIBED_INFLATED_OBSTACLE (Ruling #6, 253 起 lethal)
// - 行走代价: step * (neutral_cost + cost_factor * cell_cost)
// - goal 在障碍内: nearestValidGoal 在 tolerance 内搜索最近可行格 (近似路径)
// - cancel(): 搜索循环逐节点检查, 打断后返回规划段 CANCELED=68

#include <ayanami/modules/costmap_smac_planner/costmap_smac_planner.hpp>

#include <ayanami/messages/actions.hpp>
#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace ayanami {
namespace modules {

namespace {

constexpr double kDiagonalStep = std::sqrt(2.0);
// 规划段错误 (spec §5.1 50+): CANCELED。
constexpr uint32_t kCanceledOutcome = 68;

struct Cell {
  unsigned int x{0};
  unsigned int y{0};
};

// A* 搜索参数 (从私有 Config 提取, 匿名命名空间无需访问私有类型)。
struct SearchParams {
  bool use_diagonal{true};
  double cost_factor{0.5};
  double neutral_cost{1.0};
};

bool isBlocked(costmap_2d::Costmap2D* cm, unsigned int x, unsigned int y) {
  return cm->getCost(x, y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

// 标准 A*。返回 {found, cost, cells(start→goal)}。canceled 逐节点检查。
struct AStarResult {
  bool found{false};
  double cost{0.0};
  std::vector<Cell> cells;
};

AStarResult runAStar(costmap_2d::Costmap2D* cm, const SearchParams& params,
                     unsigned int sx, unsigned int sy, unsigned int gx,
                     unsigned int gy, const std::atomic<bool>& canceled) {
  const unsigned int width = cm->getSizeInCellsX();
  const unsigned int height = cm->getSizeInCellsY();
  const std::size_t size =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

  const auto index = [width](unsigned int x, unsigned int y) {
    return static_cast<std::size_t>(y) * width + x;
  };

  std::vector<double> g_score(size, std::numeric_limits<double>::infinity());
  std::vector<int> parent(size, -1);
  std::vector<unsigned char> closed(size, 0);

  struct Node {
    double f{0.0};
    unsigned int x{0};
    unsigned int y{0};
  };
  const auto cmp = [](const Node& a, const Node& b) { return a.f > b.f; };
  std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);

  const auto heuristic = [&](unsigned int x, unsigned int y) {
    const double dx = static_cast<double>(gx) - static_cast<double>(x);
    const double dy = static_cast<double>(gy) - static_cast<double>(y);
    return params.neutral_cost * std::hypot(dx, dy);
  };

  g_score[index(sx, sy)] = 0.0;
  open.push(Node{heuristic(sx, sy), sx, sy});

  const std::vector<std::pair<int, int>> offsets =
      params.use_diagonal
          ? std::vector<std::pair<int, int>>{
                {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                {0, 1},   {1, -1}, {1, 0},  {1, 1}}
          : std::vector<std::pair<int, int>>{{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

  while (!open.empty()) {
    if (canceled.load()) {
      return AStarResult{};
    }
    const Node cur = open.top();
    open.pop();
    const std::size_t cur_idx = index(cur.x, cur.y);
    if (closed[cur_idx]) {
      continue;
    }
    closed[cur_idx] = 1;

    if (cur.x == gx && cur.y == gy) {
      AStarResult result;
      result.found = true;
      result.cost = g_score[cur_idx];
      int trace = static_cast<int>(cur_idx);
      while (trace >= 0) {
        const unsigned int x = static_cast<unsigned int>(trace % width);
        const unsigned int y = static_cast<unsigned int>(trace / width);
        result.cells.push_back(Cell{x, y});
        trace = parent[static_cast<std::size_t>(trace)];
      }
      std::reverse(result.cells.begin(), result.cells.end());
      return result;
    }

    for (const auto& off : offsets) {
      const int nx = static_cast<int>(cur.x) + off.first;
      const int ny = static_cast<int>(cur.y) + off.second;
      if (nx < 0 || ny < 0 ||
          nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) {
        continue;
      }
      const unsigned int ux = static_cast<unsigned int>(nx);
      const unsigned int uy = static_cast<unsigned int>(ny);
      if (isBlocked(cm, ux, uy)) {
        continue;
      }
      // 8 连通禁止穿角: 两个正交邻域也须可行。
      if (off.first != 0 && off.second != 0) {
        if (isBlocked(cm, cur.x + static_cast<unsigned int>(off.first), cur.y) ||
            isBlocked(cm, cur.x, cur.y + static_cast<unsigned int>(off.second))) {
          continue;
        }
      }
      const std::size_t nidx = index(ux, uy);
      if (closed[nidx]) {
        continue;
      }
      const bool diagonal = (off.first != 0 && off.second != 0);
      const double step = diagonal ? kDiagonalStep : 1.0;
      const double move_cost =
          step * (params.neutral_cost +
                  params.cost_factor * static_cast<double>(cm->getCost(ux, uy)));
      const double tentative = g_score[cur_idx] + move_cost;
      if (tentative < g_score[nidx]) {
        parent[nidx] = static_cast<int>(cur_idx);
        g_score[nidx] = tentative;
        open.push(Node{tentative + heuristic(ux, uy), ux, uy});
      }
    }
  }
  return AStarResult{};
}

// 在 (gx,gy) 周围 radius=tolerance_cells 内找最近可行格 (含自身)。
bool nearestValidGoal(costmap_2d::Costmap2D* cm, unsigned int gx,
                      unsigned int gy, double tolerance_cells,
                      unsigned int& ox, unsigned int& oy) {
  const int radius = static_cast<int>(std::ceil(std::max(0.0, tolerance_cells)));
  for (int r = 0; r <= radius; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) {
          continue;
        }
        const int nx = static_cast<int>(gx) + dx;
        const int ny = static_cast<int>(gy) + dy;
        if (nx < 0 || ny < 0) {
          continue;
        }
        const unsigned int ux = static_cast<unsigned int>(nx);
        const unsigned int uy = static_cast<unsigned int>(ny);
        if (ux >= cm->getSizeInCellsX() || uy >= cm->getSizeInCellsY()) {
          continue;
        }
        if (isBlocked(cm, ux, uy)) {
          continue;
        }
        ox = ux;
        oy = uy;
        return true;
      }
    }
  }
  return false;
}

}  // namespace

CostmapSmacPlanner::CostmapSmacPlanner() { loadConfig(); }

void CostmapSmacPlanner::loadConfig() {
  try {
    const ayanami::Config file(
        ayanami::GlobalConfig::get_config_path("costmap_smac_planner"));
    config_.use_diagonal = file.param<bool>(
        "costmap_smac_planner", "use_diagonal", config_.use_diagonal);
    config_.tolerance = file.param<double>(
        "costmap_smac_planner", "tolerance", config_.tolerance);
    config_.cost_factor = file.param<double>(
        "costmap_smac_planner", "cost_factor", config_.cost_factor);
    config_.lethal_cost = file.param<int>(
        "costmap_smac_planner", "lethal_cost", config_.lethal_cost);
    config_.neutral_cost = file.param<double>(
        "costmap_smac_planner", "neutral_cost", config_.neutral_cost);
    config_.use_final_approach = file.param<bool>(
        "costmap_smac_planner", "use_final_approach", config_.use_final_approach);
  } catch (...) {
  }
}

void CostmapSmacPlanner::initialize(
    const std::string& /*name*/, ayanami::costmap::CostmapContext::Ptr ctx) {
  costmap_context_ = std::move(ctx);
}

bool CostmapSmacPlanner::cancel() {
  canceled_.store(true);
  return true;
}

uint32_t CostmapSmacPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                      const geometry_msgs::PoseStamped& goal,
                                      double tolerance, nav_msgs::Path& plan,
                                      double& cost, std::string& message) {
  plan.poses.clear();
  cost = 0.0;
  message.clear();
  canceled_.store(false);

  costmap_2d::Costmap2D* cm =
      costmap_context_ ? costmap_context_->globalCostmap2D() : nullptr;
  if (!cm) {
    message = "CostmapSmacPlanner is not initialized";
    return ayanami::actions::GetPathResult::INTERNAL_ERROR;
  }

  if (!start.header.frame_id.empty() && !goal.header.frame_id.empty() &&
      start.header.frame_id != goal.header.frame_id) {
    message = "Start and goal frames do not match";
    return ayanami::actions::GetPathResult::TF_ERROR;
  }

  unsigned int sx = 0;
  unsigned int sy = 0;
  unsigned int gx = 0;
  unsigned int gy = 0;
  if (!cm->worldToMap(start.pose.translation().x(),
                      start.pose.translation().y(), sx, sy)) {
    message = "Start pose is outside the global costmap";
    return ayanami::actions::GetPathResult::OUT_OF_MAP;
  }
  if (!cm->worldToMap(goal.pose.translation().x(), goal.pose.translation().y(),
                      gx, gy)) {
    message = "Goal pose is outside the global costmap";
    return ayanami::actions::GetPathResult::OUT_OF_MAP;
  }

  if (isBlocked(cm, sx, sy)) {
    message = "Start pose is blocked";
    return ayanami::actions::GetPathResult::INVALID_START;
  }

  const double tolerance_m = tolerance > 0.0 ? tolerance : config_.tolerance;
  const double tolerance_cells =
      tolerance_m / std::max(cm->getResolution(), 1e-9);

  // goal 在障碍内: nearestValidGoal 在 tolerance 内找最近可行格。
  unsigned int target_x = gx;
  unsigned int target_y = gy;
  bool approximate = false;
  if (isBlocked(cm, gx, gy)) {
    if (!nearestValidGoal(cm, gx, gy, tolerance_cells, target_x, target_y)) {
      message = "Goal pose is blocked and no valid cell within tolerance";
      return ayanami::actions::GetPathResult::NO_PATH_FOUND;
    }
    approximate = true;
  }

  SearchParams params;
  params.use_diagonal = config_.use_diagonal;
  params.cost_factor = config_.cost_factor;
  params.neutral_cost = config_.neutral_cost;

  const AStarResult result =
      runAStar(cm, params, sx, sy, target_x, target_y, canceled_);
  if (canceled_.load()) {
    message = "Planning canceled";
    return kCanceledOutcome;
  }
  if (!result.found || result.cells.empty()) {
    message = "No path found";
    return ayanami::actions::GetPathResult::NO_PATH_FOUND;
  }

  // 构建 plan: cell → world pose。
  plan.header.frame_id =
      start.header.frame_id.empty() ? "map" : start.header.frame_id;
  plan.poses.reserve(result.cells.size());
  for (const auto& cell : result.cells) {
    double wx = 0.0;
    double wy = 0.0;
    cm->mapToWorld(cell.x, cell.y, wx, wy);
    auto p = geometry_msgs::PoseStamped::fromIsometry(
        plan.header.frame_id, Eigen::Isometry3d::Identity());
    p.pose.translation() = Eigen::Vector3d(wx, wy, 0.0);
    plan.poses.push_back(p);
  }

  if (!plan.poses.empty()) {
    plan.poses.front().pose = start.pose;
    if (approximate) {
      // 近似 goal: 保持最近可行格位置, 只对齐朝向。
      plan.poses.back().pose.linear() = goal.pose.linear();
    } else {
      plan.poses.back().pose = goal.pose;
    }
    for (std::size_t i = 1; i + 1 < plan.poses.size(); ++i) {
      const Eigen::Vector3d d =
          plan.poses[i + 1].pose.translation() - plan.poses[i].pose.translation();
      const double yaw = std::atan2(d.y(), d.x());
      plan.poses[i].pose.linear() =
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
  }

  cost = result.cost;
  message = approximate ? "Smac planner found an approximate path"
                        : "Smac planner found a path";
  return ayanami::actions::GetPathResult::SUCCESS;
}

}  // namespace modules
}  // namespace ayanami
