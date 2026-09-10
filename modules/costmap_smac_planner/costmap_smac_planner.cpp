// modules/costmap_smac_planner -- A* global path planning module.
//
// Pure C++ (zero ROS): runs on the composed global+local overlay
// (PlanningOverlay; family read contract: planner = global+local
// superposition, per-cell conservative max; NO_INFORMATION dominates),
// standard 4/8-connected A*. Rewritten with reference to _legacy SmacPlannerAStar semantics:
// - Blocking: cell cost >= INSCRIBED_INFLATED_OBSTACLE (lethal from 253 on)
// - Traversal cost: step * (neutral_cost + cost_factor * cell_cost)
// - Goal inside an obstacle: nearestValidGoal searches for the nearest valid cell within tolerance (approximate path)
// - cancel: checked node by node in the search loop; once interrupted, returns ErrorCode::CANCELED

#include "costmap_smac_planner.hpp"

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

constexpr double k_diagonal_step = std::sqrt(2.0);

struct Cell {
  unsigned int x{0};
  unsigned int y{0};
};

// A* search parameters (extracted from the private Param; the anonymous namespace cannot access private types).
struct SearchParams {
  bool use_diagonal{true};
  double cost_factor{0.5};
  double neutral_cost{1.0};
};

// Overlay cost accessor: cost of an overlay cell under the family read
// contract (planner = global+local superposition). Provided by the caller
// (PlanningOverlay::costAt); kept as a template parameter so A* stays
// allocation-free on the hot path.

// Standard A*. Returns {found, cost, cells(start->goal)}. canceled is checked node by node.
struct AStarResult {
  bool found{false};
  double cost{0.0};
  std::vector<Cell> cells;
};

template <typename CostAt>
AStarResult runAStar(const ayanami::costmap::PlanningOverlay& overlay,
                     const SearchParams& params,
                     CostAt&& cost_at, unsigned int sx, unsigned int sy,
                     unsigned int gx, unsigned int gy,
                     const std::atomic<bool>& canceled) {
  const auto blocked = [&cost_at](unsigned int x, unsigned int y) {
    return cost_at(x, y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  };
  const unsigned int width = overlay.size_x;
  const unsigned int height = overlay.size_y;
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
      if (blocked(ux, uy)) {
        continue;
      }
      // 8-connected forbids cutting corners: both orthogonal neighbors must also be valid.
      if (off.first != 0 && off.second != 0) {
        if (blocked(cur.x + static_cast<unsigned int>(off.first), cur.y) ||
            blocked(cur.x, cur.y + static_cast<unsigned int>(off.second))) {
          continue;
        }
      }
      const std::size_t nidx = index(ux, uy);
      if (closed[nidx]) {
        continue;
      }
      const bool diagonal = (off.first != 0 && off.second != 0);
      const double step = diagonal ? k_diagonal_step : 1.0;
      const double move_cost =
          step * (params.neutral_cost +
                  params.cost_factor * static_cast<double>(cost_at(ux, uy)));
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

// Finds the nearest valid cell within radius=tolerance_cells around (gx,gy) (including itself).
template <typename CostAt>
bool nearestValidGoal(const ayanami::costmap::PlanningOverlay& overlay,
                      CostAt&& cost_at,
                      unsigned int gx, unsigned int gy,
                      double tolerance_cells, unsigned int& ox,
                      unsigned int& oy) {
  const auto blocked = [&cost_at](unsigned int x, unsigned int y) {
    return cost_at(x, y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  };
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
        if (ux >= overlay.size_x || uy >= overlay.size_y) {
          continue;
        }
        if (blocked(ux, uy)) {
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


void CostmapSmacPlanner::configureParameters(const ayanami::ModuleContext& context) {
  // Parameter loading, separated from configuration: the loader calls this
  // right after a successful load, and every value lands in the module's own
  // CostmapSmacPlannerParam member (whose field initializers are the defaults for keys the
  // config omits).
  // Parameters are read from the ModuleContext-injected own configuration (the module does not know the global config layout);
  // missing config keys keep the default values.
  const Config& file = context.config;
  // Param section convention: the section key = the module json "name"
  // field, delivered as ModuleContext::name.
  const std::string& section = context.name;
  param.use_diagonal = file.param<bool>(section, "use_diagonal",
                                        param.use_diagonal);
  param.tolerance = file.param<double>(section, "tolerance",
                                       param.tolerance);
  param.cost_factor = file.param<double>(section, "cost_factor",
                                         param.cost_factor);
  param.lethal_cost = file.param<int>(section, "lethal_cost",
                                      param.lethal_cost);
  param.neutral_cost = file.param<double>(section,
                                          "neutral_cost", param.neutral_cost);
  param.use_final_approach =
      file.param<bool>(section, "use_final_approach",
                       param.use_final_approach);
}

void CostmapSmacPlanner::configure(
    const ModuleContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // Direct-construction callers (tests, or a host that builds the object
  // itself) never go through the loader's configureParameters call, so bind
  // the parameters here as well. Re-reading the same keys from the same
  // config is idempotent.
  configureParameters(context);
  map_context = std::move(map);
}


bool CostmapSmacPlanner::cancel() {
  canceled.store(true);
  return true;
}

bool CostmapSmacPlanner::setNewGoal(
    const geometry_msgs::PoseStamped&, const geometry_msgs::PoseStamped&,
    double) {
  goal_revision.fetch_add(1);
  // Interrupt the current A* loop. PlannerExecution observes its own goal
  // revision and immediately invokes makePlan again on the same thread.
  canceled.store(true);
  return true;
}

ResultT<void> CostmapSmacPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, double tolerance,
    nav_msgs::Path& plan_out) {
  canceled.store(false);

  if (!map_context) {
    return ResultT<void>::failure(
        ErrorCode::MAP_UNAVAILABLE, "CostmapSmacPlanner has no costmap module");
  }
  // Family read contract: the planner reads the global+local overlay.
  // composePlanningOverlay holds the two map shared locks ONLY for the O(N)
  // compose pass and releases them on return; the A* then queries the reused
  // member buffer lock-free in O(1) per cell, so the map writer is never
  // stalled by the search itself.
  if (!map_context->composePlanningOverlay(overlay_buffer)) {
    return ResultT<void>::failure(
        ErrorCode::MAP_UNAVAILABLE, "CostmapSmacPlanner has no global costmap");
  }
  const auto& overlay = overlay_buffer;
  const auto cost_at = [&overlay](unsigned int mx, unsigned int my) {
    return overlay.costAt(mx, my);
  };

  // Interface contract (AbstractPlanner): `start` is always a fully resolved
  // pose -- the execution layer substitutes the current robot pose for an
  // empty start before the module is called.
  const geometry_msgs::PoseStamped& start_pose = start;

  if (!start_pose.header.frame_id.empty() && !goal.header.frame_id.empty() &&
      start_pose.header.frame_id != goal.header.frame_id) {
    return ResultT<void>::failure(ErrorCode::TRANSFORM_UNAVAILABLE,
                                   "Start and goal frames do not match");
  }

  unsigned int sx = 0;
  unsigned int sy = 0;
  unsigned int gx = 0;
  unsigned int gy = 0;
  if (!overlay.worldToMap(start_pose.pose.position.x(),
                          start_pose.pose.position.y(), sx, sy)) {
    return ResultT<void>::failure(ErrorCode::START_INVALID,
                                   "Start pose is outside the global costmap");
  }
  if (!overlay.worldToMap(goal.pose.position.x(), goal.pose.position.y(),
                          gx, gy)) {
    return ResultT<void>::failure(ErrorCode::GOAL_INVALID,
                                   "Goal pose is outside the global costmap");
  }

  if (overlay.blockedCell(sx, sy)) {
    return ResultT<void>::failure(ErrorCode::START_INVALID,
                                   "Start pose is blocked");
  }

  const double tolerance_m = tolerance > 0.0 ? tolerance : param.tolerance;
  const double tolerance_cells =
      tolerance_m / std::max(overlay.resolution, 1e-9);

  // Goal inside an obstacle: nearestValidGoal finds the nearest valid cell within tolerance.
  unsigned int target_x = gx;
  unsigned int target_y = gy;
  bool approximate = false;
  if (overlay.blockedCell(gx, gy)) {
    if (!nearestValidGoal(overlay, cost_at, gx, gy, tolerance_cells, target_x,
                          target_y)) {
      return ResultT<void>::failure(
          ErrorCode::GOAL_INVALID,
          "Goal pose is blocked and no valid cell within tolerance");
    }
    approximate = true;
  }

  SearchParams params;
  params.use_diagonal = param.use_diagonal;
  params.cost_factor = param.cost_factor;
  params.neutral_cost = param.neutral_cost;

  const AStarResult result =
      runAStar(overlay, params, cost_at, sx, sy, target_x, target_y, canceled);
  if (canceled.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED, "Planning canceled");
  }
  if (!result.found || result.cells.empty()) {
    return ResultT<void>::failure(ErrorCode::PATH_NOT_FOUND,
                                   "No path found");
  }

  // Build the plan: cell -> world pose (overlay frame = global costmap frame).
  nav_msgs::Path plan;
  plan.header.frame_id = overlay.frame;
  plan.poses.reserve(result.cells.size());
  for (const auto& cell : result.cells) {
    double wx = 0.0;
    double wy = 0.0;
    overlay.mapToWorld(cell.x, cell.y, wx, wy);
    auto p = geometry_msgs::PoseStamped::fromIsometry(
        plan.header.frame_id, Eigen::Isometry3d::Identity());
    p.pose.position = Eigen::Vector3d(wx, wy, 0.0);
    plan.poses.push_back(p);
  }

  if (!plan.poses.empty()) {
    plan.poses.front().pose = start_pose.pose;
    if (approximate) {
      // Approximate goal: keep the nearest valid cell position and align only the heading.
      plan.poses.back().pose.orientation = goal.pose.orientation;
    } else {
      plan.poses.back().pose = goal.pose;
    }
    for (std::size_t i = 1; i + 1 < plan.poses.size(); ++i) {
      const Eigen::Vector3d d =
          plan.poses[i + 1].pose.position - plan.poses[i].pose.position;
      const double yaw = std::atan2(d.y(), d.x());
      plan.poses[i].pose.orientation =
          Eigen::Quaterniond(
              Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    }
  }

  plan_out = std::move(plan);
  auto out = ResultT<void>::success();
  out.message = approximate ? "Smac planner found an approximate path"
                            : "Smac planner found a path";
  return out;
}

}  // namespace modules
}  // namespace ayanami
