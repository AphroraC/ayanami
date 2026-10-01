// modules/costmap_astar_planner -- A* global path planning module.
//
// Pure C++ (zero ROS): plans on the OVERLAY of the held global grid copy and
// a fresh local snapshot (static-global policy: global = static+inflation,
// fresh dynamic obstacle marks live only in the local rolling map). The
// overlay fusion is a per-point max (FusedMapView) -- the two grids' cost
// sources are disjoint, each carries its own inflation. All work runs on
// these immutable inputs with no map lock held (the global copy refreshes
// only when the writer's generation advances; the local copy is one
// short-lock snapshot per call), so the map writer is never delayed by the
// search.
// Standard 4/8-connected A*. Rewritten with reference to _legacy
// AstarPlannerAStar semantics:
// - Blocking: fused cell cost >= INSCRIBED_INFLATED_OBSTACLE (lethal from 253 on)
// - Traversal cost: step * (neutral_cost + cost_factor * cell_cost)
// - Goal inside an obstacle: nearestValidGoal searches for the nearest valid cell within tolerance (approximate path)
// - Start tolerance: a start inside the inscribed band (253) is planned from as-is in ESCAPE mode
//   (only LETHAL cells block: the band itself must be traversable to leave it); a LETHAL (254)
//   start is rejected
// - cancel: checked node by node in the search loop; once interrupted, returns ErrorCode::CANCELED

#include "costmap_astar_planner.hpp"

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

// Standard A* over the fused overlay view. Returns {found, cost,
// cells(start->goal)}. canceled is checked node by node. The view is
// immutable: the search holds NO map lock (the caller prepared the held
// global copy and the short-lock local snapshot before calling).
struct AStarResult {
  bool found{false};
  double cost{0.0};
  std::vector<Cell> cells;
};

AStarResult runAStar(const ayanami::costmap::FusedMapView& view,
                     const SearchParams& params, unsigned int sx,
                     unsigned int sy, unsigned int gx, unsigned int gy,
                     const std::atomic<bool>& canceled, bool escape_mode) {
  const auto blocked = [&view, escape_mode](unsigned int x, unsigned int y) {
    const unsigned char cost = view.costAtCell(x, y);
    return cost >= (escape_mode ? costmap_2d::LETHAL_OBSTACLE
                                : costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  };
  const auto& grid = *view.global;
  const unsigned int width = grid.size_x;
  const unsigned int height = grid.size_y;
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
                  params.cost_factor *
                      static_cast<double>(view.costAtCell(ux, uy)));
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
bool nearestValidGoal(const ayanami::costmap::FusedMapView& view,
                      unsigned int gx, unsigned int gy,
                      double tolerance_cells, unsigned int& ox,
                      unsigned int& oy) {
  const auto blocked = [&view](unsigned int x, unsigned int y) {
    return view.blockedAtCell(x, y);
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
        if (ux >= view.global->size_x || uy >= view.global->size_y) {
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


void CostmapAstarPlanner::configureParameters(const ayanami::ModuleContext& context) {
  // Parameter loading, separated from configuration: the loader calls this
  // right after a successful load, and every value lands in the module's own
  // CostmapAstarPlannerParam member (whose field initializers are the defaults for keys the
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

void CostmapAstarPlanner::configure(
    const ModuleContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // Direct-construction callers (tests, or a host that builds the object
  // itself) never go through the loader's configureParameters call, so bind
  // the parameters here as well. Re-reading the same keys from the same
  // config is idempotent.
  configureParameters(context);
  map_context = std::move(map);
}


bool CostmapAstarPlanner::cancel() {
  canceled.store(true);
  return true;
}

ResultT<void> CostmapAstarPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, double tolerance,
    nav_msgs::Path& plan_out) {
  canceled.store(false);

  if (!map_context) {
    return ResultT<void>::failure(
        ErrorCode::MAP_UNAVAILABLE, "CostmapAstarPlanner has no costmap module");
  }
  // Global grid: held copy refreshed lazily by generation. The global map
  // is static (static+inflation), so after the first copy every planning
  // call is one atomic load; the writer bumps the generation on every real
  // mutation (clear commands included), which triggers exactly one re-copy.
  const std::uint64_t global_gen = map_context->globalGeneration();
  if (!have_global_copy || global_gen != global_copy_generation) {
    if (!map_context->snapshotGlobal(global_copy)) {
      return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                    "CostmapAstarPlanner has no global costmap");
    }
    global_copy_generation = global_copy.generation;
    have_global_copy = true;
  }
  // Local overlay: one short-lock snapshot per planning call (the context
  // reuses the buffer's capacity). An absent or unreadable local map, or an
  // unavailable map<-odom transform, degrades to global-only planning --
  // same graceful degradation rule as the smoother's collision check.
  ayanami::costmap::FusedMapView view;
  view.global = &global_copy;
  if (map_context->snapshotLocal(local_snapshot) && local_snapshot.valid()) {
    if (map_context->localFromGlobal(view.local_from_global)) {
      view.local = &local_snapshot;
    }
  }
  return makePlanWithMap(start, goal, tolerance, plan_out, view);
}

ResultT<void> CostmapAstarPlanner::makePlanWithMap(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, double tolerance,
    nav_msgs::Path& plan_out, const ayanami::costmap::FusedMapView& view) {
  const auto* grid = view.global;
  const auto& snapshot = *grid;
  if (!view.valid()) {
    return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                  "CostmapAstarPlanner has no global costmap");
  }

  // Interface contract (AbstractPlanner): `start` is always a fully resolved
  // pose -- the execution layer substitutes the current robot pose for an
  // empty start before the module is called.
  const geometry_msgs::PoseStamped& start_pose = start;

  if (!start_pose.header.frame_id.empty() && !goal.header.frame_id.empty() &&
      start_pose.header.frame_id != goal.header.frame_id) {
    return ResultT<void>::failure(ErrorCode::TRANSFORM_UNAVAILABLE,
                                   "Start and goal frames do not match");
  }
  if (!start_pose.header.frame_id.empty() &&
      start_pose.header.frame_id != snapshot.frame) {
    return ResultT<void>::failure(
        ErrorCode::TRANSFORM_UNAVAILABLE,
        "Start frame '" + start_pose.header.frame_id +
            "' does not match the global costmap frame '" + snapshot.frame +
            "'");
  }

  unsigned int sx = 0;
  unsigned int sy = 0;
  unsigned int gx = 0;
  unsigned int gy = 0;
  if (!grid->worldToMap(start_pose.pose.position.x(),
                        start_pose.pose.position.y(), sx, sy)) {
    return ResultT<void>::failure(ErrorCode::START_INVALID,
                                   "Start pose is outside the global costmap");
  }
  if (!grid->worldToMap(goal.pose.position.x(), goal.pose.position.y(),
                        gx, gy)) {
    return ResultT<void>::failure(ErrorCode::GOAL_INVALID,
                                   "Goal pose is outside the global costmap");
  }

  const auto blocked = [&view](unsigned int x, unsigned int y) {
    return view.blockedAtCell(x, y);
  };
  // Start tolerance (nav2 NavFn parity): the robot is already wherever it
  // is. A start inside the inscribed band (253) is the inflation verdict
  // around it -- planning FROM it is how the robot drives back out; hard
  // failing here deadlocks a band-grazing tracker (every replan reports
  // START_INVALID, a non-recoverable code, so no recovery can ever fix it).
  // Only a LETHAL start (inside a mapped obstacle: a TF/placement fault)
  // is rejected.
  const unsigned char start_cost = view.costAtCell(sx, sy);
  if (start_cost >= costmap_2d::LETHAL_OBSTACLE) {
    return ResultT<void>::failure(ErrorCode::START_INVALID,
                                  "Start pose is inside a lethal obstacle");
  }
  // Escape mode: with the start inside the band, the whole band thickness
  // toward free space is itself >= 253 -- the strict predicate would leave
  // the search nowhere to expand and every replan would report
  // PATH_NOT_FOUND. While escaping, only LETHAL cells block (band cells
  // stay heavily penalized through the traversal cost, so the search leaves
  // the band at the first opportunity); a start outside the band keeps the
  // strict predicate.
  const bool escape_mode =
      start_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

  // Tolerance is CONFIG-owned (costmap_astar_planner.json::tolerance), not
  // goal-owned: the BT exposes no tolerance port and a fixed goal-side
  // default would silently override the module's own configuration. The
  // interface parameter is accepted for the AbstractPlanner contract and
  // ignored here.
  (void)tolerance;
  const double tolerance_m = param.tolerance;
  const double tolerance_cells =
      tolerance_m / std::max(grid->resolution, 1e-9);

  // Goal inside an obstacle: nearestValidGoal finds the nearest valid cell within tolerance.
  unsigned int target_x = gx;
  unsigned int target_y = gy;
  bool approximate = false;
  if (blocked(gx, gy)) {
    if (!nearestValidGoal(view, gx, gy, tolerance_cells, target_x,
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
      runAStar(view, params, sx, sy, target_x, target_y, canceled,
               escape_mode);
  if (canceled.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED, "Planning canceled");
  }
  if (!result.found || result.cells.empty()) {
    return ResultT<void>::failure(ErrorCode::PATH_NOT_FOUND,
                                   "No path found");
  }

  // Build the plan: cell -> world pose (grid frame = global costmap frame).
  nav_msgs::Path plan;
  plan.header.frame_id = snapshot.frame;
  plan.poses.reserve(result.cells.size());
  for (const auto& cell : result.cells) {
    double wx = 0.0;
    double wy = 0.0;
    grid->mapToWorld(cell.x, cell.y, wx, wy);
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
  out.message = approximate ? "A* planner found an approximate path"
                            : "A* planner found a path";
  return out;
}

}  // namespace modules
}  // namespace ayanami
