// Task 21: modules/costmap_smac_planner — A* 全局路径规划模块。
//
// 纯 C++ (零 ROS): 在 CostmapView 加锁零拷贝读图 (global master grid) 上做
// 标准 4/8 连通 A*。参考 _legacy SmacPlannerAStar 语义但重写:
// - 阻断: cell cost >= INSCRIBED_INFLATED_OBSTACLE (Ruling #6, 253 起 lethal)
// - 行走代价: step * (neutral_cost + cost_factor * cell_cost)
// - goal 在障碍内: nearestValidGoal 在 tolerance 内搜索最近可行格 (近似路径)
// - cancel(): 搜索循环逐节点检查, 打断后返回 ErrorCode::CANCELED

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

// A* 搜索参数 (从私有 Param 提取, 匿名命名空间无需访问私有类型)。
struct SearchParams {
  bool use_diagonal{true};
  double cost_factor{0.5};
  double neutral_cost{1.0};
};

bool isBlocked(const costmap_2d::Costmap2D* cm, unsigned int x, unsigned int y) {
  return cm->getCost(x, y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

// 标准 A*。返回 {found, cost, cells(start→goal)}。canceled 逐节点检查。
struct AStarResult {
  bool found{false};
  double cost{0.0};
  std::vector<Cell> cells;
};

AStarResult runAStar(const costmap_2d::Costmap2D* cm, const SearchParams& params,
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
      const double step = diagonal ? k_diagonal_step : 1.0;
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
bool nearestValidGoal(const costmap_2d::Costmap2D* cm, unsigned int gx,
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


void CostmapSmacPlanner::configure(
    const PluginContext& context,
    ayanami::costmap::CostmapMapContext::ConstPtr map) {
  // 参数经 PluginContext 注入的自身配置读取 (插件不知道全局配置布局);
  // 配置键缺失时保持默认值。
  const Config& file = context.config;
  param.use_diagonal = file.param<bool>("costmap_smac_planner", "use_diagonal",
                                        param.use_diagonal);
  param.tolerance = file.param<double>("costmap_smac_planner", "tolerance",
                                       param.tolerance);
  param.cost_factor = file.param<double>("costmap_smac_planner", "cost_factor",
                                         param.cost_factor);
  param.lethal_cost = file.param<int>("costmap_smac_planner", "lethal_cost",
                                      param.lethal_cost);
  param.neutral_cost = file.param<double>("costmap_smac_planner",
                                          "neutral_cost", param.neutral_cost);
  param.use_final_approach =
      file.param<bool>("costmap_smac_planner", "use_final_approach",
                       param.use_final_approach);
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

PlannerResultT CostmapSmacPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, double tolerance) {
  canceled.store(false);

  if (!map_context) {
    return PlannerResultT::failure(
        ErrorCode::MAP_UNAVAILABLE, "CostmapSmacPlanner has no costmap module");
  }
  // 持 shared 锁读图 (整个规划持有, <10ms), 零拷贝; 地图写者等锁写入。
  auto costmap = map_context->lockGlobal();

  const costmap_2d::Costmap2D* cm = costmap.grid();
  if (!cm) {
    return PlannerResultT::failure(
        ErrorCode::MAP_UNAVAILABLE, "CostmapSmacPlanner has no global costmap");
  }

  // 3.1 minor carry: start pose 留空 (默认构造: 平移全零 + frame 空) 时回退到
  // 读视图解析的机器人位姿 (视图构建时经 TransformStore 解析)。
  geometry_msgs::PoseStamped start_pose = start;
  if (start_pose.pose.position.norm() == 0.0 &&
      start_pose.header.frame_id.empty() && costmap.has_robot_pose()) {
    start_pose = costmap.robot_pose();
  }

  if (!start_pose.header.frame_id.empty() && !goal.header.frame_id.empty() &&
      start_pose.header.frame_id != goal.header.frame_id) {
    return PlannerResultT::failure(ErrorCode::TRANSFORM_UNAVAILABLE,
                                   "Start and goal frames do not match");
  }

  unsigned int sx = 0;
  unsigned int sy = 0;
  unsigned int gx = 0;
  unsigned int gy = 0;
  if (!cm->worldToMap(start_pose.pose.position.x(),
                      start_pose.pose.position.y(), sx, sy)) {
    return PlannerResultT::failure(ErrorCode::START_INVALID,
                                   "Start pose is outside the global costmap");
  }
  if (!cm->worldToMap(goal.pose.position.x(), goal.pose.position.y(),
                      gx, gy)) {
    return PlannerResultT::failure(ErrorCode::GOAL_INVALID,
                                   "Goal pose is outside the global costmap");
  }

  if (isBlocked(cm, sx, sy)) {
    return PlannerResultT::failure(ErrorCode::START_INVALID,
                                   "Start pose is blocked");
  }

  const double tolerance_m = tolerance > 0.0 ? tolerance : param.tolerance;
  const double tolerance_cells =
      tolerance_m / std::max(cm->getResolution(), 1e-9);

  // goal 在障碍内: nearestValidGoal 在 tolerance 内找最近可行格。
  unsigned int target_x = gx;
  unsigned int target_y = gy;
  bool approximate = false;
  if (isBlocked(cm, gx, gy)) {
    if (!nearestValidGoal(cm, gx, gy, tolerance_cells, target_x, target_y)) {
      return PlannerResultT::failure(
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
      runAStar(cm, params, sx, sy, target_x, target_y, canceled);
  if (canceled.load()) {
    return PlannerResultT::failure(ErrorCode::CANCELED, "Planning canceled");
  }
  if (!result.found || result.cells.empty()) {
    return PlannerResultT::failure(ErrorCode::PATH_NOT_FOUND,
                                   "No path found");
  }

  // 构建 plan: cell → world pose。
  nav_msgs::Path plan;
  plan.header.frame_id =
      start_pose.header.frame_id.empty() ? "map" : start_pose.header.frame_id;
  plan.poses.reserve(result.cells.size());
  for (const auto& cell : result.cells) {
    double wx = 0.0;
    double wy = 0.0;
    cm->mapToWorld(cell.x, cell.y, wx, wy);
    auto p = geometry_msgs::PoseStamped::fromIsometry(
        plan.header.frame_id, Eigen::Isometry3d::Identity());
    p.pose.position = Eigen::Vector3d(wx, wy, 0.0);
    plan.poses.push_back(p);
  }

  if (!plan.poses.empty()) {
    plan.poses.front().pose = start_pose.pose;
    if (approximate) {
      // 近似 goal: 保持最近可行格位置, 只对齐朝向。
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

  auto out = PlannerResultT::success(std::move(plan));
  out.message = approximate ? "Smac planner found an approximate path"
                            : "Smac planner found a path";
  return out;
}

}  // namespace modules
}  // namespace ayanami
