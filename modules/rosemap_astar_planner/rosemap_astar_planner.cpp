// modules/rosemap_astar_planner -- ESDF-native A* global planner.
//
// Pure C++ (zero ROS). Search skeleton ported from rose_navigation
// path_search/a*.cpp (MIT); turn-speed limiting + velocity propagation
// ported from navi_minco_bit minco_planner PTAllocation semantics (concept
// reimplementation, see the license note in the module README). All work
// runs on one immutable RosemapSnapshot -- the field writer is never
// delayed by planning.

#include "rosemap_astar_planner.hpp"

#include <ayanami/utility/config.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>

namespace ayanami {
namespace modules {

namespace {

constexpr double k_infinity = std::numeric_limits<double>::infinity();
constexpr double k_diagonal_step = 1.4142135623730951;

// ---- time parameterization (navi PTAllocation concept) ----

// Turn-speed limit at one path vertex: angles beyond the deadzone decay the
// global cruise speed exponentially down to turn_min_vel.
double curvatureDecay(double angle, double global_vmax, double deadzone,
                      double saturation, double min_vel,
                      double decay_power) {
  if (angle < deadzone) {
    return global_vmax;
  }
  const double clamped_angle = std::min(angle, saturation);
  const double ratio = (clamped_angle - deadzone) / saturation;
  const double local_vmax =
      min_vel + (global_vmax - min_vel) * std::exp(-ratio * decay_power);
  return std::max(local_vmax, min_vel);
}

// Local speed cap for the segment ENDING at vertex seg_idx + 1, from the
// turn angle between segment seg_idx and seg_idx + 1.
double turnSpeedAtSegmentEnd(const std::vector<Eigen::Vector2d>& path,
                             int seg_idx, double global_vmax,
                             double deadzone, double saturation,
                             double min_turn_vel, double decay_power) {
  const int n = static_cast<int>(path.size());
  if (seg_idx < 0 || seg_idx + 2 >= n) {
    return global_vmax;
  }
  const Eigen::Vector2d d1 = path[seg_idx + 1] - path[seg_idx];
  const Eigen::Vector2d d2 = path[seg_idx + 2] - path[seg_idx + 1];
  const double n1 = d1.norm();
  const double n2 = d2.norm();
  if (n1 <= 1e-6 || n2 <= 1e-6) {
    return global_vmax;
  }
  const double dot = std::clamp(d1.dot(d2) / (n1 * n2), -1.0, 1.0);
  return curvatureDecay(std::acos(dot), global_vmax, deadzone, saturation,
                        min_turn_vel, decay_power);
}

// Forward speed bound: accelerate at most amax per segment, capped by the
// local turn limit and by the remaining braking distance.
double computeNextSpeed(double v_current, double seg_len,
                        double remain_after, double a_max,
                        double local_vmax) {
  if (!std::isfinite(v_current) || v_current < 0.0) {
    v_current = 0.0;
  }
  if (!std::isfinite(seg_len) || seg_len <= 0.0) {
    return 0.0;
  }
  const double a_safe = std::max(1e-3, std::abs(a_max));
  double v_next = std::sqrt(std::max(0.0, v_current * v_current + 2.0 * a_safe * seg_len));
  if (std::isfinite(local_vmax) && local_vmax > 0.0) {
    v_next = std::min(v_next, local_vmax);
  }
  const double v_cap_stop =
      remain_after > 1e-6
          ? std::sqrt(std::max(0.0, 2.0 * a_safe * remain_after))
          : 0.0;
  v_next = std::min(v_next, v_cap_stop);
  if (!std::isfinite(v_next) || v_next < 0.0) {
    return 0.0;
  }
  return v_next;
}

// Duration of one segment connecting speeds v_current -> v_next over
// seg_len (trapezoid-compatible; never below min_seg_time).
double computeSegmentTime(double seg_len, double v_current, double v_next,
                          double a_max, double min_seg_time = 0.05) {
  const double t_min = std::max(1e-3, min_seg_time);
  double t = 0.0;
  const double v_sum = v_current + v_next;
  if (v_sum > 1e-3) {
    t = 2.0 * seg_len / v_sum;
  } else {
    t = seg_len / 0.1;
  }
  const double a_safe = std::max(1e-3, std::abs(a_max));
  const double required_dist =
      std::abs(v_current * v_current - v_next * v_next) / (2.0 * a_safe);
  if (seg_len >= required_dist) {
    t = std::max(t, std::abs(v_current - v_next) / a_safe);
  }
  if (!std::isfinite(t) || t < t_min) {
    return t_min;
  }
  return t;
}

// ---- search model over one snapshot ----

struct Cell {
  int x{0};
  int y{0};
};

struct AStarResult {
  bool found{false};
  bool timed_out{false};
  std::vector<Cell> cells;
};

class FieldSearchModel {
 public:
  explicit FieldSearchModel(const ayanami::rosemap::RosemapSnapshot& snapshot,
                            double safe_radius)
      : snapshot_(snapshot), safe_radius_(safe_radius) {}

  int sizeX() const { return snapshot_.size_x; }
  int sizeY() const { return snapshot_.size_y; }

  bool inBounds(int x, int y) const {
    return x >= 0 && y >= 0 && x < snapshot_.size_x && y < snapshot_.size_y;
  }

  double distance(int x, int y) const {
    return static_cast<double>(snapshot_.distanceAt(x, y));
  }

  bool blocked(int x, int y) const {
    return !inBounds(x, y) || distance(x, y) < safe_radius_;
  }

  Eigen::Vector2d cellCenter(int x, int y) const {
    double wx = 0.0;
    double wy = 0.0;
    snapshot_.mapToWorld(x, y, wx, wy);
    return {wx, wy};
  }

 private:
  const ayanami::rosemap::RosemapSnapshot& snapshot_;
  double safe_radius_;
};

struct PQItem {
  int id;
  double f;
  double h;
};

struct PQComp {
  bool operator()(const PQItem& a, const PQItem& b) const {
    constexpr double k_eps = 1e-4;
    if (std::abs(a.f - b.f) < k_eps) {
      return a.h > b.h;  // tie-break toward the goal
    }
    return a.f > b.f;
  }
};

AStarResult runAStar(const FieldSearchModel& model, bool use_diagonal,
                     double obstacle_penalty_weight, double heuristic_weight,
                     const Cell& start, const Cell& goal,
                     const std::atomic<bool>& canceled, double max_time_s,
                     int max_iterations) {
  const int size = model.sizeX() * model.sizeY();
  if (size <= 0) {
    return AStarResult{};
  }
  std::vector<double> g_score(static_cast<std::size_t>(size), k_infinity);
  std::vector<int> parent(static_cast<std::size_t>(size), -1);
  std::vector<unsigned char> closed(static_cast<std::size_t>(size), 0);

  const auto index = [&model](int x, int y) {
    return static_cast<std::size_t>(y) * model.sizeX() + x;
  };
  const auto heuristic = [&](int x, int y) {
    const double dx = static_cast<double>(goal.x - x);
    const double dy = static_cast<double>(goal.y - y);
    return heuristic_weight * std::hypot(dx, dy);
  };

  struct Node {
    double f{0.0};
    int x{0};
    int y{0};
  };
  const auto cmp = [](const Node& a, const Node& b) { return a.f > b.f; };
  std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);

  g_score[index(start.x, start.y)] = 0.0;
  open.push(Node{heuristic(start.x, start.y), start.x, start.y});

  const std::vector<std::pair<int, int>> offsets =
      use_diagonal
          ? std::vector<std::pair<int, int>>{{-1, -1}, {-1, 0}, {-1, 1},
                                             {0, -1},  {0, 1},  {1, -1},
                                             {1, 0},   {1, 1}}
          : std::vector<std::pair<int, int>>{
                {-1, 0}, {1, 0}, {0, -1}, {0, 1}};

  const auto start_time = std::chrono::steady_clock::now();
  int iterations = 0;
  while (!open.empty()) {
    ++iterations;
    if ((iterations & 0xFF) == 0) {
      if (canceled.load()) {
        return AStarResult{};
      }
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        start_time)
              .count();
      if (elapsed > max_time_s || iterations > max_iterations) {
        AStarResult timeout;
        timeout.timed_out = true;
        return timeout;
      }
    }
    const Node current = open.top();
    open.pop();
    const std::size_t current_index = index(current.x, current.y);
    if (closed[current_index] != 0) {
      continue;
    }
    closed[current_index] = 1;

    if (current.x == goal.x && current.y == goal.y) {
      AStarResult result;
      result.found = true;
      int trace = static_cast<int>(current_index);
      while (trace >= 0) {
        result.cells.push_back(Cell{trace % model.sizeX(),
                                    trace / model.sizeX()});
        trace = parent[static_cast<std::size_t>(trace)];
      }
      std::reverse(result.cells.begin(), result.cells.end());
      return result;
    }

    for (const auto& offset : offsets) {
      const int nx = current.x + offset.first;
      const int ny = current.y + offset.second;
      if (model.blocked(nx, ny)) {
        continue;
      }
      // 8-connected forbids corner cutting: both orthogonal neighbors must
      // also be clear.
      if (offset.first != 0 && offset.second != 0) {
        if (model.blocked(current.x + offset.first, current.y) ||
            model.blocked(current.x, current.y + offset.second)) {
          continue;
        }
      }
      const std::size_t next_index = index(nx, ny);
      if (closed[next_index] != 0) {
        continue;
      }
      const double step =
          (offset.first != 0 && offset.second != 0) ? k_diagonal_step : 1.0;
      const double clearance = model.distance(nx, ny);
      const double step_cost =
          step + obstacle_penalty_weight * (1.0 / (clearance + 0.1));
      const double tentative = g_score[current_index] + step_cost;
      if (tentative < g_score[next_index]) {
        parent[next_index] = static_cast<int>(current_index);
        g_score[next_index] = tentative;
        open.push(Node{tentative + heuristic(nx, ny), nx, ny});
      }
    }
  }
  return AStarResult{};
}

}  // namespace

void RosemapAstarPlanner::configureParameters(
    const ayanami::ModuleContext& context) {
  const Config& file = context.config;
  const std::string& section = context.name;
  param.safe_radius =
      file.param<double>(section, "safe_radius", param.safe_radius);
  param.obstacle_penalty_weight = file.param<double>(
      section, "obstacle_penalty_weight", param.obstacle_penalty_weight);
  param.heuristic_weight =
      file.param<double>(section, "heuristic_weight", param.heuristic_weight);
  param.use_diagonal =
      file.param<bool>(section, "use_diagonal", param.use_diagonal);
  param.simplify_threshold = file.param<double>(section, "simplify_threshold",
                                                param.simplify_threshold);
  param.string_pull =
      file.param<bool>(section, "string_pull", param.string_pull);
  param.expected_speed =
      file.param<double>(section, "expected_speed", param.expected_speed);
  param.max_accel = file.param<double>(section, "max_accel", param.max_accel);
  param.turn_deadzone =
      file.param<double>(section, "turn_deadzone", param.turn_deadzone);
  param.turn_saturation =
      file.param<double>(section, "turn_saturation", param.turn_saturation);
  param.turn_min_vel =
      file.param<double>(section, "turn_min_vel", param.turn_min_vel);
  param.turn_decay_power =
      file.param<double>(section, "turn_decay_power", param.turn_decay_power);
  param.tolerance = file.param<double>(section, "tolerance", param.tolerance);
  param.max_search_time =
      file.param<double>(section, "max_search_time", param.max_search_time);
  param.max_iterations =
      file.param<int>(section, "max_iterations", param.max_iterations);
}

void RosemapAstarPlanner::configure(
    const ModuleContext& context,
    ayanami::rosemap::RosemapMapContext::ConstPtr map) {
  configureParameters(context);
  map_context = std::move(map);
}

bool RosemapAstarPlanner::cancel() {
  canceled.store(true);
  return true;
}

ResultT<void> RosemapAstarPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, double tolerance,
    nav_msgs::Path& plan_out) {
  canceled.store(false);
  if (!map_context) {
    return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                  "RosemapAstarPlanner has no field module");
  }
  const auto snapshot = map_context->currentSnapshot();
  if (!snapshot || !snapshot->valid()) {
    return ResultT<void>::failure(ErrorCode::MAP_UNAVAILABLE,
                                  "RosemapAstarPlanner has no field snapshot");
  }
  (void)tolerance;  // tolerance is config-owned (family convention)

  // Frame handling: paths are produced in the field frame; transform the
  // endpoints once when they arrive in another frame.
  Eigen::Isometry3d field_from_start = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d field_from_goal = Eigen::Isometry3d::Identity();
  if (!map_context->fieldFromFrame(start.header.frame_id, field_from_start)) {
    return ResultT<void>::failure(
        ErrorCode::TRANSFORM_UNAVAILABLE,
        "no transform field('" + snapshot->frame + "') <- '" +
            start.header.frame_id + "'");
  }
  if (!map_context->fieldFromFrame(goal.header.frame_id, field_from_goal)) {
    return ResultT<void>::failure(
        ErrorCode::TRANSFORM_UNAVAILABLE,
        "no transform field('" + snapshot->frame + "') <- '" +
            goal.header.frame_id + "'");
  }
  const Eigen::Vector3d start_position = field_from_start * start.pose.position;
  const Eigen::Vector3d goal_position = field_from_goal * goal.pose.position;
  const Eigen::Vector2d start_point(start_position.x(), start_position.y());
  const Eigen::Vector2d goal_point(goal_position.x(), goal_position.y());

  FieldSearchModel model(*snapshot, param.safe_radius);

  const auto toCell = [&snapshot](const Eigen::Vector2d& point, Cell& cell) {
    double mx = 0.0;
    double my = 0.0;
    if (!snapshot->worldToMap(point.x(), point.y(), mx, my)) {
      return false;
    }
    cell = Cell{static_cast<int>(std::lround(mx)),
                static_cast<int>(std::lround(my))};
    return true;
  };

  Cell start_cell;
  Cell goal_cell;
  if (!toCell(start_point, start_cell) ||
      model.blocked(start_cell.x, start_cell.y)) {
    return ResultT<void>::failure(ErrorCode::START_INVALID,
                                  "Start pose is outside the field or unsafe");
  }
  const bool goal_inside = toCell(goal_point, goal_cell);
  bool approximate = false;
  Eigen::Vector2d effective_goal = goal_point;
  if (!goal_inside) {
    // Out-of-field goal: project onto the field boundary along the
    // start->goal direction; the real goal is appended after the search.
    const Eigen::Vector2d direction = goal_point - start_point;
    const double length = direction.norm();
    if (length < 1e-6) {
      return ResultT<void>::failure(ErrorCode::GOAL_INVALID,
                                    "Goal equals start");
    }
    const Eigen::Vector2d unit = direction / length;
    const double min_x = snapshot->origin_x;
    const double min_y = snapshot->origin_y;
    const double max_x = min_x + snapshot->size_x * snapshot->resolution;
    const double max_y = min_y + snapshot->size_y * snapshot->resolution;
    double t_exit = k_infinity;
    for (int axis = 0; axis < 2; ++axis) {
      const double o = axis == 0 ? start_point.x() : start_point.y();
      const double d = axis == 0 ? unit.x() : unit.y();
      const double lo = axis == 0 ? min_x : min_y;
      const double hi = axis == 0 ? max_x : max_y;
      if (std::abs(d) < 1e-12) {
        continue;
      }
      const double t = (d > 0.0 ? hi - o : lo - o) / d;
      if (t > 0.0 && t < t_exit) {
        t_exit = t;
      }
    }
    if (!std::isfinite(t_exit)) {
      return ResultT<void>::failure(ErrorCode::GOAL_INVALID,
                                    "Goal direction never enters the field");
    }
    effective_goal = start_point + t_exit * unit * 0.999;
    if (!toCell(effective_goal, goal_cell)) {
      return ResultT<void>::failure(ErrorCode::GOAL_INVALID,
                                    "Goal projection outside the field");
    }
    approximate = true;
  } else if (model.blocked(goal_cell.x, goal_cell.y)) {
    const double tolerance_cells =
        param.tolerance / std::max(snapshot->resolution, 1e-9);
    Cell safe_goal;
    // Ring search around the goal within the configured tolerance.
    bool found = false;
    const int radius = static_cast<int>(std::ceil(std::max(0.0, tolerance_cells)));
    for (int r = 0; r <= radius && !found; ++r) {
      for (int dy = -r; dy <= r && !found; ++dy) {
        for (int dx = -r; dx <= r && !found; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) {
            continue;
          }
          const int nx = goal_cell.x + dx;
          const int ny = goal_cell.y + dy;
          if (model.blocked(nx, ny)) {
            continue;
          }
          safe_goal = Cell{nx, ny};
          found = true;
        }
      }
    }
    if (!found) {
      return ResultT<void>::failure(
          ErrorCode::GOAL_INVALID,
          "Goal pose is unsafe and no safe cell within tolerance");
    }
    goal_cell = safe_goal;
    approximate = true;
  }

  const AStarResult result = runAStar(
      model, param.use_diagonal, param.obstacle_penalty_weight,
      param.heuristic_weight, start_cell, goal_cell, canceled,
      param.max_search_time, param.max_iterations);
  if (canceled.load()) {
    return ResultT<void>::failure(ErrorCode::CANCELED, "Planning canceled");
  }
  if (result.timed_out) {
    return ResultT<void>::failure(ErrorCode::TIMEOUT,
                                  "A* exceeded the search budget");
  }
  if (!result.found || result.cells.empty()) {
    return ResultT<void>::failure(ErrorCode::PATH_NOT_FOUND,
                                  "No path found");
  }

  // Cell path -> world points; the endpoints are replaced by the exact
  // start/goal before simplification so they survive unchanged.
  std::vector<Eigen::Vector2d> points;
  points.reserve(result.cells.size());
  for (const auto& cell : result.cells) {
    points.push_back(model.cellCenter(cell.x, cell.y));
  }
  points.front() = start_point;
  points.back() =
      approximate && !goal_inside ? effective_goal : goal_point;

  // ---- simplification: DP with line-of-sight splitting + string pull ----
  // lineOfSight samples the segment at half-resolution steps and requires
  // safe_radius clearance throughout (verified-clear simplification).
  const auto lineOfSight = [&model, &snapshot, this](const Eigen::Vector2d& a,
                                          const Eigen::Vector2d& b) {
    const Eigen::Vector2d delta = b - a;
    const double length = delta.norm();
    const double step = std::max(0.5 * snapshot->resolution, 1e-6);
    const int steps =
        std::max(1, static_cast<int>(std::ceil(length / step)));
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      const Eigen::Vector2d point = a + t * delta;
      const auto query =
          ayanami::rosemap::sampleDistance(*snapshot, point.x(), point.y());
      if (query.status != ayanami::rosemap::RosemapQueryStatus::kOk ||
          query.distance < param.safe_radius) {
        return false;
      }
    }
    return true;
  };

  const std::function<std::vector<Eigen::Vector2d>(
      const std::vector<Eigen::Vector2d>&, int, int)>
      simplifyDp = [&](const std::vector<Eigen::Vector2d>& pts, int i0,
                       int i1) -> std::vector<Eigen::Vector2d> {
    if (i1 - i0 <= 0) {
      return {pts[i0]};
    }
    if (i1 - i0 == 1) {
      return {pts[i0], pts[i1]};
    }
    double max_deviation = 0.0;
    int max_index = i0;
    const Eigen::Vector2d chord = pts[i1] - pts[i0];
    const double chord_length = chord.norm();
    for (int i = i0 + 1; i < i1; ++i) {
      double deviation = 0.0;
      if (chord_length > 1e-9) {
        deviation = std::abs((pts[i] - pts[i0]).x() * chord.y() -
                             (pts[i] - pts[i0]).y() * chord.x()) /
                    chord_length;
      }
      if (deviation > max_deviation) {
        max_deviation = deviation;
        max_index = i;
      }
    }
    if (max_deviation > param.simplify_threshold ||
        !lineOfSight(pts[i0], pts[i1])) {
      auto head = simplifyDp(pts, i0, max_index);
      auto tail = simplifyDp(pts, max_index, i1);
      head.insert(head.end(), tail.begin() + 1, tail.end());
      return head;
    }
    return {pts[i0], pts[i1]};
  };

  std::vector<Eigen::Vector2d> simplified = simplifyDp(
      points, 0, static_cast<int>(points.size()) - 1);
  if (param.string_pull && simplified.size() > 2) {
    std::vector<Eigen::Vector2d> pulled;
    pulled.push_back(simplified.front());
    int current = 0;
    int next = 2;
    const int last = static_cast<int>(simplified.size()) - 1;
    while (current < last) {
      if (next > last) {
        pulled.push_back(simplified.back());
        break;
      }
      if (lineOfSight(simplified[current], simplified[next])) {
        ++next;
      } else {
        pulled.push_back(simplified[next - 1]);
        current = next - 1;
        next = current + 2;
      }
    }
    simplified = std::move(pulled);
  }
  if (simplified.size() < 2) {
    simplified = {points.front(), points.back()};
  }
  if (approximate && !goal_inside) {
    simplified.push_back(goal_point);
  }

  // ---- time parameterization: turn limits + velocity propagation ----
  const int n = static_cast<int>(simplified.size());
  std::vector<double> local_vmax(n, param.expected_speed);
  for (int i = 0; i + 2 < n; ++i) {
    // Segment i's end vertex sits at simplified[i + 1].
    local_vmax[i] = turnSpeedAtSegmentEnd(
        simplified, i, param.expected_speed, param.turn_deadzone,
        param.turn_saturation, param.turn_min_vel, param.turn_decay_power);
  }
  std::vector<double> seg_len(std::max(0, n - 1), 0.0);
  for (int i = 0; i + 1 < n; ++i) {
    seg_len[i] = (simplified[i + 1] - simplified[i]).norm();
  }
  // Backward pass: braking feasibility toward the (zero-speed) goal.
  for (int i = n - 2; i >= 0; --i) {
    const double v_next = local_vmax[std::min(i + 1, n - 1)];
    const double v_cap =
        std::sqrt(std::max(0.0, v_next * v_next +
                                       2.0 * param.max_accel * seg_len[i]));
    local_vmax[i] = std::min(local_vmax[i], v_cap);
  }
  // Forward pass: acceleration feasibility + remaining-distance cap.
  double v_current = 0.0;
  double remain = 0.0;
  for (int i = n - 2; i >= 0; --i) {
    remain += seg_len[i];
  }
  std::vector<double> segment_time(n - 1, 0.0);
  for (int i = 0; i + 1 < n; ++i) {
    remain -= seg_len[i];
    const double v_next =
        computeNextSpeed(v_current, seg_len[i], remain, param.max_accel,
                         local_vmax[std::min(i + 1, n - 1)]);
    segment_time[i] = computeSegmentTime(seg_len[i], v_current, v_next,
                                         param.max_accel);
    v_current = v_next;
  }

  // ---- assemble the output path (stamps = relative seconds) ----
  nav_msgs::Path plan;
  plan.header.frame_id = snapshot->frame;
  plan.poses.reserve(simplified.size());
  double t = 0.0;
  for (int i = 0; i < n; ++i) {
    auto pose = geometry_msgs::PoseStamped::fromIsometry(
        plan.header.frame_id, Eigen::Isometry3d::Identity());
    pose.pose.position = Eigen::Vector3d(simplified[i].x(), simplified[i].y(),
                                         0.0);
    pose.header.stamp = ayanami::Timestamp(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(t)));
    if (i + 1 < n) {
      t += segment_time[i];
    }
    plan.poses.push_back(std::move(pose));
  }
  const Eigen::Vector3d goal_world_position(
      goal_position.x(), goal_position.y(), goal_position.z());
  plan.poses.front().pose.orientation = start.pose.orientation;
  if (!approximate) {
    plan.poses.back().pose.orientation = goal.pose.orientation;
  }
  for (int i = 0; i + 1 < n; ++i) {
    const Eigen::Vector2d d = simplified[i + 1] - simplified[i];
    if (d.norm() < 1e-9) {
      continue;
    }
    plan.poses[i].pose.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(std::atan2(d.y(), d.x()), Eigen::Vector3d::UnitZ()));
  }
  (void)goal_world_position;

  plan_out = std::move(plan);
  auto out = ResultT<void>::success();
  out.message = approximate
                    ? "rosemap A* planner found an approximate path"
                    : "rosemap A* planner found a path";
  return out;
}

}  // namespace modules
}  // namespace ayanami
