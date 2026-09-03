// obstacle_footprint.cpp — 迁移自 dwb_critics/src/obstacle_footprint.cpp
// (footprint 来源: costmap_ros->getRobotFootprint() → 基类注入的 footprint)。
#include "obstacle_footprint.hpp"

#include <algorithm>
#include <vector>

#include <spdlog/spdlog.h>

#include <costmap_2d/cost_values.hpp>

#include "../exceptions.hpp"
#include "line_iterator.hpp"

namespace dwb_critics {
Footprint getOrientedFootprint(const geometry_msgs::Pose2D& pose,
                               const Footprint& footprint_spec) {
  std::vector<geometry_msgs::Point> oriented_footprint;
  oriented_footprint.resize(footprint_spec.size());
  double cos_th = cos(pose.theta);
  double sin_th = sin(pose.theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    geometry_msgs::Point& new_pt = oriented_footprint[i];
    new_pt.x = pose.x + footprint_spec[i].x * cos_th -
               footprint_spec[i].y * sin_th;
    new_pt.y = pose.y + footprint_spec[i].x * sin_th +
               footprint_spec[i].y * cos_th;
  }
  return oriented_footprint;
}

bool ObstacleFootprintCritic::prepare(const geometry_msgs::Pose2D&,
                                      const nav_2d_msgs::Twist2D&,
                                      const geometry_msgs::Pose2D&,
                                      const nav_2d_msgs::Path2D&) {
  footprint_spec = footprint ? *footprint : Footprint();
  if (footprint_spec.size() == 0) {
    spdlog::error("Footprint spec is empty, maybe missing call to "
                  "setFootprint?");
    return false;
  }
  return true;
}

double ObstacleFootprintCritic::scorePose(const geometry_msgs::Pose2D& pose) {
  unsigned int cell_x, cell_y;
  if (!costmap->worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    throw dwb_core::IllegalTrajectoryException(name,
                                               "Trajectory Goes Off Grid.");
  }
  return scorePose(pose, getOrientedFootprint(pose, footprint_spec));
}

double ObstacleFootprintCritic::scorePose(const geometry_msgs::Pose2D&,
                                          const Footprint& footprint) {
  // now we really have to lay down the footprint in the costmap grid
  unsigned int x0, x1, y0, y1;
  double line_cost = 0.0;
  double footprint_cost = 0.0;

  // we need to rasterize each line in the footprint
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // get the cell coord of the first point
    if (!costmap->worldToMap(footprint[i].x, footprint[i].y, x0, y0)) {
      throw dwb_core::IllegalTrajectoryException(name,
                                                 "Footprint Goes Off Grid.");
    }

    // get the cell coord of the second point
    if (!costmap->worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      throw dwb_core::IllegalTrajectoryException(name,
                                                 "Footprint Goes Off Grid.");
    }

    line_cost = lineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);
  }

  // we also need to connect the first point in the footprint to the last point
  // get the cell coord of the last point
  if (!costmap->worldToMap(footprint.back().x, footprint.back().y, x0, y0)) {
    throw dwb_core::IllegalTrajectoryException(name,
                                               "Footprint Goes Off Grid.");
  }

  // get the cell coord of the first point
  if (!costmap->worldToMap(footprint.front().x, footprint.front().y, x1, y1)) {
    throw dwb_core::IllegalTrajectoryException(name,
                                               "Footprint Goes Off Grid.");
  }

  line_cost = lineCost(x0, x1, y0, y1);
  footprint_cost = std::max(line_cost, footprint_cost);

  // if all line costs are legal... then we can return that the footprint is
  // legal
  return footprint_cost;
}

double ObstacleFootprintCritic::lineCost(int x0, int x1, int y0, int y1) {
  double line_cost = 0.0;
  double point_cost = -1.0;

  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = pointCost(line.getX(), line.getY());  // Score the current point

    if (line_cost < point_cost) {
      line_cost = point_cost;
    }
  }

  return line_cost;
}

double ObstacleFootprintCritic::pointCost(int x, int y) {
  unsigned char cost = costmap->getCost(x, y);
  // if the cell is in an obstacle the path is invalid or unknown
  if (cost == costmap_2d::LETHAL_OBSTACLE) {
    throw dwb_core::IllegalTrajectoryException(name,
                                               "Trajectory Hits Obstacle.");
  } else if (cost == costmap_2d::NO_INFORMATION) {
    throw dwb_core::IllegalTrajectoryException(name,
                                               "Trajectory Hits Unknown Region.");
  }

  return cost;
}

}  // namespace dwb_critics
