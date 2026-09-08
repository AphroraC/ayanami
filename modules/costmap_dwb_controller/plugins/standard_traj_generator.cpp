// standard_traj_generator.cpp -- ported from dwb_plugins/src/
// standard_traj_generator.cpp (rclcpp parameters become Config; Duration becomes double).
#include "standard_traj_generator.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "xy_theta_iterator.hpp"

namespace dwb_plugins {

void StandardTrajectoryGenerator::initialize(const ayanami::Config& cfg,
                                             const std::string& plugin_name) {
  this->plugin_name = plugin_name;
  kinematics_handler = std::make_shared<KinematicsHandler>();
  kinematics_handler->initialize(cfg, plugin_name);
  initializeIterator(cfg);

  sim_time = cfg.param<double>(plugin_name, "sim_time", 1.7);
  discretize_by_time =
      cfg.param<bool>(plugin_name, "discretize_by_time", false);
  time_granularity = cfg.param<double>(plugin_name, "time_granularity", 0.5);
  linear_granularity =
      cfg.param<double>(plugin_name, "linear_granularity", 0.5);
  angular_granularity =
      cfg.param<double>(plugin_name, "angular_granularity", 0.025);
  include_last_point =
      cfg.param<bool>(plugin_name, "include_last_point", true);
  limit_vel_cmd_in_traj =
      cfg.param<bool>(plugin_name, "limit_vel_cmd_in_traj", false);
}

void StandardTrajectoryGenerator::initializeIterator(const ayanami::Config& cfg) {
  velocity_iterator = std::make_shared<XYThetaIterator>();
  velocity_iterator->initialize(cfg, kinematics_handler, plugin_name);
}

void StandardTrajectoryGenerator::startNewIteration(
    const nav_2d_msgs::Twist2D& current_velocity) {
  velocity_iterator->startNewIteration(current_velocity, sim_time);
}

bool StandardTrajectoryGenerator::hasMoreTwists() {
  return velocity_iterator->hasMoreTwists();
}

nav_2d_msgs::Twist2D StandardTrajectoryGenerator::nextTwist() {
  return velocity_iterator->nextTwist();
}

std::vector<double> StandardTrajectoryGenerator::getTimeSteps(
    const nav_2d_msgs::Twist2D& cmd_vel) {
  std::vector<double> steps;
  if (discretize_by_time) {
    steps.resize(ceil(sim_time / time_granularity));
  } else {  // discretize by distance
    double vmag = hypot(cmd_vel.x, cmd_vel.y);

    // the distance the robot would travel in sim_time if it did not change
    // velocity
    double projected_linear_distance = vmag * sim_time;

    // the angle the robot would rotate in sim_time
    double projected_angular_distance = fabs(cmd_vel.theta) * sim_time;

    // Pick the maximum of the two
    int num_steps = ceil(
        std::max(projected_linear_distance / linear_granularity,
                 projected_angular_distance / angular_granularity));
    steps.resize(num_steps);
  }
  if (steps.size() == 0) {
    steps.resize(1);
  }
  std::fill(steps.begin(), steps.end(), sim_time / steps.size());
  return steps;
}

dwb_msgs::Trajectory2D StandardTrajectoryGenerator::generateTrajectory(
    const geometry_msgs::Pose2D& start_pose,
    const nav_2d_msgs::Twist2D& start_vel,
    const nav_2d_msgs::Twist2D& cmd_vel) {
  dwb_msgs::Trajectory2D traj;
  traj.velocity = cmd_vel;
  // simulate the trajectory
  geometry_msgs::Pose2D pose = start_pose;
  nav_2d_msgs::Twist2D vel = start_vel;
  double running_time = 0.0;
  std::vector<double> steps = getTimeSteps(cmd_vel);
  traj.poses.push_back(start_pose);
  bool first_vel = false;
  for (double dt : steps) {
    // calculate velocities
    vel = computeNewVelocity(cmd_vel, vel, dt);
    if (!first_vel && limit_vel_cmd_in_traj) {
      traj.velocity = vel;
      first_vel = true;
    }

    // update the position of the robot using the velocities passed in
    pose = computeNewPosition(pose, vel, dt);

    traj.poses.push_back(pose);
    traj.time_offsets.push_back(running_time);
    running_time += dt;
  }  // end for simulation steps

  if (include_last_point) {
    traj.poses.push_back(pose);
    traj.time_offsets.push_back(running_time);
  }

  return traj;
}

/**
 * change vel using acceleration limits to converge towards sample_target-vel
 */
nav_2d_msgs::Twist2D StandardTrajectoryGenerator::computeNewVelocity(
    const nav_2d_msgs::Twist2D& cmd_vel,
    const nav_2d_msgs::Twist2D& start_vel, const double dt) {
  KinematicParameters kinematics = kinematics_handler->getKinematics();
  nav_2d_msgs::Twist2D new_vel;
  new_vel.x = projectVelocity(start_vel.x, kinematics.getAccX(),
                              kinematics.getDecelX(), dt, cmd_vel.x);
  new_vel.y = projectVelocity(start_vel.y, kinematics.getAccY(),
                              kinematics.getDecelY(), dt, cmd_vel.y);
  new_vel.theta = projectVelocity(start_vel.theta, kinematics.getAccTheta(),
                                  kinematics.getDecelTheta(), dt, cmd_vel.theta);
  return new_vel;
}

geometry_msgs::Pose2D StandardTrajectoryGenerator::computeNewPosition(
    const geometry_msgs::Pose2D start_pose, const nav_2d_msgs::Twist2D& vel,
    const double dt) {
  geometry_msgs::Pose2D new_pose;
  new_pose.x = start_pose.x +
               (vel.x * cos(start_pose.theta) +
                vel.y * cos(M_PI_2 + start_pose.theta)) *
                   dt;
  new_pose.y = start_pose.y +
               (vel.x * sin(start_pose.theta) +
                vel.y * sin(M_PI_2 + start_pose.theta)) *
                   dt;
  new_pose.theta = start_pose.theta + vel.theta * dt;
  return new_pose;
}

}  // namespace dwb_plugins
