// registration.hpp — critic / trajectory generator 静态注册表 (pluginlib 的
// ayanami 替代: 全部子插件编译进模块 .so, 按简化类名查找)。
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "trajectory_critic.hpp"
#include "trajectory_generator.hpp"

namespace ayanami {
namespace modules {

/// @brief critic 工厂注册表 (键: 简化类名, 如 "BaseObstacleCritic")。
std::map<std::string, std::function<dwb_core::TrajectoryCritic::Ptr()>>&
criticRegistry();

/// @brief trajectory generator 工厂注册表 (键: "StandardTrajectoryGenerator"
/// / "LimitedAccelGenerator")。
std::map<std::string, std::function<dwb_core::TrajectoryGenerator::Ptr()>>&
trajectoryGeneratorRegistry();

/// @brief 注册全部内置 critic (11 个 + 基类不可实例化)。
void registerAllCritics();

/// @brief 注册全部内置 trajectory generator (2 个)。
void registerAllTrajectoryGenerators();

}  // namespace modules
}  // namespace ayanami
