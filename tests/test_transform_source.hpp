#pragma once

// Test-only stub for the RobotStateController TF bridge: a minimal static
// TF tree with chain composition, standing in for the tf2-backed source
// that ayanami_ros installs in production. Stamp and timeout arguments are
// ignored (tests feed latest-time semantics).

#include <functional>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <ayanami/abstract/components/robot_state_controller.hpp>
#include <ayanami/messages/timestamp.hpp>

namespace ayanami {
namespace testing {

class TestTransformSource {
 public:
  /// @brief Register one edge: the transform of `child` in `parent`.
  /// Signature-compatible with the old RobotStateController::setTransform
  /// (the stamp/is_static arguments are accepted and ignored: tests feed
  /// latest-time semantics).
  void set(const std::string& parent, const std::string& child,
           const Timestamp&, const Eigen::Isometry3d& transform,
           bool = false) {
    set(parent, child, transform);
  }
  void set(const std::string& parent, const std::string& child,
           const Eigen::Isometry3d& transform) {
    edges_[{parent, child}] = transform;
    edges_[{child, parent}] = transform.inverse();
  }

  /// @brief Install into the RobotStateController singleton.
  void install() {
    components::RobotStateController::instance().setTransformSource(
        [this](const std::string& target, const std::string& source,
               const Timestamp&, Eigen::Isometry3d& transform, double) {
          return lookup(target, source, transform);
        });
  }

  /// @brief Compose target<-source over registered edges (BFS outward from
  /// the target frame; the answer is the transform of `source` expressed
  /// in `target`).
  bool lookup(const std::string& target, const std::string& source,
              Eigen::Isometry3d& transform) const {
    if (target == source) {
      transform = Eigen::Isometry3d::Identity();
      return true;
    }
    // reached[f] = the transform of f in target (target<-f).
    std::map<std::string, Eigen::Isometry3d> reached{{target, Eigen::Isometry3d::Identity()}};
    std::queue<std::string> frontier{{target}};
    while (!frontier.empty()) {
      const std::string current = frontier.front();
      frontier.pop();
      for (const auto& [endpoints, edge] : edges_) {
        if (endpoints.first != current) {
          continue;
        }
        const std::string& next = endpoints.second;
        if (reached.count(next) != 0) {
          continue;
        }
        // target<-next = target<-current * current<-next
        reached[next] = reached[current] * edge;
        if (next == source) {
          transform = reached[next];
          return true;
        }
        frontier.push(next);
      }
    }
    return false;
  }

 private:
  std::map<std::pair<std::string, std::string>, Eigen::Isometry3d> edges_;
};

/// @brief The per-binary shared stub source (installed into the singleton
/// on first use).
inline TestTransformSource& testTransforms() {
  static TestTransformSource source;
  static const bool installed = [] {
    source.install();
    return true;
  }();
  (void)installed;
  return source;
}

}  // namespace testing
}  // namespace ayanami
