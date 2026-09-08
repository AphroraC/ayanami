#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief Path smoother plugin abstraction (MBF-style layering).
 *
 * Zero map types: the interface contains no map parameters. In-place
 * semantics: smooth the given path directly; on success the path has been
 * modified and the result carries only status and diagnostics.
 */
class AbstractSmoother : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractSmoother>;

  virtual ~AbstractSmoother() = default;

  /// @brief Smooth the given path in place (pure virtual).
  /// @param[in,out] path The path to smooth; updated in place on success.
  /// @return ResultT<void>: path is valid when ok=true; failures carry
  ///         ErrorCode.
  virtual ResultT<void> smoothPath(ayanami::nav_msgs::Path& path,
                                   double max_duration) = 0;

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /// @brief Cancel in-progress smoothing (cooperative interrupt; unsupported by default).
  virtual bool cancel() { return false; }
};

} // namespace abstract
} // namespace ayanami
