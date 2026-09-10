#pragma once

#include "ayanami/messages/failure.hpp"
#include "ayanami/messages/nav_msgs.hpp"
#include "ayanami/types.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace ayanami {
namespace abstract {

/**
 * @brief Path smoother module abstraction (MBF-style layering).
 *
 * Zero map types: the interface contains no map parameters. In-place
 * semantics: smooth the given path directly; on success the path has been
 * modified and the result carries only status and diagnostics.
 */
class AbstractSmoother : public virtual ExtensionModule {
 public:
  using Ptr = std::shared_ptr<AbstractSmoother>;

  /// @brief Interface version of this family, as compiled into the module.
  /// Bump on ANY change to this header's virtual surface: the loader compares
  /// the value a .so reports against the one it was built with and refuses a
  /// mismatch (two modules agreeing on class names but disagreeing on layout is
  /// exactly the failure RTTI cannot catch).
  static constexpr std::uint32_t kApiVersion = 1;
  std::uint32_t apiVersion() const override { return kApiVersion; }


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
