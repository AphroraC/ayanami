#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <utility>

namespace ayanami {

/**
 * Bookkeeping for a loaded shared object.
 *
 * The current runtime intentionally keeps the module image resident. Keeping
 * this object separate from plugin instances establishes the ownership seam
 * needed for a future, fully quiescent unload without changing RTLD_GLOBAL.
 */
class LoadedModule {
 public:
  using Ptr = std::shared_ptr<LoadedModule>;

  LoadedModule(void* handle, std::string name)
      : handle_(handle), name_(std::move(name)) {}

  void* handle() const noexcept { return handle_; }
  const std::string& name() const noexcept { return name_; }

  // Unload is deliberately not implemented yet. A non-null handle means the
  // module image is resident for the process lifetime.
  bool unloadable() const noexcept { return false; }

 private:
  void* handle_{nullptr};
  std::string name_;
};

}  // namespace ayanami
