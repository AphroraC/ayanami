#pragma once
#include <cstddef>
#include <string>
#include <vector>

namespace ayanami {

// dlfcn.h is used only inside src/utility/load_module.cpp; the hpp only declares and does not include dlfcn.h.

/**
 * @brief dlopen a shared object with RTLD_LAZY | RTLD_GLOBAL | RTLD_NODELETE.
 *
 * Repeated openSo of the same so_name is deduplicated via the module registry with an incremented reference count, returning the same handle.
 *
 * @return opaque handle, or nullptr on failure (spdlog::error logged).
 * @note Handles are NEVER dlclose'd -- the module lifetime equals the process
 * lifetime (; teardown discipline in §4.5). RTLD_NODELETE keeps
 *       the image resident, so even a future dlclose hookup stays safe.
 *
 * @note Unload preparation (the unload logic itself is not implemented yet;
 *       gates = LoadedModule::unloadable() /
 *       ExtensionModule::canUnload()). Preconditions for future silent
 *       unloading:
 *       1) all actions canceled and joined (NavigationServerBase::stop);
 *       2) all module instances destructed (clearModules);
 *       3) all ExtensionModule shared_ptrs of the module released
 *          (moduleHandleRefs down to zero);
 *       4) dlclose only allowed after the unloadable() gate opens.
 */
void* openSo(const std::string& so_name);

/// @brief Module registry observation: the current handle refcount for so_name (0 when not loaded).
std::size_t moduleHandleRefs(const std::string& so_name);

/// @brief Module registry observation: list of registered module names (for diagnostics/tests).
std::vector<std::string> loadedModuleNames();

/**
 * @brief dlsym a symbol from an opened handle.
 *
 * @return symbol address (as void*), or nullptr on failure (spdlog::error).
 */
void* loadSymbol(void* h, const std::string& sym);

/**
 * @brief Load a factory function pointer from a shared object.
 *
 * T must be a function pointer type (e.g. `int(*)()`, `ExtensionModule*(*)()`).
 * Returns the dlsym'd symbol reinterpret_cast to T, or nullptr when the .so
 * cannot be opened or the symbol is missing. The caller invokes the returned
 * factory to obtain an object instance.
 */
template <typename T>
T loadModuleFromSo(const std::string& so_name, const std::string& sym) {
  void* handle = openSo(so_name);
  if (handle == nullptr) {
    return nullptr;
  }
  void* symbol = loadSymbol(handle, sym);
  if (symbol == nullptr) {
    return nullptr;
  }
  return reinterpret_cast<T>(symbol);
}

} // namespace ayanami
