#pragma once
#include <string>

namespace ayanami {

// dlfcn.h 只在 src/utility/load_module.cpp 内使用；hpp 只声明不 include dlfcn.h。

/**
 * @brief dlopen a shared object with RTLD_LAZY | RTLD_GLOBAL.
 *
 * @return opaque handle, or nullptr on failure (spdlog::error logged).
 * @note Handles are NEVER dlclose'd — the plugin lifetime equals the process
 *       lifetime (spec §4.1; teardown discipline in §4.5).
 */
void* openSo(const std::string& so_name);

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
