#pragma once
#include <cstddef>
#include <string>
#include <vector>

namespace ayanami {

// dlfcn.h 只在 src/utility/load_module.cpp 内使用；hpp 只声明不 include dlfcn.h。

/**
 * @brief dlopen a shared object with RTLD_LAZY | RTLD_GLOBAL | RTLD_NODELETE.
 *
 * 同一 so_name 重复 openSo 经模块登记表去重并递增引用计数, 返回同一句柄。
 *
 * @return opaque handle, or nullptr on failure (spdlog::error logged).
 * @note Handles are NEVER dlclose'd — the plugin lifetime equals the process
 *       lifetime (spec §4.1; teardown discipline in §4.5). RTLD_NODELETE 使
 *       镜像恒驻, 未来即使接入 dlclose 也安全。
 *
 * @note 卸载准备 (卸载逻辑本身尚未实现, 闸门 = LoadedModule::unloadable() /
 *       ExtensionModule::can_unload())。未来静默卸载的前置条件清单:
 *       1) 全部 action 取消并 join (NavigationServerBase::stop);
 *       2) 全部插件实例析构 (clearPlugins);
 *       3) 该模块的全部 ExtensionModule shared_ptr 释放
 *          (moduleHandleRefs 归零);
 *       4) unloadable() 闸门放行后才允许 dlclose。
 */
void* openSo(const std::string& so_name);

/// @brief 模块登记表观测: so_name 当前句柄引用计数 (未加载返回 0)。
std::size_t moduleHandleRefs(const std::string& so_name);

/// @brief 模块登记表观测: 已登记模块名列表 (诊断/测试用)。
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
