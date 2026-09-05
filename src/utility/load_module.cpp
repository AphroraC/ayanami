#include <dlfcn.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include <ayanami/utility/load_module.hpp>

namespace ayanami {
namespace {

// 模块登记表: so_name → 句柄 + 引用计数。openSo 重复加载去重 (同一句柄),
// 并为未来静默卸载保留账本 (卸载逻辑未实现; 前置条件见 load_module.hpp)。
std::mutex registry_mutex;
std::map<std::string, void*>& moduleRegistry() {
  static std::map<std::string, void*> registry;
  return registry;
}
std::map<std::string, std::size_t>& moduleRefCounts() {
  static std::map<std::string, std::size_t> ref_counts;
  return ref_counts;
}

}  // namespace

void* openSo(const std::string& so_name) {
  std::lock_guard<std::mutex> lock(registry_mutex);
  auto& registry = moduleRegistry();
  auto iter = registry.find(so_name);
  if (iter != registry.end()) {
    // 已登记: 去重, 引用计数 +1, 返回同一句柄。
    ++moduleRefCounts()[so_name];
    return iter->second;
  }
  // RTLD_NODELETE: 镜像恒驻 (即使未来 dlclose 也不卸载代码段), 为静默卸载
  // 准备安全垫; 当前路径从不 dlclose (spec §4.1)。
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY | RTLD_GLOBAL | RTLD_NODELETE);
  if (handle == nullptr) {
    spdlog::error("failed to load shared object '{}': {}", so_name, dlerror());
    return nullptr;
  }
  // 句柄从不 dlclose：插件生命周期等于进程生命周期 (spec §4.1)。
  registry.emplace(so_name, handle);
  moduleRefCounts()[so_name] = 1;
  return handle;
}

std::size_t moduleHandleRefs(const std::string& so_name) {
  std::lock_guard<std::mutex> lock(registry_mutex);
  auto& ref_counts = moduleRefCounts();
  auto iter = ref_counts.find(so_name);
  return iter == ref_counts.end() ? 0 : iter->second;
}

std::vector<std::string> loadedModuleNames() {
  std::lock_guard<std::mutex> lock(registry_mutex);
  std::vector<std::string> names;
  names.reserve(moduleRegistry().size());
  for (const auto& entry : moduleRegistry()) {
    names.push_back(entry.first);
  }
  return names;
}

void* loadSymbol(void* h, const std::string& sym) {
  if (h == nullptr) {
    return nullptr;
  }
  dlerror();  // 清除上一次错误，区分 dlsym 找到空符号与失败
  void* symbol = dlsym(h, sym.c_str());
  if (symbol == nullptr) {
    const char* err = dlerror();
    if (err != nullptr) {
      spdlog::error("failed to load symbol '{}': {}", sym, err);
    }
    return nullptr;
  }
  return symbol;
}

} // namespace ayanami
