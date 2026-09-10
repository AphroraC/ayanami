#include <dlfcn.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include <ayanami/utility/load_module.hpp>

namespace ayanami {
namespace {

// Module registry: so_name -> handle + reference count. Repeated openSo
// loads are deduplicated (same handle),
// and a ledger is kept for future silent unloading (unload logic not yet
// implemented; see load_module.hpp for preconditions).
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
    // Already registered: deduplicate, reference count +1, return the same handle.
    ++moduleRefCounts()[so_name];
    return iter->second;
  }
  // RTLD_NODELETE: keep the image resident (the code segment is not
  // unloaded even by a future dlclose), a safety cushion for silent
  // unloading; the current path never dlcloses.
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY | RTLD_GLOBAL | RTLD_NODELETE);
  if (handle == nullptr) {
    spdlog::error("failed to load shared object '{}': {}", so_name, dlerror());
    return nullptr;
  }
  // Handles are never dlclosed: the module lifetime equals the process lifetime.
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
  dlerror();  // clear the previous error, distinguishing a null symbol found by dlsym from failure
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
