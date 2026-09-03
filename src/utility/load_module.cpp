#include <dlfcn.h>
#include <string>

#include <spdlog/spdlog.h>

#include <ayanami/utility/load_module.hpp>

namespace ayanami {

void* openSo(const std::string& so_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY | RTLD_GLOBAL);
  if (handle == nullptr) {
    spdlog::error("failed to load shared object '{}': {}", so_name, dlerror());
    return nullptr;
  }
  // 句柄从不 dlclose：插件生命周期等于进程生命周期 (spec §4.1)。
  return handle;
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
