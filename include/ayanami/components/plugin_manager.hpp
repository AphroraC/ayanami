#pragma once

#include "ayanami/utility/config.hpp"
#include "ayanami/utility/load_module.hpp"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ayanami {
namespace components {

/**
 * @brief 插件装载描述 (spec §4.4)。
 *
 * name 为逻辑名 (getPlugin/hasPlugin 用), so_name 为 dlopen 目标 .so,
 * create_symbol 为该 .so 导出的工厂符号, 返回 PluginType*。
 */
struct PluginSpec {
  std::string name;
  std::string so_name;
  std::string create_symbol;
};

/**
 * @brief 类型化插件管理器 (对标 MBF AbstractPluginManager, spec §4.4)。
 *
 * loadPlugin 走 dlopen + dlsym 拿到工厂函数, 调用它获得 PluginType* 并包成
 * shared_ptr, 再回调 init_fn (调用方注入地图对象); init_fn 返回 false 时不入
 * 表。clearPlugins() 在 dlopen 句柄仍存活时释放全部插件 (spec §4.5 析构纪律)。
 *
 * 模板方法在头文件内联实现; impl/plugin_manager.cpp 为 4 个导航插件抽象做
 * 显式实例化, 把 dlopen 加载路径预编译进 libayanami (spec §4.4 全部使用点)。
 */
template <typename PluginType>
class PluginManager {
public:
  using InitFn = std::function<bool(const std::string&,
                                    std::shared_ptr<PluginType>)>;

  /// @brief 装载一个插件。dlopen + dlsym create_symbol → 调 init_fn → 入表。
  /// 失败 (so 打不开 / 符号缺失 / init_fn 拒绝) 返回 false, 不 abort。
  bool loadPlugin(const PluginSpec& spec, InitFn init_fn) {
    if (spec.name.empty() || spec.so_name.empty() ||
        spec.create_symbol.empty()) {
      return false;
    }
    auto create =
        load_module_from_so<PluginType* (*)()>(spec.so_name, spec.create_symbol);
    if (create == nullptr) {
      return false;
    }
    std::shared_ptr<PluginType> plugin(create());
    if (init_fn && !init_fn(spec.name, plugin)) {
      return false;
    }
    plugins_[spec.name] = std::move(plugin);
    so_names_[spec.name] = spec.so_name;
    return true;
  }

  /// @brief 从子配置 json 读 name/so_name/create_symbol 后 loadPlugin。
  /// @param default_name 配置缺 name 时的回退逻辑名。
  bool loadSingleFromConfig(const Config& cfg, const std::string& module_name,
                            const std::string& default_name, InitFn init_fn) {
    PluginSpec spec;
    spec.name = cfg.param<std::string>(module_name, "name", default_name);
    spec.so_name = cfg.param<std::string>(module_name, "so_name", "");
    spec.create_symbol =
        cfg.param<std::string>(module_name, "create_symbol", "");
    return loadPlugin(spec, std::move(init_fn));
  }

  /// @brief 取插件; 未装载返回 nullptr。
  std::shared_ptr<PluginType> getPlugin(const std::string& name) const {
    auto iter = plugins_.find(name);
    return iter == plugins_.end() ? nullptr : iter->second;
  }

  bool hasPlugin(const std::string& name) const {
    return plugins_.find(name) != plugins_.end();
  }

  /// @brief 已装载插件名列表 (map 序遍历, 按名有序)。
  std::vector<std::string> getLoadedNames() const {
    std::vector<std::string> names;
    names.reserve(plugins_.size());
    for (const auto& kv : plugins_) {
      names.push_back(kv.first);
    }
    return names;
  }

  /// @brief 释放全部插件对象 (dlopen 句柄不 dlclose, spec §4.1/§4.5)。
  void clearPlugins() {
    plugins_.clear();
    so_names_.clear();
  }

  /// @brief 返回某插件来源 .so; 未装载返回空串。
  std::string getSoName(const std::string& name) const {
    auto iter = so_names_.find(name);
    return iter == so_names_.end() ? std::string() : iter->second;
  }

private:
  std::map<std::string, std::shared_ptr<PluginType>> plugins_;
  std::map<std::string, std::string> so_names_;
};

} // namespace components
} // namespace ayanami
