#pragma once

#include "ayanami/utility/config.hpp"
#include "ayanami/utility/extension_module.hpp"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ayanami {
namespace components {

/**
 * @brief 插件装载描述 。
 *
 * name 为逻辑名 (getPlugin/hasPlugin 用), so_name 为 dlopen 目标 .so。
 * 工厂符号统一为 `create_extension_module` (ExtensionModule 派生类型);
 * 不再有任意 create_symbol 字段。
 */
struct PluginSpec {
  std::string name;
  std::string so_name;
};

/**
 * @brief 类型化插件管理器 (对标 MBF AbstractPluginManager)。
 *
 * loadPlugin 走 ExtensionModule::load_module (dlopen + dlsym
 * `create_extension_module` 公共工厂) 拿到 ExtensionModule::Ptr, 经类型化
 * std::dynamic_pointer_cast<PluginType> 获得 PluginType::Ptr 并保留所有权
 * (与 ExtensionModule::Ptr 共享引用计数), 再回调 init_fn (调用方注入地图对象);
 * init_fn 返回 false 时不入表。clearPlugins() 在 dlopen 句柄仍存活时释放全部
 * 插件 (析构纪律)。
 *
 * 模板方法在头文件内联实现; impl/plugin_manager.cpp 为 4 个导航插件抽象做
 * 显式实例化, 把 dlopen 加载路径预编译进 libayanami (全部使用点)。
 */
template <typename PluginType>
class PluginManager {
public:
  using InitFn = std::function<bool(const std::string&,
                                    std::shared_ptr<PluginType>)>;

  /// @brief 装载一个插件。公共工厂 create_extension_module → 类型化 cast →
  /// 调 init_fn → 入表。失败 (so 打不开 / 类型不符合 / init_fn 拒绝) 返回 false。
  bool loadPlugin(const PluginSpec& spec, InitFn init_fn) {
    if (spec.name.empty() || spec.so_name.empty()) {
      return false;
    }
    auto module = ayanami::ExtensionModule::load_module(spec.so_name);
    if (!module) {
      return false;
    }
    // 类型化转换: 只有真正派生于 PluginType 负载才接受; 转换后与
    // ExtensionModule::Ptr 共享同一引用计数 (所有权保留)。
    std::shared_ptr<PluginType> plugin =
        std::dynamic_pointer_cast<PluginType>(std::move(module));
    if (!plugin) {
      return false;
    }
    if (init_fn && !init_fn(spec.name, plugin)) {
      return false;
    }
    plugins[spec.name] = std::move(plugin);
    so_names[spec.name] = spec.so_name;
    return true;
  }

  /// @brief 从子配置 json 读 name/so_name 后 loadPlugin。
  /// @note 不做配置注入: 插件自身的参数由插件在构造/initialize 时经
  /// GlobalConfig::getConfigPath 显式读取 (glim 式, 无 thread_local)。
  /// @param default_name 配置缺 name 时的回退逻辑名。
  bool loadSingleFromConfig(const Config& cfg, const std::string& module_name,
                             const std::string& default_name, InitFn init_fn) {
    PluginSpec spec;
    spec.name = cfg.param<std::string>(module_name, "name", default_name);
    spec.so_name = cfg.param<std::string>(module_name, "so_name", "");
    return loadPlugin(spec, std::move(init_fn));
  }

  /// @brief 取插件; 未装载返回 nullptr。
  std::shared_ptr<PluginType> getPlugin(const std::string& name) const {
    auto iter = plugins.find(name);
    return iter == plugins.end() ? nullptr : iter->second;
  }

  bool hasPlugin(const std::string& name) const {
    return plugins.find(name) != plugins.end();
  }

  /// @brief 已装载插件名列表 (map 序遍历, 按名有序)。
  std::vector<std::string> getLoadedNames() const {
    std::vector<std::string> names;
    names.reserve(plugins.size());
    for (const auto& kv : plugins) {
      names.push_back(kv.first);
    }
    return names;
  }

  /// @brief 释放全部插件对象 (dlopen 句柄不 dlclose, /§4.5)。
  void clearPlugins() {
    plugins.clear();
    so_names.clear();
  }

  /// @brief 返回某插件来源 .so; 未装载返回空串。
  std::string getSoName(const std::string& name) const {
    auto iter = so_names.find(name);
    return iter == so_names.end() ? std::string() : iter->second;
  }

private:
  std::map<std::string, std::shared_ptr<PluginType>> plugins;
  std::map<std::string, std::string> so_names;
};

} // namespace components
} // namespace ayanami
