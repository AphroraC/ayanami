#pragma once

#include <memory>
#include <string>

#include "ayanami/costmap/components/costmap_map_context.hpp"
#include "ayanami/map/map_family_factory.hpp"

namespace ayanami {
namespace costmap {

/**
 * @brief costmap 地图族工厂: CostmapMapContext 的唯一创建点 (spec 地图族
 * 抽象: "核心服务器不知道 CostmapContext, 只知道地图族工厂")。
 *
 * - 宿主 (CostmapNavigation) 构造本工厂并传入
 *   CostmapNavigationServer::configureCoreMaps; 族 server 经 createContext()
 *   取得空上下文后装配 wrapper/frames/footprint/commandPort。
 * - 未来地图族 (mesh/voxel, 见 map/map_family_factory.hpp 登记) 以平行的
 *   {Family}MapFamilyFactory 接入, 族间互不感知。
 * - 工厂返回 AbstractMapContext::Ptr (抽象面); 族 server 持有
 *   CostmapMapContext::Ptr (族面), 二者经 dynamic_pointer_cast 收敛。
 */
class CostmapMapFamilyFactory final : public map::MapFamilyFactory {
 public:
  using Ptr = std::shared_ptr<CostmapMapFamilyFactory>;

  std::string familyName() const override { return "costmap"; }

  /// @brief 创建空的 costmap 族上下文 (内容由 configureCoreMaps 装配)。
  map::AbstractMapContext::Ptr createContext() override {
    return std::make_shared<CostmapMapContext>();
  }
};

}  // namespace costmap
}  // namespace ayanami
