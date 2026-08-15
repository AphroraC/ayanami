# packages/costmap_2d — 纯 C++ header-only 子库, namespace costmap_2d
# 第三方可直接 include <costmap_2d/costmap_2d.hpp>
add_library(costmap_2d INTERFACE)
add_library(costmap_2d::costmap_2d ALIAS costmap_2d)

target_include_directories(costmap_2d INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 轻型 Eigen 依赖（供后续 costmap_layers 使用）
target_link_libraries(costmap_2d INTERFACE Eigen3::Eigen)

install(TARGETS costmap_2d EXPORT ayanamiTargets)
