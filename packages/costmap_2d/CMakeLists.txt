# packages/costmap_2d -- pure C++ header-only sub-library, namespace costmap_2d
# Third parties can directly include <costmap_2d/costmap_2d.hpp>
add_library(costmap_2d INTERFACE)
add_library(costmap_2d::costmap_2d ALIAS costmap_2d)

target_include_directories(costmap_2d INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Lightweight Eigen dependency (for later use by costmap_layers)
target_link_libraries(costmap_2d INTERFACE Eigen3::Eigen)

install(TARGETS costmap_2d EXPORT ayanamiTargets)
