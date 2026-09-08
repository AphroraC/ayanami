# packages/costmap_server -- pure C++ CostmapWrapper: LayeredCostmap lifecycle + thread safety
# Consumes the costmap_2d abstractions + costmap_2d_layers concrete layers; no ROS dependency.
add_library(costmap_server INTERFACE)
add_library(costmap_server::costmap_server ALIAS costmap_server)

target_include_directories(costmap_server INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(costmap_server INTERFACE
  costmap_2d::costmap_2d
  costmap_2d_layers::costmap_2d_layers
  Eigen3::Eigen)

install(TARGETS costmap_server EXPORT ayanamiTargets)
install(DIRECTORY include/ DESTINATION include
  FILES_MATCHING PATTERN "*.hpp")
