# packages/costmap_server — 纯 C++ CostmapWrapper: LayeredCostmap 生命周期 + 线程安全
# 消费 costmap_2d 抽象 + costmap_2d_layers 具体层, 无 ROS 依赖。
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
