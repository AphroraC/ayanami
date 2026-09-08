# packages/costmap_layers -- pure C++ concrete costmap layer subclasses, namespace costmap_2d_layers
# Three layers: StaticLayer / InflationLayer / SimpleObstacleLayer
# Consumes the costmap_2d abstractions (Costmap2D/Layer/LayeredCostmap); no ROS dependency.
add_library(costmap_2d_layers STATIC
  src/static_layer.cpp
  src/inflation_layer.cpp
  src/simple_obstacle_layer.cpp)
add_library(costmap_2d_layers::costmap_2d_layers ALIAS costmap_2d_layers)

target_include_directories(costmap_2d_layers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(costmap_2d_layers PUBLIC costmap_2d::costmap_2d
  PRIVATE spdlog::spdlog)

set_target_properties(costmap_2d_layers PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS costmap_2d_layers EXPORT ayanamiTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include
  FILES_MATCHING PATTERN "*.hpp")
