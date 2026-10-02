# packages/costmap -- merged pure C++ costmap sub-library (formerly the
# costmap_2d / costmap_layers / costmap_server trio). Namespace: costmap_2d
# for everything, headers under <costmap/...>. No ayanami / ROS dependency:
# logging goes through spdlog::get("costmap") with the default-logger
# fallback, configuration is applied by the host through setters.
add_library(costmap STATIC
  src/static_layer.cpp
  src/inflation_layer.cpp
  src/simple_obstacle_layer.cpp)
add_library(costmap::costmap ALIAS costmap)

target_include_directories(costmap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(costmap PUBLIC Eigen3::Eigen PRIVATE spdlog::spdlog)

set_target_properties(costmap PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS costmap EXPORT ayanamiTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp")
