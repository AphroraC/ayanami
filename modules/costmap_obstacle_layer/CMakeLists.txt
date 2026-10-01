# modules/costmap_obstacle_layer -- rolling obstacle layer plugin.
ayanami_add_module(costmap_obstacle_layer module.cpp)
target_link_libraries(costmap_obstacle_layer PRIVATE costmap_2d_layers::costmap_2d_layers)
