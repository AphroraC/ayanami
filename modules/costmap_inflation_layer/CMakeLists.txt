# modules/costmap_inflation_layer -- inflation layer plugin.
ayanami_add_module(costmap_inflation_layer module.cpp)
target_link_libraries(costmap_inflation_layer PRIVATE costmap_2d_layers::costmap_2d_layers)
