# modules/costmap_static_layer -- static map layer plugin (consumes the
# costmap_2d_layers layer implementations).
ayanami_add_module(costmap_static_layer module.cpp)
target_link_libraries(costmap_static_layer PRIVATE costmap_2d_layers::costmap_2d_layers)
