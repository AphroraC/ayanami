# modules/costmap_neupan_controller -- NeuPAN receding-horizon controller
# (costmap family). The solver core is the vendored packages/neupan library
# (GPL-3.0); OSQP/OsqpEigen/yaml-cpp come in transitively through it.
ayanami_add_module(costmap_neupan_controller
  costmap_neupan_controller.cpp
  module.cpp)

target_link_libraries(costmap_neupan_controller PRIVATE neupan)
