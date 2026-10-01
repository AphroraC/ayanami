# modules/costmap_minsnap_smoother -- minimum-snap smoother plugin.
ayanami_add_module(costmap_minsnap_smoother
  costmap_minsnap_smoother.cpp
  minsnap_solver.cpp
  square_corridor.cpp
  module.cpp)

# OSQP corridor backend: OSQP/OsqpEigen are EXTERNAL dependencies resolved
# by find_package against the environment install (NOT vendored); the
# closed-form backend keeps the module buildable without them.
find_package(OsqpEigen QUIET)
if(OsqpEigen_FOUND)
  target_link_libraries(costmap_minsnap_smoother PRIVATE OsqpEigen::OsqpEigen)
  target_compile_definitions(costmap_minsnap_smoother
    PRIVATE AYANAMI_MINSNAP_HAS_OSQP=1)
  message(STATUS "ayanami: costmap_minsnap_smoother OSQP corridor backend enabled")
else()
  message(STATUS "ayanami: costmap_minsnap_smoother closed-form backend only (OsqpEigen not found)")
endif()
