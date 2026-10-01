# modules/rosemap_mpc_controller -- omni-base LMPC controller plugin
# (rosemap family). OsqpEigen is an environment install (NOT vendored); the
# module is skipped when the dependency is absent (same pattern as
# costmap_minsnap_smoother's OSQP backend).
find_package(OsqpEigen QUIET)
if(NOT OsqpEigen_FOUND)
  message(STATUS "ayanami: rosemap_mpc_controller disabled (OsqpEigen not found)")
  return()
endif()

ayanami_add_module(rosemap_mpc_controller
  rosemap_mpc_controller.cpp
  module.cpp)

target_link_libraries(rosemap_mpc_controller PRIVATE OsqpEigen::OsqpEigen)
