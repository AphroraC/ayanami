# modules/abstract_mpc_controller -- iterative linear MPC local controller
# (abstract family, map-free; rose_navigation LMPC port). OSQP/OsqpEigen are
# EXTERNAL dependencies (same policy as costmap_minsnap_smoother): without
# them the module is skipped -- the module list stays valid, PID remains the
# abstract-family controller.
find_package(OsqpEigen QUIET)
if(NOT OsqpEigen_FOUND)
  message(STATUS "ayanami: abstract_mpc_controller skipped (OsqpEigen not found)")
  return()
endif()
ayanami_add_module(abstract_mpc_controller
  abstract_mpc_controller.cpp
  module.cpp)
target_link_libraries(abstract_mpc_controller PRIVATE OsqpEigen::OsqpEigen)
