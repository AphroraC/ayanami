# tests/CMakeLists.txt -- all test declarations.
# Boilerplate lives in cmake/ayanami-tests.cmake (ayanami_add_test /
# ayanami_add_test_plugin); per-module switches work because every test that
# needs a module target gates on it and skips itself when the module is off.
#
# Test and target names match the legacy top-level declarations one-to-one.

# ---- dlopen fixture plugins (tests/modules) ----
ayanami_add_test_plugin(dummy_planner_module
  modules/dummy_planner_module.cpp)
ayanami_add_test_plugin(dummy_extension_module
  modules/dummy_extension_module.cpp)
ayanami_add_test_plugin(robot_state_dso_extension
  modules/robot_state_dso_extension.cpp)
ayanami_add_test_plugin(echo_extension
  modules/echo_extension.cpp)

ayanami_add_test_plugin(test_planner_module
  modules/test_planner_module.cpp)
# Interface-version guard: a genuine AbstractPlanner reporting a stale version.
ayanami_add_test_plugin(test_stale_api_planner_module
  modules/test_stale_api_planner_module.cpp)
ayanami_add_test_plugin(test_smoother_module
  modules/test_smoother_module.cpp)
ayanami_add_test_plugin(test_controller_module
  modules/test_controller_module.cpp)
ayanami_add_test_plugin(test_recovery_module
  modules/test_recovery_module.cpp)
# Exception injection: a recovery whose start() throws.

# Each .so has a single public create_extension_module factory; typing is done
# by ModuleManager dynamic_cast (costmap family / plain abstract family).



# ---- shared paths ----
set(_config_dir ${PROJECT_SOURCE_DIR}/config)

# ---- core utility / messages / state ----
ayanami_add_test(config_test SOURCES config_test.cpp)
ayanami_add_test(logging_test SOURCES logging_test.cpp)
ayanami_add_test(callback_slot_test SOURCES callback_slot_test.cpp)
ayanami_add_test(messages_test SOURCES messages_test.cpp)
ayanami_add_test(robot_state_controller_test
  SOURCES robot_state_controller_test.cpp)
# TF-query performance benchmark (10 edges @ 500 Hz; gates: success > 98%,
# mean/p99 latency < 3 ms).
ayanami_add_test(robot_state_controller_bench
  SOURCES robot_state_controller_bench.cpp)
# Cross-DSO singleton: instance() is defined in a libayanami .cpp, so
# dlopen'ed plugins and the main binary share one instance.
ayanami_add_test(robot_state_controller_cross_dso_test
  SOURCES robot_state_controller_cross_dso_test.cpp
  NO_MAIN
  ARGS $<TARGET_FILE:robot_state_dso_extension>)
ayanami_add_test(load_module_test
  SOURCES load_module_test.cpp
  NO_MAIN
  ARGS $<TARGET_FILE:dummy_planner_module>
       $<TARGET_FILE:dummy_extension_module>)
# Cross-.so echo plugin: added to a NavigationCallbacks slot after dlopen.
ayanami_add_test(callbacks_test
  SOURCES callbacks_test.cpp
  NO_MAIN
  ARGS $<TARGET_FILE:echo_extension>)

# ---- abstract components / server ----
ayanami_add_test(core_interfaces_test SOURCES core_interfaces_test.cpp)
ayanami_add_test(execution_test SOURCES execution_test.cpp)
ayanami_add_test(module_manager_test
  SOURCES module_manager_test.cpp
  NO_MAIN
  ARGS $<TARGET_FILE:dummy_planner_module>
       $<TARGET_FILE:dummy_extension_module>
       $<TARGET_FILE:test_stale_api_planner_module>)
# Derived destructors callable through a base pointer (virtual ~ regression).
ayanami_add_test(module_teardown_no_ub_test
  SOURCES module_teardown_no_ub_test.cpp)
ayanami_add_test(costmap_2d_test
  SOURCES costmap_2d_test.cpp
  DEPS costmap::costmap)
ayanami_add_test(costmap_layers_test
  SOURCES costmap_layers_test.cpp
  DEPS costmap::costmap)
ayanami_add_test(costmap_server_test
  SOURCES costmap_server_test.cpp
  DEPS costmap::costmap)

# ---- costmap core / components ----
ayanami_add_test(costmap_context_test
  SOURCES costmap_context_test.cpp
  DEPS costmap::costmap)
ayanami_add_test(costmap_components_test
  SOURCES costmap_components_test.cpp
  DEPS costmap::costmap)
# Controller per-cycle short lock: no obstacle-feed starvation.
ayanami_add_test(concurrent_controller_obstacle_test
  SOURCES concurrent_controller_obstacle_test.cpp
  DEPS costmap::costmap
  TIMEOUT 10)
ayanami_add_test(recovery_vs_obstacle_test
  SOURCES recovery_vs_obstacle_test.cpp
  DEPS costmap_cleanup_recovery
       costmap::costmap
       costmap::costmap
  TIMEOUT 10)
ayanami_add_test(snapshot_stability_test
  SOURCES snapshot_stability_test.cpp
  DEPS costmap::costmap
  TIMEOUT 10)
ayanami_add_test(planner_writer_concurrency_test
  SOURCES planner_writer_concurrency_test.cpp
  DEPS costmap::costmap
  TIMEOUT 10)
ayanami_add_test(minor_carry_wirings_test
  SOURCES minor_carry_wirings_test.cpp
  DEPS costmap_astar_planner
       costmap_pid_controller
       costmap::costmap
       costmap::costmap)

# ---- production modules ----
ayanami_add_test(costmap_astar_planner_test
  SOURCES costmap_astar_planner_test.cpp
  DEPS costmap_astar_planner costmap::costmap
  ARGS $<TARGET_FILE:costmap_astar_planner>)
ayanami_add_test(costmap_layers_static_test
  SOURCES costmap_layers_static_test.cpp)
ayanami_add_test(costmap_minsnap_smoother_test
  SOURCES costmap_minsnap_smoother_test.cpp
  DEPS costmap_minsnap_smoother costmap::costmap
  INCLUDES ${PROJECT_SOURCE_DIR}/modules/costmap_minsnap_smoother
  TIMEOUT 30
  ARGS $<TARGET_FILE:costmap_minsnap_smoother>)
ayanami_add_test(costmap_pid_controller_test
  SOURCES costmap_pid_controller_test.cpp
  DEPS costmap_pid_controller simple_goal_checker
  ARGS $<TARGET_FILE:costmap_pid_controller>)
ayanami_add_test(costmap_mppi_controller_test
  SOURCES costmap_mppi_controller_test.cpp
  DEPS costmap_mppi_controller costmap::costmap
  INCLUDES ${PROJECT_SOURCE_DIR}/modules/costmap_mppi_controller
  TIMEOUT 60
  ARGS $<TARGET_FILE:costmap_mppi_controller>)
ayanami_add_test(costmap_neupan_controller_test
  SOURCES costmap_neupan_controller_test.cpp
  DEPS costmap_neupan_controller costmap::costmap neupan
  INCLUDES ${PROJECT_SOURCE_DIR}/modules/costmap_neupan_controller
  DEFINES AYANAMI_NEUPAN_MODEL_DIR="${PROJECT_SOURCE_DIR}/packages/neupan/models/diff_sentry.bin"
  TIMEOUT 60
  ARGS $<TARGET_FILE:costmap_neupan_controller>)
ayanami_add_test(fsm_e2e_test
  SOURCES fsm_e2e_test.cpp
  DEPS rose_navigation_fsm costmap::costmap esdfmap::esdfmap
  NEEDS costmap_astar_planner
        esdfmap_minco_smoother
        abstract_pid_controller
        abstract_nothing_recovery
        simple_goal_checker
        simple_progress_checker
  ARGS $<TARGET_FILE:rose_navigation_fsm>
       $<TARGET_FILE:costmap_astar_planner>
       $<TARGET_FILE:esdfmap_minco_smoother>
       $<TARGET_FILE:abstract_pid_controller>
       $<TARGET_FILE:abstract_nothing_recovery>
       $<TARGET_FILE:simple_goal_checker>
       $<TARGET_FILE:simple_progress_checker>
       ${PROJECT_SOURCE_DIR}/tests/assets)
# CostmapNavigationFsm headless end-to-end: the FSM host over the full
# costmap module set.
ayanami_add_test(fsm_costmap_e2e_test
  SOURCES fsm_costmap_e2e_test.cpp
  DEPS costmap_navigation_fsm costmap::costmap
  NEEDS costmap_astar_planner
        costmap_minsnap_smoother
        costmap_pid_controller
        abstract_nothing_recovery
        simple_goal_checker
        simple_progress_checker
  ARGS $<TARGET_FILE:costmap_navigation_fsm>
       $<TARGET_FILE:costmap_astar_planner>
       $<TARGET_FILE:costmap_minsnap_smoother>
       $<TARGET_FILE:costmap_pid_controller>
       $<TARGET_FILE:abstract_nothing_recovery>
       $<TARGET_FILE:simple_goal_checker>
       $<TARGET_FILE:simple_progress_checker>
       ${PROJECT_SOURCE_DIR}/tests/assets)
ayanami_add_test(fsm_neupan_e2e_test
  SOURCES fsm_neupan_e2e_test.cpp
  DEPS costmap_navigation_fsm costmap_neupan_controller costmap::costmap
  NEEDS costmap_astar_planner costmap_minsnap_smoother abstract_nothing_recovery simple_goal_checker simple_progress_checker
  DEFINES AYANAMI_NEUPAN_MODEL_DIR="${PROJECT_SOURCE_DIR}/packages/neupan/models/diff_sentry.bin"
  TIMEOUT 120
  ARGS $<TARGET_FILE:costmap_navigation_fsm>
       $<TARGET_FILE:costmap_astar_planner>
       $<TARGET_FILE:costmap_minsnap_smoother>
       $<TARGET_FILE:costmap_neupan_controller>
       $<TARGET_FILE:abstract_nothing_recovery>
       $<TARGET_FILE:simple_goal_checker>
       $<TARGET_FILE:simple_progress_checker>
       ${PROJECT_SOURCE_DIR}/tests/assets
       ${PROJECT_SOURCE_DIR}/packages/neupan/models/diff_sentry.bin)
ayanami_add_test(local_window_rolling_test
  SOURCES local_window_rolling_test.cpp)

# ---- rosemap family ----
ayanami_add_test(esdfmap_test
  SOURCES esdfmap_test.cpp
  DEPS esdfmap::esdfmap)

# ---- esdfmap_minco_smoother (dlopen the production module .so) ----
ayanami_add_test(esdfmap_minco_smoother_test
  SOURCES esdfmap_minco_smoother_test.cpp
  DEPS esdfmap::esdfmap
  TIMEOUT 30)

# ---- FsmDriver (in-test mocks, no dlopen) ----
ayanami_add_test(fsm_test
  SOURCES fsm_test.cpp)
